"""Compatibility shim to let Ultralytics 8.4.78 train on Intel GPUs (torch XPU).

This version of Ultralytics has no native ``device="xpu"`` support: ``select_device``
only understands CUDA/MPS/CPU, and the trainer's memory helpers call ``torch.cuda.*``
unconditionally. Importing this module and calling :func:`enable_xpu_support` patches
those spots at runtime so training runs on an Intel Arc GPU.

The patches are applied to the imported module objects only; ``site-packages`` is left
untouched so a reinstall or upgrade simply makes this shim unnecessary.
"""

from __future__ import annotations

import gc

import torch


def _make_select_device(orig):
    def select_device(device="", *args, **kwargs):
        name = str(device).strip().lower()
        if name == "xpu" or name.startswith("xpu:"):
            if not (hasattr(torch, "xpu") and torch.xpu.is_available()):
                raise RuntimeError(
                    "device='xpu' requested but torch.xpu is not available. "
                    "Install torch from https://download.pytorch.org/whl/xpu and check the GPU driver."
                )
            index = int(name.split(":", 1)[1]) if ":" in name else 0
            torch.xpu.set_device(index)
            return torch.device("xpu", index)
        return orig(device, *args, **kwargs)

    return select_device


def _xpu_get_memory(self, fraction=False):
    """Drop-in replacement for BaseTrainer._get_memory that understands XPU."""
    memory, total = 0, 0
    device = self.device
    if device.type == "mps":
        memory = torch.mps.driver_allocated_memory()
        if fraction:
            import psutil

            return psutil.virtual_memory().percent / 100
    elif device.type == "xpu":
        memory = torch.xpu.memory_reserved(device)
        if fraction:
            total = torch.xpu.get_device_properties(device).total_memory
    elif device.type != "cpu":
        memory = torch.cuda.memory_reserved()
        if fraction:
            total = torch.cuda.get_device_properties(device).total_memory
    return ((memory / total) if total > 0 else 0) if fraction else (memory / 2**30)


def _xpu_clear_memory(self, threshold=None):
    """Drop-in replacement for BaseTrainer._clear_memory that understands XPU."""
    if threshold:
        assert 0 <= threshold <= 1, "Threshold must be between 0 and 1."
        if self._get_memory(fraction=True) <= threshold:
            return
    gc.collect()
    device = self.device
    if device.type == "mps":
        torch.mps.empty_cache()
    elif device.type == "xpu":
        torch.xpu.empty_cache()
    elif device.type == "cpu":
        return
    else:
        torch.cuda.empty_cache()


def enable_xpu_support() -> None:
    """Patch Ultralytics so ``device="xpu"`` trains on an Intel GPU.

    Safe to call multiple times. No-op if the installed Ultralytics already supports XPU.
    """
    from ultralytics.engine import trainer as trainer_mod
    from ultralytics.utils import torch_utils

    if getattr(torch_utils, "_xpu_shim_applied", False):
        return

    wrapped = _make_select_device(torch_utils.select_device)
    torch_utils.select_device = wrapped
    # select_device is imported by name into the trainer module, so patch that ref too.
    trainer_mod.select_device = wrapped

    trainer_mod.BaseTrainer._get_memory = _xpu_get_memory
    trainer_mod.BaseTrainer._clear_memory = _xpu_clear_memory

    torch_utils._xpu_shim_applied = True
