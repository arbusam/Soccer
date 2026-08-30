"""Compatibility shim to let Ultralytics 8.4.78 train on Intel GPUs (torch XPU).

This version of Ultralytics has no native ``device="xpu"`` support: ``select_device``
only understands CUDA/MPS/CPU, and the trainer's memory helpers call ``torch.cuda.*``
unconditionally. Importing this module and calling :func:`enable_xpu_support` patches
those spots at runtime so training runs on an Intel Arc GPU.

The patches are applied to the imported module objects only; ``site-packages`` is left
untouched so a reinstall or upgrade simply makes this shim unnecessary.

``batch=-1`` (AutoBatch) also needs this shim: stock Ultralytics always calls
``torch.cuda.get_device_properties``, which crashes an XPU-only torch wheel.
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


def _xpu_autobatch(
    model,
    imgsz: int,
    fraction: float,
    batch_size: int,
    max_num_obj: int,
    dataset_size: int,
) -> int:
    """Ultralytics AutoBatch using torch.xpu instead of torch.cuda."""
    import numpy as np
    from ultralytics.utils import LOGGER, colorstr
    from ultralytics.utils.torch_utils import profile_ops

    prefix = colorstr("AutoBatch: ")
    LOGGER.info(
        f"{prefix}Computing optimal batch size for imgsz={imgsz} at "
        f"{fraction * 100:.0f}% XPU memory utilization."
    )
    device = next(model.parameters()).device
    gb = 1 << 30
    d = f"XPU:{device.index}"
    properties = torch.xpu.get_device_properties(device)
    t = properties.total_memory / gb
    r = torch.xpu.memory_reserved(device) / gb
    a = torch.xpu.memory_allocated(device) / gb
    f = t - (r + a)
    LOGGER.info(
        f"{prefix}{d} ({properties.name}) {t:.2f}G total, {r:.2f}G reserved, "
        f"{a:.2f}G allocated, {f:.2f}G free"
    )

    batch_sizes = [1, 2, 4, 8, 16] if t < 16 else [1, 2, 4, 8, 16, 32, 64]
    if dataset_size > 0:
        batch_sizes = [b for b in batch_sizes if b <= dataset_size]
    ch = model.yaml.get("channels", 3)
    try:
        img = [torch.empty(b, ch, imgsz, imgsz) for b in batch_sizes]
        results = profile_ops(img, model, n=1, device=device, max_num_obj=max_num_obj)
        xy = [
            [x, y[2]]
            for i, (x, y) in enumerate(zip(batch_sizes, results))
            if y
            and isinstance(y[2], (int, float))
            and 0 < y[2] < t
            and (i == 0 or not results[i - 1] or y[2] > results[i - 1][2])
        ]
        fit_x, fit_y = zip(*xy) if xy else ([], [])
        p = np.polyfit(fit_x, fit_y, deg=1)
        b = int((round(f * fraction) - p[1]) / p[0])
        if None in results:
            i = results.index(None)
            if b >= batch_sizes[i]:
                b = batch_sizes[max(i - 1, 0)]
        if b < 1 or b > 1024:
            LOGGER.warning(
                f"{prefix}batch={b} outside safe range, using default batch-size {batch_size}."
            )
            b = batch_size
        if dataset_size > 0:
            b = min(b, dataset_size)
        predicted = (np.polyval(p, b) + r + a) / t
        LOGGER.info(
            f"{prefix}Using batch-size {b} for {d} {t * predicted:.2f}G/{t:.2f}G "
            f"({predicted * 100:.0f}%)"
        )
        return b
    except Exception as exc:
        LOGGER.warning(
            f"{prefix}error detected: {exc}, using default batch-size {batch_size}."
        )
        return batch_size
    finally:
        torch.xpu.empty_cache()


def _make_autobatch(orig):
    def autobatch(model, imgsz=640, fraction=0.60, batch_size=16, max_num_obj=1, dataset_size=0):
        device = next(model.parameters()).device
        if device.type == "xpu":
            return _xpu_autobatch(
                model, imgsz, fraction, batch_size, max_num_obj, dataset_size
            )
        return orig(
            model,
            imgsz=imgsz,
            fraction=fraction,
            batch_size=batch_size,
            max_num_obj=max_num_obj,
            dataset_size=dataset_size,
        )

    return autobatch


def enable_xpu_support() -> None:
    """Patch Ultralytics so ``device="xpu"`` trains on an Intel GPU.

    Safe to call multiple times. No-op if the installed Ultralytics already supports XPU.
    """
    from ultralytics.engine import trainer as trainer_mod
    from ultralytics.utils import autobatch as autobatch_mod
    from ultralytics.utils import torch_utils

    if getattr(torch_utils, "_xpu_shim_applied", False):
        return

    wrapped = _make_select_device(torch_utils.select_device)
    torch_utils.select_device = wrapped
    # select_device is imported by name into the trainer module, so patch that ref too.
    trainer_mod.select_device = wrapped

    trainer_mod.BaseTrainer._get_memory = _xpu_get_memory
    trainer_mod.BaseTrainer._clear_memory = _xpu_clear_memory
    autobatch_mod.autobatch = _make_autobatch(autobatch_mod.autobatch)

    torch_utils._xpu_shim_applied = True
