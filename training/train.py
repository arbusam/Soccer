#!/usr/bin/env python3
"""Train YOLO26-OBB on Label Studio exports and export the best weights to NCNN.

Defaults to the Intel Arc GPU via PyTorch XPU (same setup as ~/training).
Nano and small (and larger) sizes keep separate run/export directories so they coexist.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

TRAINING_ROOT = Path(__file__).resolve().parent
if str(TRAINING_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAINING_ROOT))

MODEL_SIZES = ("n", "s", "m", "l", "x")
DEFAULT_SIZE = "n"
DEFAULT_DATA_DIR = TRAINING_ROOT.parent / "label-studio-data"
DEFAULT_OUT_DIR = TRAINING_ROOT / "datasets" / "open-soccer-obb"
DEFAULT_DEVICE = "xpu"


def paths_for_size(size: str) -> tuple[str, str, Path, Path]:
    """Return (model_ckpt, run_name, export_dir, best_weights) for a YOLO26-OBB size."""
    if size not in MODEL_SIZES:
        raise ValueError(f"size must be one of {MODEL_SIZES}, got {size!r}")
    model = f"yolo26{size}-obb.pt"
    name = f"open-soccer-obb-{size}"
    export_dir = TRAINING_ROOT / "exports" / f"{name}_ncnn_model"
    best_pt = TRAINING_ROOT / "runs" / name / "weights" / "best.pt"
    return model, name, export_dir, best_pt


def _ensure_dataset(data_yaml: Path, data_dir: Path, project_id: int, force_export: bool) -> Path:
    from export_label_studio import export_dataset

    if force_export or not data_yaml.is_file():
        out_dir = data_yaml.parent if data_yaml.name == "data.yaml" else DEFAULT_OUT_DIR
        return export_dataset(
            data_dir=data_dir,
            out_dir=out_dir,
            project_id=project_id,
            val_fraction=0.2,
            seed=42,
        )
    return data_yaml


def _is_xpu(device: str) -> bool:
    name = device.strip().lower()
    return name == "xpu" or name.startswith("xpu:")


def _prepare_device(device: str, amp: bool) -> tuple[str, bool]:
    """Validate device and enable the Ultralytics XPU shim when needed."""
    import torch

    name = device.strip().lower()
    if _is_xpu(device):
        if not hasattr(torch, "xpu") or not torch.xpu.is_available():
            raise RuntimeError(
                "PyTorch XPU is not available. Use the Intel XPU torch build "
                "(same as ~/training), e.g.\n"
                "  /home/arhan/training/.venv/bin/python training/train.py ...\n"
                "or: pip install torch torchvision "
                "--index-url https://download.pytorch.org/whl/xpu"
            )
        from xpu_patch import enable_xpu_support

        enable_xpu_support()
        index = int(name.split(":", 1)[1]) if ":" in name else 0
        print(f"Using XPU device: {torch.xpu.get_device_name(index)}")
        if amp:
            print(
                "Warning: AMP is not supported on Intel XPU with this Ultralytics "
                "shim (GradScaler is CUDA-only). Disabling AMP for this run."
            )
            amp = False
        return name, amp

    if name in {"0", "cuda", "cuda:0"} or name.startswith("cuda:"):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but torch.cuda.is_available() is False.")
        return device, amp

    return device, amp


def export_ncnn(
    weights: Path,
    export_dir: Path,
    imgsz: int,
    quantize: int | None,
) -> Path:
    from ultralytics import YOLO

    # NCNN conversion via PNNX is more reliable on CPU.
    model = YOLO(str(weights))
    export_kwargs: dict = {"format": "ncnn", "imgsz": imgsz, "device": "cpu"}
    if quantize is not None:
        export_kwargs["quantize"] = quantize

    ncnn_path = Path(model.export(**export_kwargs))
    if export_dir.exists():
        shutil.rmtree(export_dir)
    export_dir.parent.mkdir(parents=True, exist_ok=True)
    if ncnn_path.is_dir():
        shutil.copytree(ncnn_path, export_dir)
    else:
        export_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ncnn_path, export_dir / ncnn_path.name)

    print(f"NCNN model:   {export_dir}")
    return export_dir


def train_and_export(
    data_yaml: Path,
    model_name: str,
    epochs: int,
    imgsz: int,
    batch: int,
    device: str,
    workers: int,
    project: Path,
    name: str,
    export_dir: Path,
    quantize: int | None,
    amp: bool,
    skip_export: bool,
    resume: bool,
    mosaic: float,
) -> Path | None:
    device, amp = _prepare_device(device, amp)
    from ultralytics import YOLO

    model = YOLO(model_name)
    train_kwargs: dict = {
        "data": str(data_yaml),
        "epochs": epochs,
        "imgsz": imgsz,
        "batch": batch,
        "workers": workers,
        "project": str(project),
        "name": name,
        "exist_ok": True,
        "device": device,
        "amp": amp,
        "mosaic": mosaic,
        "resume": resume,
    }
    results = model.train(**train_kwargs)
    best_pt = Path(results.save_dir) / "weights" / "best.pt"
    if not best_pt.is_file():
        raise FileNotFoundError(f"Training finished but best weights missing: {best_pt}")
    print(f"Best weights: {best_pt}")

    if skip_export:
        return None
    return export_ncnn(best_pt, export_dir, imgsz, quantize)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train YOLO26-OBB from Label Studio on Intel XPU and export NCNN."
    )
    parser.add_argument(
        "--size",
        choices=MODEL_SIZES,
        default=DEFAULT_SIZE,
        help="YOLO26-OBB size: n/s/m/l/x (default: n). Each size has its own runs/ and exports/.",
    )
    parser.add_argument(
        "--data",
        type=Path,
        default=DEFAULT_OUT_DIR / "data.yaml",
        help="Path to dataset data.yaml (exports from Label Studio if missing)",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help="Label Studio data directory used when exporting",
    )
    parser.add_argument("--project-id", type=int, default=2)
    parser.add_argument(
        "--export-dataset",
        action="store_true",
        help="Re-export the YOLO-OBB dataset from Label Studio before training",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Override base checkpoint (default: yolo26{size}-obb.pt)",
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help="Training device (default: xpu, same as ~/training). Examples: xpu, xpu:0, cpu",
    )
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--mosaic",
        type=float,
        default=1.0,
        help="Mosaic probability (default: 1.0). Use --no-mosaic to disable.",
    )
    parser.add_argument(
        "--no-mosaic",
        action="store_true",
        help="Disable mosaic augmentation (workaround for XPU scatter/gather crashes).",
    )
    parser.add_argument(
        "--amp",
        action="store_true",
        help="Enable mixed precision (not supported on Intel XPU with the current shim)",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume from training/runs/<name>/weights/last.pt",
    )
    parser.add_argument(
        "--project",
        type=Path,
        default=TRAINING_ROOT / "runs",
        help="Ultralytics runs directory",
    )
    parser.add_argument(
        "--name",
        default=None,
        help="Run name (default: open-soccer-obb-{size})",
    )
    parser.add_argument(
        "--export-dir",
        type=Path,
        default=None,
        help="NCNN output dir (default: training/exports/open-soccer-obb-{size}_ncnn_model)",
    )
    parser.add_argument(
        "--quantize",
        type=int,
        default=None,
        choices=(16, 32),
        help="NCNN export precision (16=FP16, 32=FP32). Default: Ultralytics default.",
    )
    parser.add_argument(
        "--skip-export",
        action="store_true",
        help="Train only; do not export NCNN",
    )
    parser.add_argument(
        "--export-only",
        nargs="?",
        const="",
        default=None,
        help=(
            "Skip training; export existing weights to NCNN. "
            "Optional path (default: runs/open-soccer-obb-{size}/weights/best.pt)"
        ),
    )
    args = parser.parse_args()

    default_model, default_name, default_export, default_best = paths_for_size(args.size)
    model_name = args.model or default_model
    run_name = args.name or default_name
    export_dir = (args.export_dir or default_export).resolve()

    print(f"Size={args.size}  model={model_name}  run={run_name}  export={export_dir}")

    if args.export_only is not None:
        weights = Path(args.export_only).resolve() if args.export_only else default_best.resolve()
        if not weights.is_file():
            # Fall back to pre-size-suffix nano run/export if present.
            legacy = TRAINING_ROOT / "runs" / "open-soccer-obb" / "weights" / "best.pt"
            if args.size == "n" and not args.export_only and legacy.is_file():
                weights = legacy.resolve()
                print(f"Using legacy nano weights: {weights}")
            else:
                raise SystemExit(f"Weights not found: {weights}")
        export_ncnn(
            weights=weights,
            export_dir=export_dir,
            imgsz=args.imgsz,
            quantize=args.quantize,
        )
        return

    data_yaml = _ensure_dataset(
        data_yaml=args.data.resolve(),
        data_dir=args.data_dir.resolve(),
        project_id=args.project_id,
        force_export=args.export_dataset,
    )

    train_model = model_name
    if args.resume:
        last_pt = args.project / run_name / "weights" / "last.pt"
        if not last_pt.is_file() and args.size == "n":
            legacy_last = args.project / "open-soccer-obb" / "weights" / "last.pt"
            if legacy_last.is_file():
                last_pt = legacy_last
                run_name = "open-soccer-obb"
                print(f"Resuming legacy nano run: {last_pt}")
        if not last_pt.is_file():
            raise SystemExit(f"--resume requested but checkpoint missing: {last_pt}")
        train_model = str(last_pt.resolve())

    mosaic = 0.0 if args.no_mosaic else args.mosaic
    train_and_export(
        data_yaml=data_yaml,
        model_name=train_model,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=args.project.resolve(),
        name=run_name,
        export_dir=export_dir,
        quantize=args.quantize,
        amp=args.amp,
        skip_export=args.skip_export,
        resume=args.resume,
        mosaic=mosaic,
    )


if __name__ == "__main__":
    main()
