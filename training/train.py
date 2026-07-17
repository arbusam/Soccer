#!/usr/bin/env python3
"""Train YOLO26 detect on Label Studio exports and export ONNX for Hailo compilation.

Defaults to the Intel Arc GPU via PyTorch XPU (project .venv).
Produces training/exports/open-soccer-detect-{size}/model.onnx for compile_hailo.py.
"""

from __future__ import annotations

import argparse
import ast
import shutil
import sys
from pathlib import Path

TRAINING_ROOT = Path(__file__).resolve().parent
if str(TRAINING_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAINING_ROOT))

MODEL_SIZES = ("n", "s", "m", "l", "x")
DEFAULT_SIZE = "n"
DEFAULT_DATA_DIR = TRAINING_ROOT.parent / "label-studio-data"
DEFAULT_OUT_DIR = TRAINING_ROOT / "datasets" / "open-soccer-detect"
DEFAULT_DEVICE = "xpu"


def paths_for_size(size: str) -> tuple[str, str, Path, Path]:
    if size not in MODEL_SIZES:
        raise ValueError(f"size must be one of {MODEL_SIZES}, got {size!r}")
    model = f"yolo26{size}.pt"
    name = f"open-soccer-detect-{size}"
    export_dir = TRAINING_ROOT / "exports" / name
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
    import torch

    name = device.strip().lower()
    if _is_xpu(device):
        if not hasattr(torch, "xpu") or not torch.xpu.is_available():
            raise RuntimeError(
                "PyTorch XPU is not available in this environment. "
                "Use the project .venv (torch XPU build) or pass --device cpu."
            )
        from xpu_patch import enable_xpu_support

        enable_xpu_support()
        index = int(name.split(":", 1)[1]) if ":" in name else 0
        print(f"Using XPU device: {torch.xpu.get_device_name(index)}")
        if amp:
            print(
                "Warning: AMP is not supported on Intel XPU with this Ultralytics "
                "shim. Disabling AMP for this run."
            )
            amp = False
        return name, amp

    if name in {"0", "cuda", "cuda:0"} or name.startswith("cuda:"):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but torch.cuda.is_available() is False.")
        return device, amp

    return device, amp


def export_onnx(weights: Path, export_dir: Path, imgsz: int) -> Path:
    """Export YOLO26 detect weights to ONNX (opset 11) plus metadata.yaml for Hailo."""
    import onnx
    from ultralytics import YOLO
    from ultralytics.utils import YAML

    model = YOLO(str(weights))
    onnx_path = Path(
        model.export(format="onnx", imgsz=imgsz, opset=11, simplify=True, device="cpu")
    )

    if export_dir.exists():
        shutil.rmtree(export_dir)
    export_dir.mkdir(parents=True, exist_ok=True)

    dest_onnx = export_dir / "model.onnx"
    shutil.move(str(onnx_path), dest_onnx)

    meta = {p.key: p.value for p in onnx.load(dest_onnx, load_external_data=False).metadata_props}
    for key in ("stride", "batch", "channels"):
        if key in meta:
            meta[key] = int(meta[key])
    for key in ("imgsz", "names", "args", "end2end"):
        if key in meta:
            meta[key] = ast.literal_eval(meta[key])
    meta.setdefault("task", "detect")
    meta.setdefault(
        "names",
        {0: "Ball", 1: "Bot"},
    )
    YAML.save(export_dir / "metadata.yaml", meta)

    print(f"ONNX model:   {dest_onnx}")
    print(f"Metadata:     {export_dir / 'metadata.yaml'}")
    return dest_onnx


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
    amp: bool,
    skip_export: bool,
    resume: bool,
    mosaic: float,
) -> Path | None:
    device, amp = _prepare_device(device, amp)
    from ultralytics import YOLO

    model = YOLO(model_name)
    results = model.train(
        data=str(data_yaml),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        workers=workers,
        project=str(project),
        name=name,
        exist_ok=True,
        device=device,
        amp=amp,
        mosaic=mosaic,
        resume=resume,
    )
    best_pt = Path(results.save_dir) / "weights" / "best.pt"
    if not best_pt.is_file():
        raise FileNotFoundError(f"Training finished but best weights missing: {best_pt}")
    print(f"Best weights: {best_pt}")

    if skip_export:
        return None
    return export_onnx(best_pt, export_dir, imgsz)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train YOLO26 detect from Label Studio and export ONNX for Hailo."
    )
    parser.add_argument(
        "--size",
        choices=MODEL_SIZES,
        default=DEFAULT_SIZE,
        help="YOLO26 detect size: n/s/m/l/x (default: n)",
    )
    parser.add_argument(
        "--data",
        type=Path,
        default=DEFAULT_OUT_DIR / "data.yaml",
        help="Path to dataset data.yaml (exports detect labels from Label Studio if missing)",
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
        help="Re-export the YOLO-detect dataset from Label Studio before training",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Override base checkpoint (default: yolo26{size}.pt)",
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help="Training device (default: xpu). Examples: xpu, xpu:0, cpu, cuda:0",
    )
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--mosaic", type=float, default=1.0)
    parser.add_argument(
        "--no-mosaic",
        action="store_true",
        help="Disable mosaic augmentation (XPU scatter/gather workaround)",
    )
    parser.add_argument("--amp", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--project",
        type=Path,
        default=TRAINING_ROOT / "runs",
    )
    parser.add_argument("--name", default=None)
    parser.add_argument(
        "--export-dir",
        type=Path,
        default=None,
        help="ONNX export dir (default: training/exports/open-soccer-detect-{size})",
    )
    parser.add_argument("--skip-export", action="store_true")
    parser.add_argument(
        "--export-only",
        nargs="?",
        const="",
        default=None,
        help=(
            "Skip training; export existing weights to ONNX. "
            "Optional path (default: runs/open-soccer-detect-{size}/weights/best.pt)"
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
            raise SystemExit(f"Weights not found: {weights}")
        export_onnx(weights=weights, export_dir=export_dir, imgsz=args.imgsz)
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
        amp=args.amp,
        skip_export=args.skip_export,
        resume=args.resume,
        mosaic=mosaic,
    )


if __name__ == "__main__":
    main()
