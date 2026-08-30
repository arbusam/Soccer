#!/usr/bin/env python3
"""Train YOLO26 detect at nano/small/medium, then compile nano + small to Hailo HEF.

Each stage runs as a separate subprocess so a crashed/OOM training run for one
size does not take the rest of the pipeline down, and so torch device state is
never reused across sizes.

Example:
  python training/build_all.py --epochs 100 --hw-arch hailo8
  python training/build_all.py --batch-n 16 --batch-s 8 --batch-m 4
  python training/build_all.py --skip-train        # only compile existing ONNX
  python training/build_all.py --skip-hailo        # only train/export ONNX
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

TRAINING_ROOT = Path(__file__).resolve().parent
REPO_ROOT = TRAINING_ROOT.parent
TRAIN_PYTHON = REPO_ROOT / ".venv" / "bin" / "python"
HAILO_PYTHON = Path.home() / "hailo-dfc-venv" / "bin" / "python"
STATE_PATH = TRAINING_ROOT / ".build_all_state.json"

TRAIN_SIZES = ("n", "s", "m")
HAILO_SIZES = ("n", "s")


def _run(cmd: list[str]) -> None:
    print("\n$ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True, cwd=str(REPO_ROOT))


def _require_python(path: Path, purpose: str) -> None:
    if not path.is_file():
        raise SystemExit(f"{purpose} Python interpreter not found: {path}")


def _save_state(state: dict[str, object]) -> None:
    temporary = STATE_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    temporary.replace(STATE_PATH)


def _load_or_create_state(resume: bool) -> dict[str, object]:
    if resume:
        if not STATE_PATH.is_file():
            raise SystemExit(
                f"--resume requested, but no interrupted build state exists at {STATE_PATH}"
            )
        try:
            state = json.loads(STATE_PATH.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError) as exc:
            raise SystemExit(f"Cannot read build state {STATE_PATH}: {exc}") from exc
        print(f"Resuming interrupted build started at {state['started_at']}")
        return state

    state: dict[str, object] = {
        "started_at": time.time(),
        "completed": [],
        "current": None,
    }
    _save_state(state)
    return state


def _stage_completed(state: dict[str, object], stage: str) -> bool:
    return stage in state["completed"]


def _start_stage(state: dict[str, object], stage: str) -> None:
    state["current"] = stage
    _save_state(state)


def _complete_stage(state: dict[str, object], stage: str) -> None:
    completed = state["completed"]
    if stage not in completed:
        completed.append(stage)
    state["current"] = None
    _save_state(state)


def _batch_for_size(size: str, args: argparse.Namespace) -> int:
    """Per-size override, else the shared --batch default."""
    override = getattr(args, f"batch_{size}", None)
    return args.batch if override is None else override


def train_size(
    size: str, args: argparse.Namespace, export_dataset: bool, resume: bool
) -> None:
    cmd = [
        str(TRAIN_PYTHON),
        str(TRAINING_ROOT / "train.py"),
        "--size",
        size,
        "--epochs",
        str(args.epochs),
        "--patience",
        str(args.patience),
        "--imgsz",
        str(args.imgsz),
        "--batch",
        str(_batch_for_size(size, args)),
        "--device",
        args.device,
        "--workers",
        str(args.workers),
    ]
    if export_dataset:
        cmd.append("--export-dataset")
    if args.no_mosaic:
        cmd.append("--no-mosaic")
    if args.amp:
        cmd.append("--amp")
    if resume:
        cmd.append("--resume")
    _run(cmd)


def compile_size(size: str, args: argparse.Namespace) -> None:
    onnx = TRAINING_ROOT / "exports" / f"open-soccer-detect-{size}" / "model.onnx"
    if not onnx.is_file():
        raise SystemExit(f"ONNX missing for size {size}: {onnx}")
    out_dir = REPO_ROOT / f"open-soccer-detect-{size}_hailo_model"
    _run(
        [
            str(HAILO_PYTHON),
            str(TRAINING_ROOT / "compile_hailo.py"),
            "--onnx",
            str(onnx),
            "--out-dir",
            str(out_dir),
            "--hw-arch",
            args.hw_arch,
            "--imgsz",
            str(args.imgsz),
            "--calib-images",
            str(args.calib_images),
            "--model-name",
            f"open_soccer_detect_{size}",
        ]
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train nano/small/medium detect models and compile nano + small for Hailo."
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument(
        "--patience",
        type=int,
        default=50,
        help="Epochs without val improvement before stopping (0 disables). Passed to train.py.",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument(
        "--batch",
        type=int,
        default=8,
        help="Default training batch size for every model size (-1 = AutoBatch)",
    )
    for size in TRAIN_SIZES:
        parser.add_argument(
            f"--batch-{size}",
            type=int,
            default=None,
            dest=f"batch_{size}",
            help=f"Batch size for YOLO26{size} (default: same as --batch)",
        )
    parser.add_argument("--device", default="xpu")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--no-mosaic", action="store_true")
    parser.add_argument("--amp", action="store_true")
    parser.add_argument(
        "--export-dataset",
        action="store_true",
        help="Re-export the dataset from Label Studio before the first training run",
    )
    parser.add_argument("--hw-arch", default="hailo8", choices=("hailo8", "hailo8l"))
    parser.add_argument("--calib-images", type=int, default=64)
    parser.add_argument("--skip-train", action="store_true")
    parser.add_argument("--skip-hailo", action="store_true")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip completed stages and resume interrupted training checkpoints",
    )
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Continue with remaining sizes when one stage fails",
    )
    args = parser.parse_args()

    state = _load_or_create_state(args.resume)
    failures: list[str] = []

    if not args.skip_train:
        _require_python(TRAIN_PYTHON, "Training")
        for index, size in enumerate(TRAIN_SIZES):
            stage = f"train:{size}"
            if _stage_completed(state, stage):
                print(f"\nSkipping stage completed by the interrupted build: {stage}")
                continue
            checkpoint = (
                TRAINING_ROOT
                / "runs"
                / f"open-soccer-detect-{size}"
                / "weights"
                / "last.pt"
            )
            resume_checkpoint = (
                args.resume
                and checkpoint.is_file()
                and checkpoint.stat().st_mtime >= float(state["started_at"])
            )
            try:
                _start_stage(state, stage)
                train_size(
                    size,
                    args,
                    export_dataset=args.export_dataset and index == 0,
                    resume=resume_checkpoint,
                )
                _complete_stage(state, stage)
            except subprocess.CalledProcessError as exc:
                if not args.keep_going:
                    raise SystemExit(f"Training failed for size {size} (exit {exc.returncode})")
                failures.append(stage)

    if not args.skip_hailo:
        _require_python(HAILO_PYTHON, "Hailo")
        for size in HAILO_SIZES:
            stage = f"hailo:{size}"
            if _stage_completed(state, stage):
                print(f"\nSkipping stage completed by the interrupted build: {stage}")
                continue
            try:
                _start_stage(state, stage)
                compile_size(size, args)
                _complete_stage(state, stage)
            except (subprocess.CalledProcessError, SystemExit) as exc:
                if not args.keep_going:
                    raise SystemExit(f"Hailo compile failed for size {size}: {exc}")
                failures.append(stage)

    if failures:
        print("\nCompleted with failures: " + ", ".join(failures), file=sys.stderr)
        raise SystemExit(1)
    STATE_PATH.unlink()
    print("\nAll stages complete.")


if __name__ == "__main__":
    main()
