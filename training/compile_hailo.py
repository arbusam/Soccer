#!/usr/bin/env python3
"""Compile a YOLO26 detect ONNX model to Hailo HEF (hailo8 / AI HAT+ 26 TOPS).

Requires the Hailo Dataflow Compiler (DFC) on an x86_64 Linux host — not the Pi.

YOLO26's end-to-end head (TopK / GatherElements / ReduceMax) is unsupported on
Hailo. Parsing stops at the decoded box and class-score tensors as *separate*
outputs (``/model.23/Mul_2``, ``/model.23/Sigmoid``), each ``(1, 4, 8400)`` for
nc=4. Do **not** cut at ``/model.23/Transpose`` / ``Concat_3``: that concatenates
pixel boxes with sigmoid scores into one tensor, and INT8 quantization then uses
a box-sized qp scale (~3+) that crushes every confidence to ~0.
``lib/hailo_ball.py`` merges the two streams and runs conf filter + NMS on the host.

Example:
  python training/compile_hailo.py \\
    --onnx training/exports/open-soccer-detect-n/model.onnx \\
    --calib-dir training/datasets/open-soccer-detect/images/train \\
    --out-dir open-soccer-detect-n_hailo_model
"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ONNX = REPO_ROOT / "training" / "exports" / "open-soccer-detect-n" / "model.onnx"
DEFAULT_OUT = REPO_ROOT / "open-soccer-detect-n_hailo_model"
DEFAULT_HW_ARCH = "hailo8"
DEFAULT_IMGSZ = 640
DEFAULT_CALIB_IMAGES = 64
# Separate box / score outputs so each gets its own quantization range.
# (Concat+Transpose into one (1,8400,4+nc) tensor destroys sigmoid scores.)
DEFAULT_END_NODE_NAMES = ("/model.23/Mul_2", "/model.23/Sigmoid")


def _require_dfc():
    try:
        from hailo_sdk_client import ClientRunner  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "Hailo Dataflow Compiler (hailo_sdk_client) is not installed.\n"
            "Install the DFC wheel from the Hailo Developer Zone on an x86_64 "
            "Linux machine, then re-run this script.\n"
            "Compilation cannot be done on the Raspberry Pi."
        ) from exc


def _load_calib_set(calib_dir: Path, imgsz: int, count: int) -> np.ndarray:
    from PIL import Image

    image_files = sorted(
        p
        for p in calib_dir.rglob("*")
        if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    )
    if not image_files:
        raise FileNotFoundError(f"No calibration images found under {calib_dir}")

    rng = random.Random(42)
    chosen = [rng.choice(image_files) for _ in range(count)]
    calibset = np.zeros((count, imgsz, imgsz, 3), dtype=np.float32)
    for i, path in enumerate(chosen):
        img = Image.open(path).convert("RGB").resize((imgsz, imgsz))
        calibset[i] = np.asarray(img, dtype=np.float32)
    return calibset


def compile_hef(
    onnx_path: Path,
    out_dir: Path,
    calib_dir: Path,
    hw_arch: str,
    imgsz: int,
    calib_images: int,
    model_name: str,
    end_node_names: list[str] | tuple[str, ...] = DEFAULT_END_NODE_NAMES,
) -> Path:
    _require_dfc()
    from hailo_sdk_client import ClientRunner

    if not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX not found: {onnx_path}")

    out_dir.mkdir(parents=True, exist_ok=True)
    work_onnx = out_dir / onnx_path.name
    if work_onnx.resolve() != onnx_path.resolve():
        shutil.copy2(onnx_path, work_onnx)

    meta_src = onnx_path.parent / "metadata.yaml"
    if meta_src.is_file():
        shutil.copy2(meta_src, out_dir / "metadata.yaml")

    # Input uint8 → float /255; TopK / NMS stay on the host.
    model_script = (
        "normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])\n"
    )
    alls_path = out_dir / "model_script.alls"
    alls_path.write_text(model_script, encoding="utf-8")

    end_nodes = list(end_node_names)
    print(f"Parsing ONNX with hw_arch={hw_arch}, end_node_names={end_nodes} ...")
    runner = ClientRunner(hw_arch=hw_arch)
    runner.translate_onnx_model(
        str(work_onnx),
        model_name,
        end_node_names=end_nodes,
    )
    runner.load_model_script(model_script)

    print(f"Building calibration set from {calib_dir} ({calib_images} images) ...")
    calibset = _load_calib_set(calib_dir, imgsz, calib_images)

    print("Optimizing / quantizing (this can take a while) ...")
    runner.optimize(calibset)
    har_path = out_dir / f"{model_name}.o.har"
    runner.save_har(str(har_path))

    print("Compiling HEF ...")
    hef = runner.compile()
    hef_path = out_dir / "model.hef"
    hef_path.write_bytes(hef)
    print(f"Compiled HEF: {hef_path}")
    print("Copy this folder to the Pi and point tests/model.py / lib/camera.py at it.")
    return hef_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compile YOLO26 detect ONNX → Hailo HEF (hailo8)."
    )
    parser.add_argument("--onnx", type=Path, default=DEFAULT_ONNX)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT,
        help="Deploy folder (contains model.hef + metadata.yaml)",
    )
    parser.add_argument(
        "--calib-dir",
        type=Path,
        default=None,
        help="Calibration images (default: detect dataset train images)",
    )
    parser.add_argument(
        "--hw-arch",
        default=DEFAULT_HW_ARCH,
        choices=("hailo8", "hailo8l"),
        help="hailo8 = AI HAT+ 26 TOPS; hailo8l = 13 TOPS",
    )
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    parser.add_argument("--calib-images", type=int, default=DEFAULT_CALIB_IMAGES)
    parser.add_argument("--model-name", default="open_soccer_detect_n")
    parser.add_argument(
        "--end-node-names",
        nargs="+",
        default=list(DEFAULT_END_NODE_NAMES),
        help=(
            "ONNX nodes to stop parsing at "
            "(default: /model.23/Mul_2 /model.23/Sigmoid). "
            "Keeps boxes and scores as separate outputs for correct INT8 quant; "
            "cuts off YOLO26 end2end TopK ops Hailo cannot compile."
        ),
    )
    args = parser.parse_args()

    calib_dir = args.calib_dir
    if calib_dir is None:
        calib_dir = (
            REPO_ROOT / "training" / "datasets" / "open-soccer-detect" / "images" / "train"
        )

    try:
        compile_hef(
            onnx_path=args.onnx.resolve(),
            out_dir=args.out_dir.resolve(),
            calib_dir=calib_dir.resolve(),
            hw_arch=args.hw_arch,
            imgsz=args.imgsz,
            calib_images=args.calib_images,
            model_name=args.model_name,
            end_node_names=args.end_node_names,
        )
    except Exception as exc:
        # Surface DFC missing as a clean exit; re-raise anything else.
        if "hailo_sdk_client" in str(exc) or isinstance(exc, SystemExit):
            raise
        print(f"Hailo compile failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
