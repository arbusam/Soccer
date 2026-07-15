#!/usr/bin/env python3
"""Export Label Studio Open Soccer annotations to a YOLO-OBB dataset."""

from __future__ import annotations

import argparse
import json
import math
import random
import shutil
import sqlite3
from pathlib import Path

CLASS_NAMES = ("Ball", "Blue Goal", "Yellow Goal", "Bot")
CLASS_TO_ID = {name: i for i, name in enumerate(CLASS_NAMES)}

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA_DIR = REPO_ROOT / "label-studio-data"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "datasets" / "open-soccer-obb"


def _ls_image_path(data_dir: Path, image_uri: str) -> Path:
    """Map Label Studio `/data/...` URIs to files under the data dir."""
    uri = image_uri.strip()
    if uri.startswith("/data/"):
        return data_dir / "media" / uri[len("/data/") :]
    path = Path(uri)
    if path.is_file():
        return path
    return data_dir / "media" / uri.lstrip("/")


def _rotated_corners(
    x_pct: float,
    y_pct: float,
    w_pct: float,
    h_pct: float,
    rotation_deg: float,
) -> list[tuple[float, float]]:
    """Label Studio percent box + clockwise rotation → normalized OBB corners.

    Returns four (x, y) corners in [0, 1], order TL→TR→BR→BL before rotation.
    """
    x = x_pct / 100.0
    y = y_pct / 100.0
    w = w_pct / 100.0
    h = h_pct / 100.0
    cx = x + w / 2.0
    cy = y + h / 2.0

    local = (
        (-w / 2.0, -h / 2.0),
        (w / 2.0, -h / 2.0),
        (w / 2.0, h / 2.0),
        (-w / 2.0, h / 2.0),
    )
    # Label Studio rotation is clockwise-positive.
    rad = math.radians(rotation_deg)
    cos_r = math.cos(rad)
    sin_r = math.sin(rad)

    corners: list[tuple[float, float]] = []
    for dx, dy in local:
        rx = cx + dx * cos_r + dy * sin_r
        ry = cy - dx * sin_r + dy * cos_r
        corners.append((min(1.0, max(0.0, rx)), min(1.0, max(0.0, ry))))
    return corners


def _result_to_yolo_obb_lines(result_json: str) -> list[str]:
    lines: list[str] = []
    for item in json.loads(result_json):
        if item.get("type") != "rectanglelabels":
            continue
        value = item.get("value") or {}
        labels = value.get("rectanglelabels") or []
        if not labels:
            continue
        class_id = CLASS_TO_ID.get(labels[0])
        if class_id is None:
            continue
        corners = _rotated_corners(
            float(value["x"]),
            float(value["y"]),
            float(value["width"]),
            float(value["height"]),
            float(value.get("rotation") or 0.0),
        )
        coords = " ".join(f"{x:.6f} {y:.6f}" for x, y in corners)
        lines.append(f"{class_id} {coords}")
    return lines


def _load_labeled_tasks(
    db_path: Path,
    project_id: int,
) -> list[tuple[dict, str]]:
    """Return [(task.data, annotation.result), ...] — latest non-cancelled per task."""
    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute(
            """
            SELECT t.id, t.data, tc.result, tc.updated_at
            FROM task AS t
            JOIN task_completion AS tc ON tc.task_id = t.id
            WHERE t.project_id = ?
              AND tc.was_cancelled = 0
            ORDER BY t.id ASC, tc.updated_at DESC
            """,
            (project_id,),
        ).fetchall()
    finally:
        conn.close()

    latest: dict[int, tuple[dict, str]] = {}
    for task_id, data_json, result_json, _updated_at in rows:
        if task_id in latest:
            continue
        data = json.loads(data_json)
        if "image" not in data:
            continue
        latest[task_id] = (data, result_json)
    return list(latest.values())


def _write_data_yaml(out_dir: Path) -> Path:
    yaml_path = out_dir / "data.yaml"
    names_block = "\n".join(f"  {i}: {name}" for i, name in enumerate(CLASS_NAMES))
    yaml_path.write_text(
        "\n".join(
            [
                f"path: {out_dir.resolve()}",
                "train: images/train",
                "val: images/val",
                "names:",
                names_block,
                "",
            ]
        ),
        encoding="utf-8",
    )
    return yaml_path


def export_dataset(
    data_dir: Path,
    out_dir: Path,
    project_id: int,
    val_fraction: float,
    seed: int,
) -> Path:
    db_path = data_dir / "label_studio.sqlite3"
    if not db_path.is_file():
        raise FileNotFoundError(f"Label Studio DB not found: {db_path}")

    samples = _load_labeled_tasks(db_path, project_id)
    if not samples:
        raise RuntimeError(f"No labeled tasks found for project_id={project_id}")

    if out_dir.exists():
        shutil.rmtree(out_dir)
    for split in ("train", "val"):
        (out_dir / "images" / split).mkdir(parents=True)
        (out_dir / "labels" / split).mkdir(parents=True)

    rng = random.Random(seed)
    indices = list(range(len(samples)))
    rng.shuffle(indices)
    val_count = max(1, int(round(len(samples) * val_fraction))) if len(samples) > 1 else 0
    val_ids = set(indices[:val_count])

    exported = 0
    skipped = 0
    for i, (data, result_json) in enumerate(samples):
        src = _ls_image_path(data_dir, data["image"])
        if not src.is_file():
            skipped += 1
            print(f"skip missing image: {src}")
            continue

        lines = _result_to_yolo_obb_lines(result_json)
        if not lines:
            skipped += 1
            continue

        split = "val" if i in val_ids else "train"
        stem = f"{src.stem}_{i:04d}"
        dst_image = out_dir / "images" / split / f"{stem}{src.suffix.lower()}"
        dst_label = out_dir / "labels" / split / f"{stem}.txt"
        shutil.copy2(src, dst_image)
        dst_label.write_text("\n".join(lines) + "\n", encoding="utf-8")
        exported += 1

    if exported == 0:
        raise RuntimeError("No samples exported (missing images or empty labels)")

    yaml_path = _write_data_yaml(out_dir)
    print(
        f"Exported {exported} images "
        f"(skipped {skipped}) → {out_dir} "
        f"[train/val split, val≈{val_fraction:.0%}]"
    )
    print(f"Dataset config: {yaml_path}")
    return yaml_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export Label Studio annotations to YOLO-OBB format."
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help="Label Studio data directory (default: repo label-studio-data/)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Output YOLO-OBB dataset directory",
    )
    parser.add_argument("--project-id", type=int, default=2, help="Label Studio project id")
    parser.add_argument(
        "--val-fraction",
        type=float,
        default=0.2,
        help="Fraction of images reserved for validation",
    )
    parser.add_argument("--seed", type=int, default=42, help="Train/val shuffle seed")
    args = parser.parse_args()

    if not 0.0 <= args.val_fraction < 1.0:
        raise SystemExit("--val-fraction must be in [0, 1)")

    export_dataset(
        data_dir=args.data_dir.resolve(),
        out_dir=args.out_dir.resolve(),
        project_id=args.project_id,
        val_fraction=args.val_fraction,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
