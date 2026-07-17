#!/usr/bin/env python3
"""Export Label Studio annotations to a YOLO-detect dataset, or rewrite them in-place as axis-aligned boxes."""

from __future__ import annotations

import argparse
import json
import math
import random
import shutil
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

CLASS_NAMES = ("Ball", "Bot")
CLASS_TO_ID = {name: i for i, name in enumerate(CLASS_NAMES)}

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA_DIR = REPO_ROOT / "label-studio-data"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "datasets" / "open-soccer-detect"


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
    """Label Studio percent box + clockwise rotation → normalized corners in [0, 1]."""
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


def _aabb_percent(
    x_pct: float,
    y_pct: float,
    w_pct: float,
    h_pct: float,
    rotation_deg: float,
) -> tuple[float, float, float, float] | None:
    """Axis-aligned percent box covering a rotated Label Studio rectangle."""
    corners = _rotated_corners(x_pct, y_pct, w_pct, h_pct, rotation_deg)
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width = (max_x - min_x) * 100.0
    height = (max_y - min_y) * 100.0
    if width <= 0.0 or height <= 0.0:
        return None
    return min_x * 100.0, min_y * 100.0, width, height


def _result_to_yolo_detect_lines(result_json: str) -> list[str]:
    """Axis-aligned YOLO detect labels from Label Studio rectangles."""
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
        aabb = _aabb_percent(
            float(value["x"]),
            float(value["y"]),
            float(value["width"]),
            float(value["height"]),
            float(value.get("rotation") or 0.0),
        )
        if aabb is None:
            continue
        x_pct, y_pct, w_pct, h_pct = aabb
        cx = (x_pct + w_pct / 2.0) / 100.0
        cy = (y_pct + h_pct / 2.0) / 100.0
        lines.append(f"{class_id} {cx:.6f} {cy:.6f} {w_pct / 100.0:.6f} {h_pct / 100.0:.6f}")
    return lines


def _axis_align_result(result_json: str) -> tuple[str, int]:
    """Rewrite rotated rectanglelabels to AABB with rotation=0. Returns (json, changed_count)."""
    items = json.loads(result_json)
    changed = 0
    for item in items:
        if item.get("type") != "rectanglelabels":
            continue
        value = item.get("value") or {}
        rotation = float(value.get("rotation") or 0.0)
        if abs(rotation) < 1e-9:
            continue
        aabb = _aabb_percent(
            float(value["x"]),
            float(value["y"]),
            float(value["width"]),
            float(value["height"]),
            rotation,
        )
        if aabb is None:
            continue
        x_pct, y_pct, w_pct, h_pct = aabb
        value["x"] = x_pct
        value["y"] = y_pct
        value["width"] = w_pct
        value["height"] = h_pct
        value["rotation"] = 0.0
        item["value"] = value
        changed += 1
    return json.dumps(items, ensure_ascii=False), changed


def update_label_studio_to_detect(
    data_dir: Path,
    project_id: int,
    *,
    apply: bool,
) -> None:
    """Convert rotated boxes in Label Studio's sqlite DB to axis-aligned rectangles.

    Stop Label Studio before --apply so it does not overwrite the DB on exit.
    """
    db_path = data_dir / "label_studio.sqlite3"
    if not db_path.is_file():
        raise FileNotFoundError(f"Label Studio DB not found: {db_path}")

    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute(
            """
            SELECT tc.id, tc.result
            FROM task_completion AS tc
            JOIN task AS t ON t.id = tc.task_id
            WHERE t.project_id = ?
              AND tc.was_cancelled = 0
            ORDER BY tc.id ASC
            """,
            (project_id,),
        ).fetchall()

        updates: list[tuple[str, int]] = []
        boxes_changed = 0
        for completion_id, result_json in rows:
            new_result, changed = _axis_align_result(result_json)
            if changed:
                updates.append((new_result, completion_id))
                boxes_changed += changed

        print(
            f"Found {boxes_changed} rotated box(es) across {len(updates)} annotation(s) "
            f"(project_id={project_id})."
        )
        if not updates:
            print("Nothing to update.")
            return

        if not apply:
            print("Dry run only. Re-run with --apply to write changes into Label Studio.")
            print("Stop Label Studio before applying so it does not overwrite the DB on exit.")
            return

        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        backup = db_path.with_name(f"{db_path.name}.bak-detect-{stamp}")
        shutil.copy2(db_path, backup)
        print(f"Backup: {backup}")

        now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")
        conn.executemany(
            "UPDATE task_completion SET result = ?, updated_at = ? WHERE id = ?",
            [(result, now, completion_id) for result, completion_id in updates],
        )
        conn.commit()
        print(f"Updated {len(updates)} annotation(s). Reload Label Studio to review.")
    finally:
        conn.close()


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

        lines = _result_to_yolo_detect_lines(result_json)
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
        f"Exported {exported} images (detect) "
        f"(skipped {skipped}) → {out_dir} "
        f"[train/val split, val≈{val_fraction:.0%}]"
    )
    print(f"Dataset config: {yaml_path}")
    return yaml_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Export Label Studio annotations to YOLO-detect, or rewrite rotated "
            "boxes in Label Studio as axis-aligned rectangles."
        )
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
        help="Output dataset directory (default: training/datasets/open-soccer-detect)",
    )
    parser.add_argument("--project-id", type=int, default=2, help="Label Studio project id")
    parser.add_argument(
        "--val-fraction",
        type=float,
        default=0.2,
        help="Fraction of images reserved for validation",
    )
    parser.add_argument("--seed", type=int, default=42, help="Train/val shuffle seed")
    parser.add_argument(
        "--update-label-studio",
        action="store_true",
        help=(
            "Rewrite rotated rectangles in Label Studio's sqlite DB to axis-aligned "
            "boxes (rotation=0) so you can review them in the UI. Dry-run unless --apply."
        ),
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="With --update-label-studio, write changes (creates a .bak-detect-* backup first)",
    )
    args = parser.parse_args()

    if args.update_label_studio:
        update_label_studio_to_detect(
            data_dir=args.data_dir.resolve(),
            project_id=args.project_id,
            apply=args.apply,
        )
        return

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
