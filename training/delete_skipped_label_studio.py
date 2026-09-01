#!/usr/bin/env python3
"""Delete Label Studio tasks that were skipped, and their unreferenced images.

A task is skipped when it has cancelled annotations and no real ones (the Skip
button). Label Studio does not remove Photos/ or upload files on its own.

Stop Label Studio before --apply so it does not overwrite the DB on exit.

Example:
    python training/delete_skipped_label_studio.py
    python training/delete_skipped_label_studio.py --apply
"""

from __future__ import annotations

import argparse
import json
import shutil
import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path

TRAINING_ROOT = Path(__file__).resolve().parent
if str(TRAINING_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAINING_ROOT))

from dedupe_label_studio import _delete_tasks, _delete_unused_uploads
from export_label_studio import DEFAULT_DATA_DIR, _ls_image_path, _task_image_uri


def _load_tasks(
    db_path: Path,
    project_id: int,
    data_dir: Path,
) -> list[tuple[int, bool, int | None, Path]]:
    """Return (task_id, skip_only, file_upload_id, image_path) for the project."""
    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute(
            """
            SELECT id, data, total_annotations, cancelled_annotations, file_upload_id
            FROM task
            WHERE project_id = ?
            ORDER BY id ASC
            """,
            (project_id,),
        ).fetchall()
    finally:
        conn.close()

    tasks: list[tuple[int, bool, int | None, Path]] = []
    for task_id, data_json, total_annotations, cancelled_annotations, file_upload_id in rows:
        uri = _task_image_uri(json.loads(data_json))
        if not uri:
            continue
        skip_only = int(cancelled_annotations or 0) > 0 and int(total_annotations or 0) == 0
        tasks.append(
            (
                task_id,
                skip_only,
                file_upload_id,
                _ls_image_path(data_dir, uri),
            )
        )
    return tasks


def plan_skipped_deletes(
    tasks: list[tuple[int, bool, int | None, Path]],
) -> tuple[list[int], list[Path], list[int | None]]:
    """Tasks to drop, image files unused after that, and their file_upload ids."""
    drop_ids: list[int] = []
    drop_uploads: list[int | None] = []
    drop_path_candidates: list[Path] = []
    kept_paths: set[Path] = set()
    for task_id, skip_only, file_upload_id, path in tasks:
        if skip_only:
            drop_ids.append(task_id)
            drop_uploads.append(file_upload_id)
            drop_path_candidates.append(path)
        else:
            kept_paths.add(path)

    drop_files = [
        path
        for path in dict.fromkeys(drop_path_candidates)
        if path not in kept_paths
    ]
    return drop_ids, drop_files, drop_uploads


def delete_skipped(data_dir: Path, project_id: int, apply: bool) -> None:
    db_path = data_dir / "label_studio.sqlite3"
    if not db_path.is_file():
        raise FileNotFoundError(f"Label Studio DB not found: {db_path}")

    tasks = _load_tasks(db_path, project_id, data_dir)
    drop_ids, drop_files, drop_uploads = plan_skipped_deletes(tasks)
    by_id = {task[0]: task for task in tasks}

    print(
        f"Found {len(drop_ids)} skipped task(s), {len(drop_files)} image file(s) "
        f"with no remaining task."
    )
    for task_id in drop_ids:
        path = by_id[task_id][3]
        kept = path not in drop_files
        note = " (keep file, still used)" if kept else ""
        print(f"  task {task_id} {path.name}{note}")
    kept_skip_files = {
        by_id[task_id][3]
        for task_id in drop_ids
        if by_id[task_id][3] not in drop_files
    }
    for path in kept_skip_files:
        print(f"  keep {path}")

    if not drop_ids:
        print("Nothing to delete.")
        return
    if not apply:
        print("Dry run only. Re-run with --apply to delete skipped tasks and files.")
        print("Stop Label Studio before applying so it does not overwrite the DB on exit.")
        return

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = db_path.with_name(f"{db_path.name}.bak-skipped-{stamp}")
    shutil.copy2(db_path, backup)
    print(f"Backup: {backup}")

    conn = sqlite3.connect(db_path)
    try:
        _delete_tasks(conn, drop_ids)
        n_uploads = _delete_unused_uploads(conn, drop_uploads)
        conn.commit()
    finally:
        conn.close()

    removed_files = 0
    for path in drop_files:
        if path.is_file():
            path.unlink()
            removed_files += 1
    print(
        f"Deleted {len(drop_ids)} task(s), {n_uploads} file-upload row(s), "
        f"{removed_files} image file(s)."
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Delete skipped Label Studio tasks and unreferenced images."
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help="Label Studio data directory (default: repo label-studio-data/)",
    )
    parser.add_argument("--project-id", type=int, default=2, help="Label Studio project id")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Delete skipped tasks and files (creates a .bak-skipped-* backup first). Dry-run otherwise.",
    )
    args = parser.parse_args()
    delete_skipped(
        data_dir=args.data_dir.resolve(),
        project_id=args.project_id,
        apply=args.apply,
    )


if __name__ == "__main__":
    main()
