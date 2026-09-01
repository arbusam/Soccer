#!/usr/bin/env python3
"""Remove Label Studio tasks whose images are byte-identical to another task.

Keeps one task per SHA-256 (prefer labeled, then more annotations, then lower id).
Deletes extra tasks and unreferenced image files.

Stop Label Studio before --apply so it does not overwrite the DB on exit.

Example:
  python training/dedupe_label_studio.py
  python training/dedupe_label_studio.py --apply
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sqlite3
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

TRAINING_ROOT = Path(__file__).resolve().parent
if str(TRAINING_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAINING_ROOT))

from export_label_studio import DEFAULT_DATA_DIR, _ls_image_path, _task_image_uri

TASK_CHILD_TABLES = (
    "prediction",
    "tasks_failedprediction",
    "tasks_annotationdraft",
    "task_comment_authors",
    "tasks_tasklock",
    "fsm_taskstate",
    "io_storages_localfilesimportstoragelink",
    "io_storages_s3importstoragelink",
    "io_storages_redisimportstoragelink",
    "io_storages_gcsimportstoragelink",
    "io_storages_azureblobimportstoragelink",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_tasks(
    db_path: Path,
    project_id: int,
    data_dir: Path,
) -> list[tuple[int, bool, int, int | None, Path]]:
    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute(
            """
            SELECT id, data, is_labeled, total_annotations, file_upload_id
            FROM task
            WHERE project_id = ?
            ORDER BY id ASC
            """,
            (project_id,),
        ).fetchall()
    finally:
        conn.close()

    tasks: list[tuple[int, bool, int, int | None, Path]] = []
    for task_id, data_json, is_labeled, total_annotations, file_upload_id in rows:
        uri = _task_image_uri(json.loads(data_json))
        if not uri:
            continue
        path = _ls_image_path(data_dir, uri)
        tasks.append(
            (
                task_id,
                bool(is_labeled),
                int(total_annotations or 0),
                file_upload_id,
                path,
            )
        )
    return tasks


def _duplicate_plan(
    tasks: list[tuple[int, bool, int, int | None, Path]],
) -> tuple[list[int], list[Path], list[int | None], int]:
    by_hash: dict[str, list[tuple[int, bool, int, int | None, Path]]] = defaultdict(list)
    missing = 0
    for task in tasks:
        path = task[4]
        if not path.is_file():
            missing += 1
            print(f"skip missing image: {path}")
            continue
        by_hash[_sha256(path)].append(task)

    drop_ids: list[int] = []
    drop_uploads: list[int | None] = []
    kept_paths: set[Path] = set()
    drop_path_candidates: list[Path] = []
    groups = 0
    for group in by_hash.values():
        if len(group) < 2:
            kept_paths.add(group[0][4])
            continue
        groups += 1
        group = sorted(group, key=lambda item: (not item[1], -item[2], item[0]))
        keep = group[0]
        kept_paths.add(keep[4])
        extras = group[1:]
        print(
            f"keep task {keep[0]} ({keep[4].name}, labeled={int(keep[1])}) "
            f"drop {[item[0] for item in extras]}"
        )
        for item in extras:
            drop_ids.append(item[0])
            drop_uploads.append(item[3])
            drop_path_candidates.append(item[4])

    drop_files = [
        path
        for path in dict.fromkeys(drop_path_candidates)
        if path not in kept_paths
    ]
    return drop_ids, drop_files, drop_uploads, groups


def _delete_tasks(conn: sqlite3.Connection, task_ids: list[int]) -> None:
    placeholders = ",".join("?" * len(task_ids))
    conn.execute(
        f"DELETE FROM fsm_annotationstate WHERE annotation_id IN "
        f"(SELECT id FROM task_completion WHERE task_id IN ({placeholders}))",
        task_ids,
    )
    conn.execute(
        f"DELETE FROM task_completion WHERE task_id IN ({placeholders})",
        task_ids,
    )
    for table in TASK_CHILD_TABLES:
        conn.execute(f"DELETE FROM {table} WHERE task_id IN ({placeholders})", task_ids)
    conn.execute(f"DELETE FROM task WHERE id IN ({placeholders})", task_ids)


def _delete_unused_uploads(
    conn: sqlite3.Connection,
    upload_ids: list[int | None],
) -> int:
    wanted = sorted({upload_id for upload_id in upload_ids if upload_id is not None})
    if not wanted:
        return 0
    placeholders = ",".join("?" * len(wanted))
    still_used = {
        row[0]
        for row in conn.execute(
            f"SELECT DISTINCT file_upload_id FROM task WHERE file_upload_id IN ({placeholders})",
            wanted,
        )
    }
    unused = [upload_id for upload_id in wanted if upload_id not in still_used]
    if not unused:
        return 0
    placeholders = ",".join("?" * len(unused))
    conn.execute(f"DELETE FROM data_import_fileupload WHERE id IN ({placeholders})", unused)
    return len(unused)


def dedupe(data_dir: Path, project_id: int, apply: bool) -> None:
    db_path = data_dir / "label_studio.sqlite3"
    if not db_path.is_file():
        raise FileNotFoundError(f"Label Studio DB not found: {db_path}")

    tasks = _load_tasks(db_path, project_id, data_dir)
    drop_ids, drop_files, drop_uploads, groups = _duplicate_plan(tasks)
    print(
        f"Found {groups} duplicate group(s): "
        f"{len(drop_ids)} extra task(s), {len(drop_files)} extra file(s)."
    )
    if not drop_ids:
        print("Nothing to delete.")
        return
    if not apply:
        print("Dry run only. Re-run with --apply to delete extra tasks and files.")
        print("Stop Label Studio before applying so it does not overwrite the DB on exit.")
        return

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = db_path.with_name(f"{db_path.name}.bak-dedupe-{stamp}")
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
        description="Delete Label Studio tasks that share an exact-duplicate image."
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
        help="Delete extras (creates a .bak-dedupe-* backup first). Dry-run otherwise.",
    )
    args = parser.parse_args()
    dedupe(data_dir=args.data_dir.resolve(), project_id=args.project_id, apply=args.apply)


if __name__ == "__main__":
    main()
