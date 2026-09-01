#!/usr/bin/env python3
"""Tests for training/delete_skipped_label_studio.py."""

from __future__ import annotations

import unittest
from pathlib import Path

from delete_skipped_label_studio import plan_skipped_deletes


class PlanSkippedDeletesTests(unittest.TestCase):
    def test_deletes_skip_only_tasks_and_their_files(self) -> None:
        photos = Path("/data/Photos")
        skipped = photos / "output10_000001.png"
        labeled = photos / "output10_000002.png"
        drop_ids, drop_files, drop_uploads = plan_skipped_deletes(
            [
                (1, True, None, skipped),
                (2, False, None, labeled),
            ]
        )
        self.assertEqual(drop_ids, [1])
        self.assertEqual(drop_files, [skipped])
        self.assertEqual(drop_uploads, [None])

    def test_keeps_file_if_another_task_still_uses_it(self) -> None:
        shared = Path("/data/Photos/output10_000001.png")
        drop_ids, drop_files, drop_uploads = plan_skipped_deletes(
            [
                (1, True, 10, shared),
                (2, False, 11, shared),
            ]
        )
        self.assertEqual(drop_ids, [1])
        self.assertEqual(drop_files, [])
        self.assertEqual(drop_uploads, [10])

    def test_does_not_delete_labeled_or_unlabeled_tasks(self) -> None:
        photos = Path("/data/Photos")
        drop_ids, drop_files, _uploads = plan_skipped_deletes(
            [
                (1, False, None, photos / "a.png"),
                (2, False, None, photos / "b.png"),
            ]
        )
        self.assertEqual(drop_ids, [])
        self.assertEqual(drop_files, [])

    def test_dedupes_shared_skip_only_files(self) -> None:
        shared = Path("/data/Photos/dup.png")
        drop_ids, drop_files, drop_uploads = plan_skipped_deletes(
            [
                (1, True, 1, shared),
                (2, True, 2, shared),
            ]
        )
        self.assertEqual(drop_ids, [1, 2])
        self.assertEqual(drop_files, [shared])
        self.assertEqual(drop_uploads, [1, 2])


if __name__ == "__main__":
    unittest.main()
