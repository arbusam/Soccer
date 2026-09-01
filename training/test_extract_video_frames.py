#!/usr/bin/env python3
"""Tests for training/extract_video_frames.py."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
from extract_video_frames import (
    extract_folder,
    frame_filename,
    next_clip_number,
    sample_frame_indices,
)


class NextClipNumberTests(unittest.TestCase):
    def test_empty_dir_starts_at_one(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(next_clip_number(Path(tmp)), 1)

    def test_increments_past_highest_numbered_clip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "output_000001.png").write_bytes(b"x")
            (root / "output3_000001.png").write_bytes(b"x")
            (root / "output22_000059.png").write_bytes(b"x")
            (root / "readme.txt").write_text("ignore", encoding="utf-8")
            self.assertEqual(next_clip_number(root), 23)

    def test_missing_dir_starts_at_one(self) -> None:
        self.assertEqual(next_clip_number(Path("/tmp/does-not-exist-photos-xyz")), 1)


class FrameFilenameTests(unittest.TestCase):
    def test_format(self) -> None:
        self.assertEqual(frame_filename(23, 1), "output23_000001.png")
        self.assertEqual(frame_filename(1, 59), "output1_000059.png")


class SampleIndexTests(unittest.TestCase):
    def test_one_fps_on_three_second_clip(self) -> None:
        # 30 fps * 3 s = 90 frames → samples at t=0,1,2 → frames 0, 30, 60
        self.assertEqual(sample_frame_indices(90, 30.0, 1.0), [0, 30, 60])

    def test_half_fps(self) -> None:
        # 4 s @ 30 fps, 0.5 fps → t=0,2 → frames 0, 60
        self.assertEqual(sample_frame_indices(120, 30.0, 0.5), [0, 60])


class ExtractFolderTests(unittest.TestCase):
    def _write_solid_video(self, path: Path, *, frames: int, fps: float, color: tuple[int, int, int]) -> None:
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (32, 32),
        )
        self.assertTrue(writer.isOpened(), f"could not open writer for {path}")
        try:
            for _ in range(frames):
                frame = np.full((32, 32, 3), color, dtype=np.uint8)
                writer.write(frame)
        finally:
            writer.release()

    def test_extracts_one_frame_per_second_with_new_clip_ids(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            videos = root / "videos"
            photos = root / "Photos"
            videos.mkdir()
            photos.mkdir()
            (photos / "output5_000001.png").write_bytes(b"x")

            video_a = videos / "clip_a.mp4"
            video_b = videos / "clip_b.mp4"
            # 2 seconds @ 10 fps → frames at t=0 and t=1
            self._write_solid_video(video_a, frames=20, fps=10.0, color=(0, 0, 255))
            self._write_solid_video(video_b, frames=20, fps=10.0, color=(0, 255, 0))

            results = extract_folder(videos, photos, fps=1.0)
            self.assertEqual(
                [(path.name, clip, written) for path, clip, written in results],
                [("clip_a.mp4", 6, 2), ("clip_b.mp4", 7, 2)],
            )
            self.assertTrue((photos / "output6_000001.png").is_file())
            self.assertTrue((photos / "output6_000002.png").is_file())
            self.assertTrue((photos / "output7_000001.png").is_file())
            self.assertTrue((photos / "output7_000002.png").is_file())
            # Source names must not appear in outputs.
            self.assertFalse(any("clip_a" in path.name for path in photos.iterdir()))


if __name__ == "__main__":
    unittest.main()
