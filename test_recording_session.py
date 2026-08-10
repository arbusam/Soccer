import csv
import json
import tempfile
import unittest
from pathlib import Path

from recording_session import RecordingSession


class RecordingSessionTests(unittest.TestCase):
    def test_writes_versioned_game_and_detection_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            session_path = Path(temporary) / "match"
            session = RecordingSession(
                session_path,
                resolution=(640, 640),
                requested_fps=90,
            )
            session.record_game(
                (1, 2, 3, 4, 5, True, "STRIKER", False, 10, 500, 20, False, True),
                elapsed_s=0.25,
            )
            session.record_detection(
                {
                    "elapsed_s": 0.2,
                    "video_time_s": 0.1,
                    "capture_sequence": 10,
                    "sensor_timestamp_ns": 123,
                    "inference_sequence": 9,
                    "detection": {
                        "bbox": (1, 2, 30, 40),
                        "centre": (16, 22),
                        "confidence": 0.8,
                    },
                }
            )
            session.close()

            metadata = json.loads(session.metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["schema_version"], 1)
            self.assertEqual(metadata["game_rows"], 1)
            self.assertEqual(metadata["detection_rows"], 1)

            with session.game_path.open(encoding="utf-8", newline="") as handle:
                game_rows = list(csv.DictReader(handle))
            self.assertEqual(game_rows[0]["bot_mode"], "STRIKER")
            self.assertEqual(float(game_rows[0]["elapsed_s"]), 0.25)

            with session.detections_path.open(
                encoding="utf-8",
                newline="",
            ) as handle:
                detection_rows = list(csv.DictReader(handle))
            self.assertEqual(detection_rows[0]["detected"], "True")
            self.assertEqual(detection_rows[0]["bbox_w"], "30")

    def test_refuses_to_overwrite_nonempty_session(self):
        with tempfile.TemporaryDirectory() as temporary:
            session_path = Path(temporary) / "match"
            session_path.mkdir()
            (session_path / "existing.txt").write_text("keep", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                RecordingSession(
                    session_path,
                    resolution=(640, 640),
                    requested_fps=90,
                )


if __name__ == "__main__":
    unittest.main()
