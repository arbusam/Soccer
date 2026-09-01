import csv
import json
import sys
import tempfile
import types
from pathlib import Path

import numpy as np

try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    cv2_stub = types.ModuleType("cv2")
    cv2_stub.FONT_HERSHEY_SIMPLEX = 0
    cv2_stub.LINE_AA = 0

    def rectangle(image, start, end, colour, thickness):
        del thickness
        x1, y1 = start
        x2, y2 = end
        image[y1:y2 + 1, x1] = colour
        image[y1:y2 + 1, x2] = colour
        image[y1, x1:x2 + 1] = colour
        image[y2, x1:x2 + 1] = colour

    def circle(image, centre, radius, colour, thickness):
        del radius, thickness
        image[centre[1], centre[0]] = colour

    def put_text(image, *_args, **_kwargs):
        image[0, 0] = 255

    cv2_stub.rectangle = rectangle
    cv2_stub.circle = circle
    cv2_stub.putText = put_text
    sys.modules["cv2"] = cv2_stub

from lib import session_replay


def timeline_latest_and_nearest():
    events = [{"time": 0.1, "value": 1}, {"time": 0.3, "value": 2}]
    timeline = session_replay.EventTimeline(events, "time")
    assert timeline.latest(0.05) is None
    assert timeline.latest(0.2)["value"] == 1
    assert timeline.nearest(0.29, 0.02)["value"] == 2
    assert timeline.nearest(0.2, 0.05) is None


def loads_session_and_converts_legacy_field_order():
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        (directory / "metadata.json").write_text(
            json.dumps({"schema_version": 1}),
            encoding="utf-8",
        )
        with (directory / "game.csv").open(
            "w",
            encoding="utf-8",
            newline="",
        ) as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=(
                    "elapsed_s",
                    "x",
                    "y",
                    "yaw",
                    "ball_x",
                    "ball_y",
                    "ball_captured",
                    "bot_mode",
                    "steering_state",
                    "direction",
                    "speed",
                    "rotation",
                    "kick",
                    "dribbler",
                ),
            )
            writer.writeheader()
            writer.writerow(
                {
                    "elapsed_s": 1.5,
                    "x": 10,
                    "y": 20,
                    "yaw": 30,
                    "ball_x": "",
                    "ball_y": "",
                    "ball_captured": False,
                    "bot_mode": "DEFENCE",
                    "steering_state": True,
                    "direction": 40,
                    "speed": 500,
                    "rotation": 50,
                    "kick": False,
                    "dribbler": True,
                }
            )
        with (directory / "detections.csv").open(
            "w",
            encoding="utf-8",
            newline="",
        ) as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=(
                    "elapsed_s",
                    "video_time_s",
                    "detected",
                    "bbox_x",
                    "bbox_y",
                    "bbox_w",
                    "bbox_h",
                    "centre_x",
                    "centre_y",
                    "confidence",
                ),
            )
            writer.writeheader()
            writer.writerow(
                {
                    "elapsed_s": 1.5,
                    "video_time_s": 1.0,
                    "detected": False,
                }
            )

        session = session_replay.load_recorded_session(directory)
        tokens = session_replay.game_event_tokens(session.game_events[0])
        assert tokens[:3] == ["10", "20", "30"]
        assert not session.detection_events[0]["detected"]


def annotation_does_not_modify_source_frame():
    source = np.zeros((80, 100, 3), dtype=np.uint8)
    annotated = session_replay.annotate_video_frame(
        source,
        {
            "detected": True,
            "bbox_x": 10,
            "bbox_y": 10,
            "bbox_w": 20,
            "bbox_h": 20,
            "centre_x": 20,
            "centre_y": 20,
            "confidence": 0.9,
        },
    )
    assert not np.any(source)
    assert np.any(annotated)
