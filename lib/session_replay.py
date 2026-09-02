"""Load and synchronize recorded controller, detection, and video timelines."""

from __future__ import annotations

import bisect
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import cv2


@dataclass
class RecordedSession:
    directory: Path
    metadata: dict
    game_events: list[dict]
    detection_events: list[dict]

    @property
    def video_path(self) -> Path:
        return self.directory / self.metadata.get("video_file", "video.mp4")


class EventTimeline:
    def __init__(self, events: list[dict], time_key: str):
        self.events = sorted(events, key=lambda event: float(event[time_key]))
        self.time_key = time_key
        self.times = [float(event[time_key]) for event in self.events]

    def latest(self, timestamp: float) -> dict | None:
        index = bisect.bisect_right(self.times, timestamp) - 1
        return self.events[index] if index >= 0 else None

    def nearest(self, timestamp: float, tolerance: float) -> dict | None:
        index = bisect.bisect_left(self.times, timestamp)
        candidates = []
        if index < len(self.events):
            candidates.append(self.events[index])
        if index > 0:
            candidates.append(self.events[index - 1])
        if not candidates:
            return None
        event = min(
            candidates,
            key=lambda candidate: abs(float(candidate[self.time_key]) - timestamp),
        )
        if abs(float(event[self.time_key]) - timestamp) > tolerance:
            return None
        return event


def _optional_float(value: str | None) -> float | None:
    if value is None or value.strip() == "" or value.strip().lower() == "none":
        return None
    return float(value)


def _bool(value: str | None) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def load_recorded_session(directory: str | Path) -> RecordedSession:
    directory = Path(directory)
    metadata_path = directory / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if int(metadata.get("schema_version", 0)) != 1:
        raise ValueError(
            f"Unsupported recording schema {metadata.get('schema_version')!r}"
        )

    game_path = directory / metadata.get("game_file", "game.csv")
    with game_path.open(encoding="utf-8", newline="") as handle:
        game_events = list(csv.DictReader(handle))
    for event in game_events:
        event["elapsed_s"] = float(event["elapsed_s"])

    detections_path = directory / metadata.get(
        "detections_file",
        "detections.csv",
    )
    with detections_path.open(encoding="utf-8", newline="") as handle:
        detection_events = list(csv.DictReader(handle))
    numeric_fields = (
        "elapsed_s",
        "video_time_s",
        "bbox_x",
        "bbox_y",
        "bbox_w",
        "bbox_h",
        "centre_x",
        "centre_y",
        "confidence",
    )
    for event in detection_events:
        for field in numeric_fields:
            event[field] = _optional_float(event.get(field))
        event["detected"] = _bool(event.get("detected"))

    return RecordedSession(directory, metadata, game_events, detection_events)


def game_event_tokens(event: dict) -> list[str]:
    """Convert a versioned game row to the legacy parser's field order."""
    fields = (
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
    )
    return [str(event.get(field, "None")) for field in fields]


class VideoReader:
    """Timestamp-seeking PyAV reader that keeps only one decoded frame."""

    def __init__(self, path: str | Path):
        try:
            import av
        except ImportError as exc:
            raise RuntimeError(
                "Recorded-session replay requires PyAV. Install the 'av' package."
            ) from exc

        self._av = av
        self.container = av.open(str(path))
        self.stream = self.container.streams.video[0]
        self.time_base = float(self.stream.time_base)
        self.start_time = int(self.stream.start_time or 0)
        average_rate = self.stream.average_rate
        self.fps = float(average_rate) if average_rate is not None else 30.0
        if self.stream.duration is not None:
            self.duration_s = float(self.stream.duration * self.stream.time_base)
        elif self.container.duration is not None:
            self.duration_s = float(self.container.duration / av.time_base)
        else:
            self.duration_s = 0.0
        packet_times = []
        for packet in self.container.demux(self.stream):
            if packet.pts is None or packet.size <= 0:
                continue
            packet_time = float((packet.pts - self.start_time) * self.stream.time_base)
            if packet_time >= 0 and (
                not packet_times or packet_time > packet_times[-1]
            ):
                packet_times.append(packet_time)
        self.frame_times = packet_times
        if self.frame_times:
            self.duration_s = max(self.duration_s, self.frame_times[-1])
        self.container.seek(0, stream=self.stream, backward=True)
        self._iterator = iter(self.container.decode(self.stream))
        self._last_time = None
        self._last_rgb = None

    def _seek(self, timestamp: float) -> None:
        target = max(0.0, timestamp - 0.25)
        self.container.seek(
            int(target / self.time_base),
            stream=self.stream,
            backward=True,
        )
        self._iterator = iter(self.container.decode(self.stream))
        self._last_time = None
        self._last_rgb = None

    def frame_at(self, timestamp: float):
        timestamp = max(0.0, float(timestamp))
        if (
            self._last_time is not None
            and (timestamp < self._last_time or timestamp - self._last_time > 1.0)
        ):
            self._seek(timestamp)

        for frame in self._iterator:
            frame_time = (
                float((frame.pts - self.start_time) * frame.time_base)
                if frame.pts is not None
                else (
                    0.0
                    if self._last_time is None
                    else self._last_time + (1.0 / self.fps)
                )
            )
            self._last_time = frame_time
            self._last_rgb = frame.to_ndarray(format="rgb24")
            if frame_time + (0.5 / self.fps) >= timestamp:
                break
        return self._last_rgb

    def close(self) -> None:
        self.container.close()


def annotate_video_frame(frame_rgb, detection: dict | None):
    """Draw recorded inference output on a replay-only frame copy."""
    annotated = frame_rgb.copy()
    if detection is None:
        label = "NOT INFERRED"
        colour = (255, 210, 0)
    elif not detection.get("detected"):
        label = "NO BALL"
        colour = (255, 80, 80)
    else:
        x = round(detection["bbox_x"])
        y = round(detection["bbox_y"])
        width = round(detection["bbox_w"])
        height = round(detection["bbox_h"])
        centre_x = round(detection["centre_x"])
        centre_y = round(detection["centre_y"])
        confidence = detection.get("confidence")
        cv2.rectangle(annotated, (x, y), (x + width, y + height), (255, 165, 0), 2)
        cv2.circle(annotated, (centre_x, centre_y), 5, (255, 255, 255), -1)
        label = f"BALL {confidence:.2f}" if confidence is not None else "BALL"
        colour = (255, 165, 0)
    cv2.putText(
        annotated,
        label,
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        colour,
        2,
        cv2.LINE_AA,
    )
    return annotated
