"""Opt-in, bounded timing capture. No hardware dependencies; clocks use MONOTONIC."""

from __future__ import annotations

import json
import math
import platform
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

active = None


def now_ns():
    return time.monotonic_ns()


def _host_state():
    state = {}
    try:
        state["temperature_millic"] = int(
            Path("/sys/class/thermal/thermal_zone0/temp").read_text()
        )
    except (OSError, ValueError):
        pass
    try:
        state["throttling"] = subprocess.check_output(
            ["vcgencmd", "get_throttled"], text=True, timeout=2
        ).strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return state


class Recorder:
    def __init__(self, *, warmup=10.0, duration=60.0, capacity=100_000, metadata=None):
        if (
            not math.isfinite(warmup)
            or not math.isfinite(duration)
            or warmup < 0
            or duration <= 0
            or capacity <= 0
        ):
            raise ValueError(
                "Timing needs finite warmup >= 0, duration > 0 and capacity > 0"
            )
        self.capacity = capacity
        self.local = threading.local()
        self.buffers = []
        self.stop = threading.Event()
        self.metadata = dict(metadata or {})
        self.metadata.update(
            python=sys.version,
            platform=platform.platform(),
            gil_enabled=getattr(sys, "_is_gil_enabled", lambda: True)(),
            start_host=_host_state(),
        )
        try:
            self.metadata["revision"] = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], text=True, timeout=2
            ).strip()
            self.metadata["git_status"] = subprocess.check_output(
                ["git", "status", "--short"], text=True, timeout=2
            )
        except (OSError, subprocess.SubprocessError):
            pass
        self.start_ns = now_ns() + round(warmup * 1e9)
        self.end_ns = self.start_ns + round(duration * 1e9)
        self.thread = threading.Thread(
            target=self._heartbeat, name="timing-heartbeat", daemon=True
        )
        self.thread.start()

    def record(self, name, start, end=None, value=None):
        end = now_ns() if end is None else end
        if self.stop.is_set() or end >= self.end_ns:
            return
        buf = getattr(self.local, "buffer", None)
        if buf is None:
            buf = {
                "thread": threading.current_thread().name,
                "rows": [],
                "observations": [],
                "dropped": 0,
                "lock": threading.Lock(),
            }
            self.local.buffer = buf
            # Registration is GIL-protected; each buffer has one producer.
            self.buffers.append(buf)
        if end < self.start_ns:
            return
        self._append(
            buf, "rows", (name, start, end, end - start if value is None else value)
        )

    def _append(self, buf, key, row):
        if not buf["lock"].acquire(blocking=False):
            buf["dropped"] += 1
            return
        try:
            if self.stop.is_set():
                return
            if len(buf["rows"]) + len(buf["observations"]) >= self.capacity:
                buf["dropped"] += 1
                return
            buf[key].append(row)
        finally:
            buf["lock"].release()

    def observe(self, name, values):
        stamp = now_ns()
        if self.stop.is_set() or not self.start_ns <= stamp < self.end_ns:
            return
        # State observations originate from the already-registered main thread.
        buf = getattr(self.local, "buffer", None)
        if buf is None:
            self.record("capture.registration", stamp, stamp)
            buf = getattr(self.local, "buffer", None)
            if buf is None:
                return
        self._append(buf, "observations", (name, stamp, values))

    def _heartbeat(self):
        previous = now_ns()
        while not self.stop.wait(0.005):
            current = now_ns()
            self.record(
                "heartbeat.delay",
                previous,
                current,
                max(0, current - previous - 5_000_000),
            )
            previous = current
            if current >= self.end_ns:
                return

    def attach_native(self, lidar):
        brackets = []
        for _ in range(7):
            before = now_ns()
            native = lidar.timing_clock_ns()
            after = now_ns()
            brackets.append((after - before, before, native, after))
        _, before, native, after = min(brackets)
        if not before <= native <= after:
            raise RuntimeError("Python and native CLOCK_MONOTONIC epochs do not match")
        self.metadata.update(
            native_clock_bracket_ns=[before, native, after],
            extension=lidar.__file__,
            runtime_gil_released=lidar.runtime_gil_released,
        )
        lidar.timing_start(self.start_ns, self.end_ns, self.capacity)

    def finish(self, path, lidar=None):
        finished_ns = now_ns()
        self.stop.set()
        self.thread.join(timeout=1)
        self.metadata["end_host"] = _host_state()
        native = lidar.timing_finish() if lidar is not None else []
        snapshots = []
        for buf in self.buffers.copy():
            with buf["lock"]:
                snapshots.append(
                    {
                        key: (value.copy() if isinstance(value, list) else value)
                        for key, value in buf.items()
                        if key != "lock"
                    }
                )
        payload = {
            "schema_version": 1,
            "clock": "CLOCK_MONOTONIC",
            "start_ns": self.start_ns,
            "end_ns": self.end_ns,
            "finished_ns": finished_ns,
            "metadata": self.metadata,
            "buffers": [*snapshots, *native],
        }
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, default=str))
        summary = summarize(payload)
        path.with_suffix(path.suffix + ".summary.json").write_text(
            json.dumps(summary, indent=2)
        )
        path.with_suffix(path.suffix + ".trace.json").write_text(
            json.dumps(chrome_trace(payload))
        )
        return summary


def record(name, start, end=None, value=None):
    if active is not None:
        active.record(name, start, end, value)


def call(name, function, *args):
    if active is None:
        return function(*args)
    start = now_ns()
    try:
        return function(*args)
    finally:
        record(name, start)


@contextmanager
def _measured_lock(lock, label):
    start = now_ns()
    lock.acquire()
    acquired = now_ns()
    try:
        yield
    finally:
        released = now_ns()
        lock.release()
        # Never append records while holding the hardware lock.
        record("i2c." + label + ".wait", start, acquired)
        record("i2c." + label + ".hold", acquired, released)


def measured_lock(lock, label):
    return lock if active is None else _measured_lock(lock, label)


def summarize(payload):
    groups = {}
    for buf in payload["buffers"]:
        for name, start, end, value in buf["rows"]:
            if start < payload["start_ns"] or end >= payload["end_ns"]:
                continue
            groups.setdefault(name, []).append(value)
    result = {"dropped": sum(b["dropped"] for b in payload["buffers"]), "metrics": {}}
    seconds = max(
        0,
        (
            min(payload.get("finished_ns", payload["end_ns"]), payload["end_ns"])
            - payload["start_ns"]
        )
        / 1e9,
    )
    result["captured_seconds"] = seconds
    result["event_rates_hz"] = {
        name: len(values) / seconds if seconds else 0
        for name, values in groups.items()
        if name
        in (
            "scan.accepted",
            "scan.skipped",
            "camera.capture",
            "camera.publish_age",
            "imu.interval",
            "drive.interval",
            "logic.interval",
        )
    }
    observations = [
        o
        for b in payload["buffers"]
        for o in b.get("observations", [])
        if o[0] == "state"
    ]
    result["state_samples"] = len(observations)
    if observations:
        result["confident_pose_percent"] = (
            100
            * sum(o[2]["pose"][0] is not None for o in observations)
            / len(observations)
        )
        result["scan_gate_paused_percent"] = (
            100
            * sum(not o[2]["scan_updates_enabled"] for o in observations)
            / len(observations)
        )
    for name, values in sorted(groups.items()):
        values.sort()

        def percentile(p, values=values):
            return values[max(0, math.ceil(p * len(values)) - 1)] / 1e6

        result["metrics"][name] = {
            "count": len(values),
            "median_ms": percentile(0.5),
            "p95_ms": percentile(0.95),
            "p99_ms": percentile(0.99),
            "max_ms": values[-1] / 1e6,
        }
        if name in ("drive.interval", "drive.writes.interval"):
            for threshold in (25, 40):
                count = sum(v > threshold * 1e6 for v in values)
                result["metrics"][name][f"over_{threshold}ms"] = {
                    "count": count,
                    "percent": 100 * count / len(values),
                }
    return result


def chrome_trace(payload):
    events = []
    for index, buf in enumerate(payload["buffers"]):
        events.append(
            {
                "ph": "M",
                "name": "thread_name",
                "pid": 1,
                "tid": index,
                "args": {"name": buf["thread"]},
            }
        )
        for name, stamp, values in buf.get("observations", []):
            events.append(
                {
                    "ph": "i",
                    "s": "t",
                    "name": name,
                    "pid": 1,
                    "tid": index,
                    "ts": (stamp - payload["start_ns"]) / 1000,
                    "args": values,
                }
            )
        for name, start, end, value in buf["rows"]:
            events.append(
                {
                    "ph": "X",
                    "name": name,
                    "pid": 1,
                    "tid": index,
                    "ts": (start - payload["start_ns"]) / 1000,
                    "dur": (end - start) / 1000,
                    "args": {"value_ns": value},
                }
            )
    return {"traceEvents": events}
