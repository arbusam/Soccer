import json
import subprocess
import sys
import threading

import pytest

from lib import timing


def test_bounded_window_summary_and_trace(tmp_path, monkeypatch):
    monkeypatch.setattr(timing, "_host_state", dict)
    recorder = timing.Recorder(warmup=0, duration=10, capacity=2)
    try:
        start = recorder.start_ns
        recorder.record("drive.interval", start, start + 20_000_000)
        recorder.record("drive.interval", start + 20_000_000, start + 65_000_000)
        recorder.record("drive.interval", start + 65_000_000, start + 85_000_000)
        recorder.record("outside", recorder.end_ns, recorder.end_ns + 1)
        path = tmp_path / "run.json"
        summary = recorder.finish(path)
        assert summary["dropped"] == 1
        metric = summary["metrics"]["drive.interval"]
        assert metric["count"] == 2
        assert metric["p99_ms"] == 45
        assert metric["over_40ms"] == {"count": 1, "percent": 50.0}
        assert "outside" not in summary["metrics"]
        trace = json.loads(path.with_suffix(".json.trace.json").read_text())
        assert any(e["name"] == "drive.interval" for e in trace["traceEvents"])
    finally:
        recorder.stop.set()
        recorder.thread.join(timeout=1)


def test_disabled_lock_is_original_and_exception_releases_lock(monkeypatch):
    lock = threading.Lock()
    assert timing.measured_lock(lock, "test") is lock
    rows = []
    monkeypatch.setattr(timing, "active", object())

    def record(*args):
        assert not lock.locked()
        rows.append(args)

    monkeypatch.setattr(timing, "record", record)
    with pytest.raises(ValueError), timing.measured_lock(lock, "test"):
        raise ValueError("failed transaction")
    assert [r[0] for r in rows] == ["i2c.test.wait", "i2c.test.hold"]


def test_native_capture_clock_capacity_and_scan_status(tmp_path):
    source = """
from lib import lidar, timing
import sys
r = timing.Recorder(warmup=0, duration=5, capacity=12)
r.attach_native(lidar)
lidar.test_mcl_start(2430, 1820)
lidar.test_mcl_update_scan([])
for _ in range(20):
    lidar.set_imu_yaw(0)
lidar.test_mcl_stop()
r.finish(sys.argv[1], lidar)
"""
    path = tmp_path / "native.json"
    subprocess.run([sys.executable, "-c", source, str(path)], check=True, timeout=10)
    payload = json.loads(path.read_text())
    native = [b for b in payload["buffers"] if b["thread"].startswith("native-")]
    assert len(native) == 1
    assert len(native[0]["rows"]) == 12
    assert native[0]["dropped"] > 0
    assert any(row[0] == "scan.skipped" for row in native[0]["rows"])
    bracket = payload["metadata"]["native_clock_bracket_ns"]
    assert bracket[0] <= bracket[1] <= bracket[2]
