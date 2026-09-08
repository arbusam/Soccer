"""Hardware-free localization and GIL regression tests. Build lib.lidar first."""

import json
import math
import subprocess
import sys
import threading
from pathlib import Path

import pytest

from lib import lidar

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "expression",
    [
        "lidar.set_imu_yaw(0)",
        "lidar.predict_odometry(0, 0, 0, .01)",
        "lidar.get_pose()",
        "lidar.get_coordinates()",
        "lidar.get_coordinates_info()",
        "lidar.is_coordinates_ready()",
        "lidar.scan_updates_enabled()",
        "lidar.get_last_scan_correction()",
        "lidar.get_recovery_status()",
        "lidar.get_mcl_update_count()",
    ],
)
def test_python_progress_during_native_mutex_wait(expression):
    # A subprocess bounds even a deadlock and gives each case clean native globals.
    source = f"""
import json, threading, time
from lib import lidar
lidar.test_mcl_start(2430, 1820)
ready = threading.Event()
stop = threading.Event()
beats = []
def heartbeat():
    ready.set()
    while not stop.wait(.002):
        beats.append(time.monotonic_ns())
t = threading.Thread(target=heartbeat)
t.start()
ready.wait()
lidar.test_mcl_hold_mutex(200)
start = time.monotonic_ns()
{expression}
end = time.monotonic_ns()
released = lidar.test_mcl_mutex_released_ns()
stop.set()
t.join()
print(json.dumps({{"progress": sum(start + 20_000_000 < b < released for b in beats),
                  "elapsed_ms": (end-start)/1e6,
                  "release": lidar.runtime_gil_released}}))
"""
    result = subprocess.run(
        [sys.executable, "-c", source],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=5,
        check=True,
    )
    data = json.loads(result.stdout)
    assert data["elapsed_ms"] >= 100, data
    if data["release"]:
        assert data["progress"] > 0, data
    else:
        assert data["progress"] == 0, data


def synthetic_scan(x=950, y=650, yaw=0, partial=False):
    # Independent ray/segment intersection including goal hardware.
    segments = [
        ((0, 0), (2430, 0)),
        ((2430, 0), (2430, 1820)),
        ((2430, 1820), (0, 1820)),
        ((0, 1820), (0, 0)),
        ((300, 685), (0, 685)),
        ((226, 685), (226, 1140)),
        ((0, 1135), (300, 1135)),
        ((2130, 685), (2430, 685)),
        ((2204, 685), (2204, 1140)),
        ((2430, 1135), (2130, 1135)),
    ]
    scan = []
    for angle in range(0, 360, 2):
        theta = math.radians(angle + yaw)
        ux, uy = math.cos(theta), math.sin(theta)
        hits = []
        for (ax, ay), (bx, by) in segments:
            sx, sy = bx - ax, by - ay
            denominator = ux * sy - uy * sx
            if abs(denominator) < 1e-9:
                continue
            distance = ((ax - x) * sy - (ay - y) * sx) / denominator
            position = ((ax - x) * uy - (ay - y) * ux) / denominator
            if distance > 0 and 0 <= position <= 1:
                incidence = abs(ux * sy - uy * sx) / math.hypot(sx, sy)
                hits.append((distance, incidence))
        distance, incidence = min(hits)
        hit = incidence >= 0.25
        if not partial or angle < 220:
            scan.append((float(angle), distance if hit else 0.0, 15 if hit else 0, hit))
    return scan


def converge(scan, steps=100):
    for _ in range(steps):
        lidar.test_mcl_predict(0, 0, 0, 0.02)
        lidar.test_mcl_update_scan(scan)
    return lidar.get_pose()


def test_convergence_partial_scans_prediction_and_rotation_gate():
    lidar.test_mcl_start(2430, 1820)
    try:
        lidar.test_mcl_set_imu_yaw(0)
        lidar.test_mcl_reset()
        x, y, yaw, confidence = converge(synthetic_scan())
        assert x is not None, lidar.get_coordinates_info()
        assert math.hypot(x - 950, y - 650) < 100
        assert abs(yaw) < 8
        assert 0 <= confidence <= 1
        partial = converge(synthetic_scan(partial=True), steps=30)
        assert partial[0] is not None
        assert math.hypot(partial[0] - 950, partial[1] - 650) < 150
        before = lidar.get_pose()
        lidar.predict_odometry(200, 0, 0, 0.1)
        after = lidar.get_pose()
        assert 10 < after[0] - before[0] < 30
        lidar.predict_odometry(0, 0, 60, 0.02)
        assert not lidar.scan_updates_enabled()
        before = lidar.get_coordinates_info()
        lidar.test_mcl_update_scan(synthetic_scan())
        assert lidar.get_coordinates_info() == before
        for _ in range(10):
            lidar.predict_odometry(0, 0, 0, 0.02)
        assert lidar.scan_updates_enabled()
    finally:
        lidar.test_mcl_stop()


def test_snapshots_during_scan_updates():
    lidar.test_mcl_start(2430, 1820)
    errors = []

    def update():
        try:
            for _ in range(25):
                lidar.test_mcl_update_scan(synthetic_scan())
        except Exception as exc:
            errors.append(exc)

    worker = threading.Thread(target=update, daemon=True)
    try:
        worker.start()
        for _ in range(100):
            lidar.set_imu_yaw(0)
            lidar.predict_odometry(0, 0, 0, 0.01)
            assert len(lidar.get_pose()) == 4
            assert len(lidar.get_coordinates_info()) == 5
            assert isinstance(lidar.is_coordinates_ready(), bool)
        worker.join(timeout=5)
        assert not worker.is_alive()
        assert not errors
    finally:
        lidar.test_mcl_stop()
