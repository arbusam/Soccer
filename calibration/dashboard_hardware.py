"""Single-owner hardware worker for the LAN calibration dashboard."""

import copy
import json
import math
import queue
import threading
import time
from collections import deque

PITCH = (2430, 1820)


def target_command(pose, target, speed):
    """Return direction/speed/yaw and the RPM-limited speed for dashboard display."""
    x, y, yaw = pose[:3]
    dx, dy = target[0] - x, target[1] - y
    distance = math.hypot(dx, dy)
    direction = math.degrees(math.atan2(dy, dx))
    requested = min(speed, distance / 300.0 * speed)
    angle = math.radians(yaw - direction + 45)
    peak = max(abs(math.sin(angle)), abs(math.cos(angle)))
    limit = 400 * 50 * math.pi / 60 / peak
    return direction, requested, yaw, min(requested, limit), distance <= 10


class Hardware:
    def __init__(self, root, notify, *, port="/dev/ttyUSB0", baud=460800):
        self.root = root
        self.notify = notify
        self.port = port
        self.baud = baud
        self.lock = threading.RLock()
        self.controller = None
        self.session = None
        self.pause = None
        self.active_cancel = threading.Event()
        self.localisation_stop_requested = threading.Event()
        self.target = None
        self.speed = 500
        self.motion_cancel = threading.Event()
        self.motion_cancel.set()
        self.jobs = queue.Queue(maxsize=4)
        self.closing = threading.Event()
        self.status = {"mode": "idle", "localisation": None, "error": None}
        self.trail = deque(maxlen=300)
        self.thread = threading.Thread(target=self._run, name="dashboard-hardware", daemon=True)
        self.thread.start()

    def submit(self, action, data, cancel):
        with self.lock:
            if self.status["mode"] not in ("idle", "stopped", "monitoring"):
                raise ValueError("Hardware operation already running")
            self.jobs.put_nowait((action, data, cancel))
            self.status["mode"] = "queued " + action

    def snapshot(self):
        with self.lock:
            return copy.deepcopy({**self.status, "target": self.target, "trajectory": list(self.trail)})

    def stop_drive(self):
        with self.lock:
            controller, self.controller = self.controller, None
            self.target = None
            if controller is None:
                return
            self.status["mode"] = "stopping"
        errors = []
        try:
            controller.stop()
        except Exception as exc:
            errors.append(str(exc))
        finally:
            from lib.movement import disable_calibration_motors
            errors.extend(disable_calibration_motors(controller.motors))
            with self.lock:
                if errors:
                    self.status["error"] = "Motor shutdown failed: " + "; ".join(errors)
                self.status["mode"] = "monitoring" if self.session else "idle"
        if errors:
            self.notify("Motor shutdown failed: " + "; ".join(errors), error=True)

    def _localise(self, cancel):
        if self.session is not None:
            return
        from lib import lidar
        from lib.imu import IMU
        from lib.localisation_service import (
            LocalisationSession,
            capture_startup_yaw,
            feed_imu_yaw_prior,
        )
        from lib.movement import LidarVelocityEstimator

        imu = None
        try:
            lidar.init(self.port, self.baud)
            imu = IMU()
            startup = capture_startup_yaw(imu, cancel_event=cancel)
            feed_imu_yaw_prior(lidar, imu, startup)
            lidar.start_coordinates(*PITCH)
            self.session = LocalisationSession(lidar, imu, startup, LidarVelocityEstimator())
        except BaseException:
            if imu is not None:
                imu.close()
            lidar.shutdown()
            raise

    def request_stop_localisation(self):
        """Ask the hardware thread to stop driving and release LIDAR/IMU."""
        self.active_cancel.set()
        self.motion_cancel.set()
        self.localisation_stop_requested.set()
        with self.lock:
            self.status["mode"] = "stopping localisation"

    def _stop_localisation(self):
        self.stop_drive()
        with self.lock:
            session, self.session = self.session, None
        if session is not None:
            session.close()
        with self.lock:
            self.status.update(mode="idle", localisation=None, error=None)
            self.trail.clear()
        self.localisation_stop_requested.clear()
        self.notify("Localisation stopped")

    def _drive(self, data, cancel):
        from lib.config import load_config
        from lib.movement import MovementController
        from lib.switch import Switch

        if self.session is None or not self.session.state.get("fresh"):
            raise ValueError("Start localisation and wait for a fresh pose before driving")
        if not self.session.state["pose"][4]:
            raise ValueError("No confident pose")
        if self.pause is None:
            self.pause = Switch(load_config().pause_switch_pin)
        if self.pause.read():
            raise ValueError("Physical pause switch is active")
        saved = json.loads((self.root / "calibration_data.json").read_text())
        saved_addresses = [motor["address"] for motor in saved["motors"]]
        if saved_addresses[:len(data["addresses"])] != data["addresses"]:
            raise ValueError("Drive addresses/order must match the saved motor calibration")
        self.stop_drive()
        controller = MovementController.from_i2c_addresses(
            data["addresses"], 50, 100, 400, 3,
            calibration_file=str(self.root / "calibration_data.json"),
        )
        with self.lock:
            if cancel.is_set():
                controller.stop()
                return
            self.controller = controller
            self.target = data["target"]
            self.speed = data["speed"]
            self.motion_cancel = cancel
            self.status["mode"] = "driving"

    def _calibrate(self, data, cancel):
        from lib.movement import calibrate_motors, get_motors_for_calibration

        self.stop_drive()
        if self.session is not None:
            self.session.close()
            self.session = None
        with self.lock:
            self.status.update(mode="calibrating", localisation=None)
        motors, count, addresses = get_motors_for_calibration(data["addresses"])

        def progress(info):
            with self.lock:
                self.status["calibration"] = info

        result = calibrate_motors(
            motors, count, addresses, str(self.root / "calibration_data.json"),
            cancel_event=cancel, progress=progress, timeout_s=120,
        )
        with self.lock:
            self.status["calibration"] = {"results": result["motors"], "complete": True}
        self.notify("Motor calibration saved")

    def _run(self):
        next_map = 0
        try:
            while not self.closing.is_set():
                started = time.monotonic()
                if self.localisation_stop_requested.is_set():
                    self._stop_localisation()
                    continue
                try:
                    action, data, cancel = self.jobs.get_nowait()
                except queue.Empty:
                    action = None
                try:
                    if action is not None and cancel.is_set():
                        with self.lock:
                            self.status["mode"] = "monitoring" if self.session else "idle"
                    if action is not None and not cancel.is_set():
                        self.active_cancel = cancel
                        with self.lock:
                            self.status.update(mode="starting " + action, error=None)
                        if action == "localise":
                            self._localise(cancel)
                        elif action == "drive":
                            self._drive(data, cancel)
                        elif action == "calibrate":
                            self._calibrate(data, cancel)
                            cancel.set()
                        if self.controller is None:
                            with self.lock:
                                self.status["mode"] = "monitoring" if self.session else "idle"
                    if self.session is not None:
                        with self.lock:
                            controller = self.controller
                        state = self.session.tick(controller)
                        if time.monotonic() >= next_map:
                            state["scan"] = list(self.session.lidar.get_scan_list())[::2]
                            next_map = time.monotonic() + 0.2
                            if state["pose"][4]:
                                with self.lock:
                                    self.trail.append(state["pose"][:3])
                        else:
                            state["scan"] = (self.status.get("localisation") or {}).get("scan", [])
                        with self.lock:
                            self.status["localisation"] = state
                        if controller is not None:
                            if (self.motion_cancel.is_set() or not state["fresh"]
                                    or not state["pose"][4] or self.pause.read()):
                                raise InterruptedError("Drive stopped: pause, lost control, or stale/invalid pose")
                            with self.lock:
                                target = self.target
                            if target is not None:
                                direction, speed, yaw, limited, arrived = target_command(
                                    state["pose"], target, self.speed,
                                )
                                with self.lock:
                                    self.status["drive"] = {
                                        "requested_speed": self.speed, "ramped_request": speed,
                                        "rpm_limited_speed": limited, "rpm_limited": limited < speed,
                                    }
                                if arrived:
                                    self.motion_cancel.set()
                                    self.stop_drive()
                                    self.notify("Target reached; disarmed")
                                else:
                                    controller.move(direction, speed, yaw, 1.0, yaw)
                except (Exception, SystemExit) as exc:
                    stopping_localisation = (
                        action == "localise"
                        and self.localisation_stop_requested.is_set()
                    )
                    self.motion_cancel.set()
                    if action is not None:
                        cancel.set()
                    self.stop_drive()
                    if stopping_localisation:
                        self._stop_localisation()
                        continue
                    with self.lock:
                        self.status["error"] = str(exc) or "Hardware initialization failed"
                        if self.status["mode"] != "stopping":
                            self.status["mode"] = "stopped"
                    self.notify(str(exc) or "Hardware initialization failed", error=True)
                    if action is None and self.session is not None:
                        with self.lock:
                            if self.status["localisation"] is not None:
                                self.status["localisation"]["fresh"] = False
                        self.session.close()
                        self.session = None
                self.closing.wait(max(0, 0.02 - (time.monotonic() - started)))
        finally:
            self.stop_drive()
            if self.session is not None:
                self.session.close()
            if self.pause is not None:
                self.pause.switch.deinit()

    def close(self):
        self.motion_cancel.set()
        self.active_cancel.set()
        self.localisation_stop_requested.set()
        self.closing.set()
        self.stop_drive()
        self.thread.join(timeout=6)
