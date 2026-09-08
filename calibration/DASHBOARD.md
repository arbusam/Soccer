# Pi calibration dashboard

From the project root on the Pi:

```bash
.venv/bin/python calibration_dashboard.py
```

Open `http://<pi-ip>:8080` on a phone or computer on the same LAN. The startup message also gives the Pi's `.local` hostname; use its IP if your device does not resolve mDNS. All browser assets are served by the Pi and work without internet access.

Use the project's usual Pi environment: Picamera2/libcamera, OpenCV, HailoRT and the compiled HEF, Blinka and motor drivers, and the compiled LIDAR extension. No additional web framework or Node installation is required. A missing camera/Hailo dependency is shown in the dashboard; it does not prevent the other panels opening. Start localisation separately from its panel; the same button changes to **Stop localisation** while it is starting or running and releases LIDAR and IMU when stopped.

Run standalone: stop `main.py`, camera tests, motor scripts and localisation tests before launching. A lock prevents a second dashboard instance; other existing scripts do not participate in that lock. Anyone on the LAN can view and take control, so use a trusted local network.

## Controls

- **Take control** grants one browser an exclusive lease. It sends a heartbeat every 500 ms. Other devices can view or press **Stop motors**.
- **Arm motors** is required before calibration or target driving. Stop, release, a two-second heartbeat timeout, or an aborted/completed drive disarms the controller. Reconnecting does not resume motion. Switching away from a phone browser may throttle heartbeats and intentionally stop the robot.
- Motor calibration additionally requires the **Wheels are clear** checkbox. Drivers are calibrated in the displayed address order, with a 120-second timeout per driver. Calibration stops localisation; start localisation again afterwards.
- Drive testing requires four calibrated wheels (and optionally the dribbler), addresses in the saved calibration order, fresh LIDAR/MCL/IMU data, and an inactive physical pause switch. Click the field or enter target coordinates, then press **Drive to target**. There is no automatic obstacle avoidance.
- Speed defaults to 500 mm/s. The slider covers 0–1000; the number input covers 0–5000. Numbers over 1000 stay in the number input while the slider sits at its maximum. Existing 400 RPM wheel limits still cap actual speed; the panel reports that cap. Targets slow down inside 300 mm and finish within 10 mm.
- **Stop motors** is a software stop, not a replacement for disconnecting motor power if a driver or I2C connection fails. Shutdown errors are shown in Activity.

## Camera, colours and saved data

The Camera panel overlays the highest-confidence ball and every detected bot on the exact inference frame. Ball distance uses the box centre; bot distance uses the bottom centre and the same radial calibration. Missing distance calibration leaves boxes/bearings available.

The **Detection model** selector lists compiled `open-soccer-detect-*_hailo_model` directories that contain `model.hef`. Changing between Nano (`n`), Small (`s`), or another installed variant restarts only the camera/Hailo pipeline; motor and localisation state are unaffected. The preview briefly reports `switching model`, then shows the active model. Model input dimensions come from each directory's `metadata.yaml`.

**Pick a pixel** freezes an unannotated, lossless frame. Tap it to inspect original RGB, BGR and OpenCV HSV values (H 0–179, S/V 0–255). At most eight frozen frames are retained across viewers; freeze again if an old frame expires. PNG downloads provide raw and annotated snapshots.

Ball samples need a fresh detection and a positive measured distance. Sample several distinct radial positions; duplicate radial positions are rejected to avoid ill-conditioned polynomial fits. **Held by dribbler** preserves the existing capture-calibration metadata. Preview the curve and fit metrics, then Save. Clear/remove only edit the draft until Save.

Goal bounds preview immediately for both blue/cyan and yellow. Save writes the preview bounds; Revert reloads the last save; Restore Defaults changes the draft back to the original values. A lower channel bound must not exceed its upper bound.

Files in the project root:

- `ball_distance_calibration.json`: existing distance format, including samples/resolution; reloaded into the dashboard camera after saving.
- `calibration_data.json`: existing motor format; replaced only after every motor succeeds and shutdown succeeds.
- `goal_thresholds.json`: `blue` and `yellow` objects, each with three-element `lower`/`upper` HSV arrays. Normal `OpenCV()`/camera startup loads these too. Restart another already-running consumer to load changes.
- `calibration_backups/`: timestamped copies made before replacement. To restore, stop the dashboard and copy the chosen JSON back to its original root filename.

**Download debug log** exports current diagnostics, up to 300 activity messages and approximately five minutes of one-second telemetry. The field uses X right, Y down, and clockwise-positive yaw; wheel body velocity uses forward/left. Scan points use the latest displayed pose, so this is a diagnostic overlay, not a motion-deskewed reconstruction. No particle-cloud binding is required.

## Options

```bash
.venv/bin/python calibration_dashboard.py --port 8080 --preview-fps 15
.venv/bin/python calibration_dashboard.py --lidar-port /dev/ttyUSB0 --lidar-baud 460800
```

The default bind address is `0.0.0.0`; `--host 127.0.0.1` restricts access to the Pi itself. Preview FPS may be 1–30. Capture and inference are independent of preview rate. Up to 24 preview connections share encoded buffers; hidden tabs release their streams.

## Validation

Hardware-independent regression tests:

```bash
.venv/bin/python -m pytest tests/dashboard_test.py tests/camera_recording.py
.venv/bin/ruff check
```

The HTTP test needs permission to bind a loopback socket. The tests use fake hardware and do not move real motors.

**Pi acceptance checks remain required before unrestricted drive testing.** They cannot be performed on a desktop without the robot:

1. Check live colours and both model classes against the scene. Freeze a known coloured area and confirm pixel readings. Verify both masks, Save, and restart normal camera operation to confirm saved bounds load.
2. Capture near/middle/far ball samples, save, restart, and compare estimates with measurements. Check the backup and held-ball metadata.
3. With wheels clear, run motor calibration. Test cancellation part-way through a motor, browser disconnection, and timeout handling; confirm each driver actually leaves autonomous calibration mode and stops. Check that failed runs preserve the old JSON. The software selects zero torque and exits calibration command mode; this transition requires verification with the installed firmware.
4. Keep wheels clear for initial drive checks: verify the physical pause switch and browser Stop halt motion, disconnecting the controlling browser stops within two seconds plus hardware response time, and stopping does not resume on reconnection or restored pose. Re-arm and repeat to verify controller recreation.
5. On a clear pitch, begin with a low requested speed. Verify target direction, slowdown, arrival, lost-pose stopping, and reported RPM limiting before testing higher inputs. Disconnect/reconnect sensors while stationary to check health/error reporting.
6. Connect two devices. Confirm both see the same perception results, only one controls edits/motion, and a slow viewer does not interrupt the other. Exit with Ctrl+C and confirm hardware is released.

In **Camera → Camera settings**, take control, enter an **Analogue gain** within the displayed sensor range, and click **Apply gain**. The live preview updates after the camera processes the control. Gain starts at 10× (clamped to the sensor range), survives detection model switches, and resets when the dashboard restarts. It is not saved to normal camera configuration.
