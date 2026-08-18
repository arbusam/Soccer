Any time you don't understand something about how this project works, try and figure it out. If you still don't understand, ask for help. Once you figure it out/get an answer, add an explanation of the problem and how to solve it to this file, so you don't run into the same problem again. Do not just save every change you make here, only add to this file if you didn't understand something and you had to spend time working it out.
Whenever you finish writing code, lint with `.venv/bin/ruff check` (or `.venv/bin/ruff check <path>`). If that does not work (for example `.venv` is missing or the command fails), tell the user. CI uses Ruff **0.16.2** (pinned in `.github/workflows/ruff.yml`); install the same version locally (`pip install 'ruff>=0.16.2'`) so local results match GitHub Actions. Ruff 0.15.x has fewer default rules and will pass checks that 0.16 fails.

## Library (motor / movement)

**Movement module (`movement.py`)** – main API for driving motors:

- **`init_motors(i2c_addresses, calibration_file="calibration_data.json")`** → `(motors, motor_modes)`. Creates up to 8 `PowerfulBLDCDriver` instances, sets PID/limits, loads saved calibration from the JSON file, applies `set_ELECANGLEOFFSET` / `set_SINCOSCENTRE`, and puts motors in FOC speed mode (command mode 12). Exits with an error if the calibration file is missing or has fewer motors than requested. Use this for normal operation.
- **`get_motors_for_calibration(i2c_addresses)`** → `(motors, motor_count, normalized_addresses)`. Creates drivers and sets PID/limits only (no calibration, no FOC). Used by `calibrate.py`.
- **`calibrate_motors(motors, motor_count, i2c_addresses, calibration_file="calibration_data.json")`**. Puts each motor in calibration mode, runs physical calibration, reads `ELECANGLEOFFSET` and `SINCOSCENTRE`, and writes them to the JSON file. Call this only when you need to (re)calibrate; use `calibrate.py` as the entry point.
- **`MovementController.move(direction, speed, rotation, rotation_speed, yaw, dribbler=False)`**. Non-blocking: only updates command targets. A background drive thread (50 Hz) owns accel ramping, yaw correction, and I2C speed writes. `direction` / `rotation` / `yaw` use the project frame where `0` = startup-forward and `90` = startup-right (clockwise positive). `rotation_speed` is a 0.0-1.0 scalar for yaw-correction strength.
- **`imu_yaw_to_relative_yaw(imu_yaw, startup_yaw)`** → `float`. Convert raw IMU yaw into the project's startup-relative heading frame. The upside-down IMU reports clockwise turns as decreasing yaw, so the correct conversion is `wrap_angle_deg(startup_yaw - imu_yaw)`.
- **`_prompt_i2c_addresses()`** → list of I2C addresses. Interactive prompt (number of motors, then each address). Used by scripts that don't hardcode addresses.

**Problem:** Accel and yaw correction used to run inside `move()`, so a stalled control loop (slow LIDAR/camera/strategy) produced a huge `dt` (accel jumps) and paused heading correction between calls.

**Solution:** `MovementController` runs a fixed-rate drive thread. `move()` only stores targets under a lock; the thread applies `MAX_VELOCITY_CHANGE_PER_SEC` with a capped `dt`, recomputes yaw correction, and writes motors. Callers can still stall without distorting the ramp or freezing yaw hold (yaw stays at the last value passed to `move()` until the next call).

**External driver:** `steelbar_powerful_bldc_driver.PowerfulBLDCDriver(i2c_bus, address)` (e.g. from `git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver`). Driver firmware version must be 3. Important methods: `set_ELECANGLEOFFSET`, `set_SINCOSCENTRE`, `get_calibration_ELECANGLEOFFSET`, `get_calibration_SINCOSCENTRE`, `set_speed`, `get_speed_QDR`, `update_quick_data_readout`, plus PID/limits and mode config.

**Calibration workflow:** Run `python calibrate.py` once (or after hardware change) to create/overwrite `calibration_data.json`. After that, `init_motors(...)` loads that file and does not run physical calibration. Calibration file format: `{"motors": [{"address": <int>, "elecangleoffset": <int>, "sincoscentre": <int>}, ...]}`.

## Movement: speed (mm/s) to motor RPM (`movement.py`)

**Problem:** What are `a_speed`, `b_speed`, `c_speed`, `d_speed`, and `max_trans_rpm` when `speed` is a given value (e.g. 500)?

**How it works:**
- `speed` is in mm/s. `direction`, `rotation`, and `yaw` use the project heading frame where `0` = startup-forward and `90` = startup-right. The IMU does **not** natively use this sign convention; convert raw IMU yaw with `imu_yaw_to_relative_yaw(imu_yaw, startup_yaw)` before passing it into `move()`.
- `movement.py` converts the global translation heading into the robot's local frame with `local_direction = yaw - direction + 45` (degrees). The `+45` rotates into the wheel basis because the wheels sit on the diagonals, leaving the front edge clear.
- Wheel velocities in mm/s use the current code signs: `a_value = -sin(local_direction)*speed`, `b_value = -cos(local_direction)*speed`, `c_value = -a_value`, `d_value = -b_value`.
- Conversion to RPM uses wheel diameter (e.g. `WHEEL_DIAMETER = 50` mm in `defence.py`): `mmps_to_rpm = 60 / (diameter * π)`.
- Motor speeds in RPM: `a_speed = a_value * mmps_to_rpm`, and similarly for b, c, d.
- Yaw correction uses `yaw_error = wrap_angle_deg(rotation - yaw)`, so positive error means "turn clockwise to match the target heading". The requested yaw RPM is limited to `max_yaw_rpm`, then the translation RPMs are scaled down if needed to leave headroom for that yaw correction while preserving the requested movement direction.
- With diameter 50 mm: `mmps_to_rpm ≈ 0.382`. Example: speed 500, `direction = yaw = 0` gives `local_direction = 45°`, which corresponds to local forward in the current wheel basis.

**Problem:** QDR wheel odometry had the correct forward speed (`vx`) but the opposite sign on `vy`.

**Solution:** This project's yaw is clockwise-positive and field `+y` is right (`90°`). Body `vy` is defined as **left**, so convert global velocity with `body_vy = gx * sin(yaw) - gy * cos(yaw)` (the left axis is `(sin yaw, -cos yaw)`). The ROS/CCW formula `−gx sin + gy cos` is the **right** axis here and will flip `vy`. `loc_predict_odometry` uses the same left-positive body frame; rebuild the C++ extension after changing it.

## LIDAR localization (`lidar_module.cpp` + `localisation.cpp`)

**Solution — 3-DOF Monte Carlo localization (MCL) in C++:**

- Particles track `(x, y, yaw)` on the known pitch map (outer walls + goal hardware segments).
- Scan thread keeps explicit no-return bearings (`hit=false`) as well as valid hits. MCL bins them into 2° angular bins before scoring.
- Likelihood is incidence-aware: ray casting returns range + wall normal; near-normal walls are expected to return, grazing walls (`|cos θ|` ≲ 0.25) are expected to miss. Hits use a Gaussian+outlier mixture with incidence-scaled σ; misses get a soft expected/unexpected miss probability instead of free-space penalties.
- Confidence combines inlier quality, particle spread, and visible wall-normal diversity (one wall alone cannot claim a strong along-wall pose). Resample only when ESS drops below 50% of the particle count so partial scans keep diversity.
- `predict_odometry(vx, vy, omega, dt)` propagates particles between scans (call from Python each control loop). Pass IMU gyro z as `omega_deg_s` (clockwise positive). `vx`/`vy` are body-frame mm/s with **vx = forward** and **vy = left**.
- Estimation runs in a background thread; Python reads the latest pose.
- Fast rotation gate: when `|omega|` exceeds 50 deg/s, MCL skips LIDAR scan updates and dead-reckons via predict only. Scan updates resume after `|omega|` stays below 25 deg/s for 150 ms. `lidar.scan_updates_enabled()` reports whether scans are currently accepted.

**Problem:** LIDAR often drops walls at extreme incidence angles. The old hit-only Gaussian treated every surviving return as a perfect first-wall match and ignored missing bearings, so partial scans could under-constrain or destabilize the pose.

**Solution:** Model visibility from incidence angle; score explicit misses; lower confidence when geometry is under-constrained. Offline check: `python test_grazing_localisation.py` (uses `lidar.test_mcl_*` hooks, no hardware).

**Python API:**

1. `lidar.init(port, baudrate)` — start scan thread.
2. `lidar.start_coordinates(pitch_x, pitch_y)` — start MCL thread and build pitch map.
3. `lidar.set_imu_yaw(yaw_deg)` — feed startup-relative IMU yaw as a soft MCL yaw prior (call each control loop).
4. `lidar.predict_odometry(vx_mm_s, vy_mm_s, omega_deg_s, dt_s)` — propagate particles between scans (also drives the fast-rotation gate from `omega_deg_s`).
5. `lidar.get_pose()` → `(x, y, yaw_deg, confidence)` — last estimate (None if not confident).
6. `lidar.get_coordinates()` → `(x, y)` — backward-compatible confident position only.
7. `lidar.get_coordinates_info()` → `(x, y, yaw_deg, confidence, ok)` — diagnostics.
8. `lidar.is_coordinates_ready()` → `bool` — true once first confident pose exists.
9. `lidar.scan_updates_enabled()` → `bool` — false while MCL is pausing LIDAR updates during fast rotation.
10. `lidar.shutdown()` — stops localization, scan thread, and motor.

**IMU in MCL:** `imu.py` enables `BNO_REPORT_GYROSCOPE` and `BNO_REPORT_ROTATION_VECTOR`. Gyro: `get_gyro_z_deg_s()` (clockwise positive in project frame) is passed as `omega_deg_s` to `predict_odometry` each control loop. Absolute yaw: convert with `imu_yaw_to_relative_yaw(imu_yaw, startup_yaw)` and call `lidar.set_imu_yaw(...)` each loop (and before the first pose wait). MCL applies a soft Gaussian yaw prior (σ = 45°) on every scan update so LIDAR still owns fine, drift-free heading while the IMU breaks the 180° field symmetry. Init/recovery particles sample yaw around the IMU when a reading exists, otherwise over ±180°.

**Stuck on "Waiting for first pose estimate...":** While waiting for `is_coordinates_ready()`, still call `predict_odometry(0, 0, omega, dt)` each loop (even when stationary). Predict applies translation/yaw process noise; without it, particles collapse after the first resample and often never reach the confidence threshold. Also print `get_coordinates_info()` / `get_scan_count()` during the wait so low confidence vs empty scans is visible.

**Rebuild after changing `lidar_module.cpp` or `localisation.cpp`:** `python setup.py build_ext --inplace`

**Build error `cannot find -lsl_lidar_sdk` / g++ exit code 1:** The Python extension links against the RPLidar SDK static library. If `rplidar_sdk/output/Linux/Release/` does not exist or has no `libsl_lidar_sdk.a`, build the SDK first: `make -C rplidar_sdk/sdk`. Then run `python setup.py build_ext --inplace` again.

## Team-frame convention (`defence.py` / `simulate.py`)

**Problem:** `defence()` and `goalie()` used to contain separate yellow/non-yellow branches. That made the cyan-side behavior drift from the yellow-side behavior, including cases where `defence()` would face its own goal instead of the enemy goal.

**Solution:** Keep the strategy code in a single "yellow-side" frame inside `defence.py`, and let `simulate.py` rotate cyan bots into that frame before calling `defence()` or `goalie()`. The required transform is a 180° rotation of the full world state: `(x, y) -> (PITCH_WIDTH - x, PITCH_HEIGHT - y)` and `yaw/direction/rotation -> angle + 180° (mod 360)`. After the controller returns, rotate `direction` and `rotation` back into the real field frame.

## Label Studio YOLO pre-annotation (`training/ls_yolo_backend.py`)

**Problem:** Manually drawing Ball/Bot boxes on every frame is slow once a detect model already exists.

**Solution:** Run a small Label Studio ML backend that loads `training/runs/open-soccer-detect-m/weights/best.pt` (override with `LS_ML_WEIGHTS`) and returns axis-aligned `RectangleLabels` predictions. Same-host media is resolved from `LABEL_STUDIO_DATA_DIR` (`/data/...` → `label-studio-data/media/...`), so no API token is required.

1. Start: `systemctl --user enable --now label-studio-ml` (unit from `systemd/label-studio-ml.service` → `~/.config/systemd/user/`), or `.venv/bin/python training/ls_yolo_backend.py`.
2. In Label Studio → Open Soccer → Settings → Model, connect `http://127.0.0.1:9090` (interactive preannotations off).
3. Open a task (or use Retrieve predictions) — only `Ball` / `Bot` are emitted; other model classes (e.g. goals) are skipped. Confidence defaults to `0.25` (`LS_ML_CONF`), or `model_score_threshold` on `<RectangleLabels>` if set.
4. After retraining, `systemctl --user restart label-studio-ml` so it reloads `best.pt`.

## Hailo-8 AI HAT+ ball inference (`training/`)

**Problem:** Ultralytics NCNN on the Pi CPU (~5 FPS) never uses the AI HAT+ (Hailo-8, 26 TOPS). Hailo only runs compiled `.hef` models. Training is axis-aligned YOLO26 detect (not OBB).

**Solution:**

1. On the Pi: `bash scripts/verify_hailo.sh` (needs `hailo-all` + `/dev/hailo*`).
2. Optional — rewrite rotated Label Studio boxes to axis-aligned AABBs in the DB so you can review them in the UI (stop Label Studio first):  
   `python training/export_label_studio.py --update-label-studio` (dry-run), then `--update-label-studio --apply`.
3. Export YOLO-detect dataset: `python training/export_label_studio.py` → `training/datasets/open-soccer-detect/`.
4. Train + ONNX (project `.venv`): `python training/train.py --size n` → `training/exports/open-soccer-detect-n/model.onnx` (XPU via `training/xpu_patch.py`; AMP off on XPU; use `--no-mosaic` if XPU scatter/gather errors).
5. On x86 with Hailo DFC: `python training/compile_hailo.py --hw-arch hailo8` → `open-soccer-detect-n_hailo_model/model.hef`.
6. Copy that folder to the Pi. `test_model.py` and `camera.py` use [`hailo_ball.py`](hailo_ball.py) (HailoRT) for inference.

YOLO26’s end2end TopK head is unsupported on Hailo. `compile_hailo.py` cuts at `/model.23/Mul_2` (decoded xyxy boxes) and `/model.23/Sigmoid` (class scores) as **two** HEF outputs so each gets its own INT8 range. The HEF uses input normalization only; `hailo_ball.py` merges the streams and runs conf filter + NMS on the host.

**Empty detections / all class max=0.0 with float32 output:** Usually the HEF was compiled with a single end node (`/model.23/Transpose` or `Concat_3`) that concatenates pixel boxes (~0–640) with sigmoid scores (~0–1). One qp scale (~3+) cannot represent confidences, so dequantized scores collapse to ~0 even though ONNX float inference is fine. Fix: recompile with `python training/compile_hailo.py` (defaults are now `Mul_2` + `Sigmoid`), copy the new `model.hef` to the Pi. Diagnose with `python test_model.py --image model_test_image.png --debug` — you want two outputs and Ball max ≫ 0. Sanity-check the ONNX with onnxruntime if needed.

**UINT8 vs FLOAT32:** Keep HailoRT output as `FormatType.FLOAT32` (`quantized=False`). Manual UINT8 + `quant_info` also drops detections when qp is wrong/missing.

## Striker ball-hiding hysteresis (`striker.py`)

**Problem:** With separate `BALL_HIDING_START_DIST` / `BALL_HIDING_END_DIST`, a naive `if dist < END: aim elif dist >= START: hide` leaves a dead zone between them. Crossing that band (or fluttering near either threshold) snapped `rotation` between wall-facing (`120`/`240`) and `0` / goal heading, so the bot oscillated CW/CCW.

**Solution:** Persist hiding in the returned `steering_state` bool. Enter hide when `dist >= START`; stay hidden until `dist < END`, then aim. `main.py` / `simulate.py` must feed the previous steering flag back in each call.

**Aim angle:** Do not aim at goal-centre Y. Take the angular midpoint of directions that hit `GOAL_BACK_Y_MIN..MAX` on `CYAN_GOAL_BACK_X`, clipped to rays that pass through the goal mouth (`CYAN_GOAL_MOUTH_X`, side walls at `GOAL_SIDE_WALL_Y_MIN/MAX` inset by `BALL_RADIUS`). If the clipped back-wall window is empty, bank onto the inside of the opposite side wall as close to the back as possible, keeping at least `SIDE_WALL_CLEARANCE_DEG` (2°) from the near wall when the mouth sector allows it. Aim/kick directions must also clear both mouth posts by `BALL_RADIUS + GOAL_LINE_WIDTH/2` (angular mouth clearance alone can still clip a post). Kick only when the actual `yaw` (kick impulse direction) passes `kick_direction_scores`, not merely when yaw is close to the aim angle. If neither a back-wall nor a valid rebound shot exists, dribble toward own goal and pitch centre (`x - SHOT_REPOSITION_PULL_X`, `GOAL_CENTRE_Y`) instead of kicking.

## Blinka / `import board` on Pi 5 (`ModuleNotFoundError: lgpio`)

**Problem:** `import board` (Adafruit Blinka) fails on Raspberry Pi 5 with Python 3.13: `ModuleNotFoundError: No module named 'lgpio'`. Blinka’s BCM2712 pin backend imports `lgpio`, which is not always installed as a transitive dep (especially on 3.13).

**Solution:** In the project venv on the Pi: `pip install adafruit-lgpio` (listed in `requirements.txt`). Alternative: `sudo apt install python3-lgpio` and recreate the venv with `--system-site-packages`.
