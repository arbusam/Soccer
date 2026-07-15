Any time you don't understand something about how this project works, try and figure it out. If you still don't understand, ask for help. Once you figure it out/get an answer, add an explanation of the problem and how to solve it to this file, so you don't run into the same problem again. Do not just save every change you make here, only add to this file if you didn't understand something and you had to spend time working it out.
Whenever you finish writing code, lint with `.venv/bin/ruff check` (or `.venv/bin/ruff check <path>`). If that does not work (for example `.venv` is missing or the command fails), tell the user.

## Library (motor / movement)

**Movement module (`movement.py`)** – main API for driving motors:

- **`init_motors(i2c_addresses, calibration_file="calibration_data.json")`** → `(motors, motor_modes)`. Creates up to 8 `PowerfulBLDCDriver` instances, sets PID/limits, loads saved calibration from the JSON file, applies `set_ELECANGLEOFFSET` / `set_SINCOSCENTRE`, and puts motors in FOC speed mode (command mode 12). Exits with an error if the calibration file is missing or has fewer motors than requested. Use this for normal operation.
- **`get_motors_for_calibration(i2c_addresses)`** → `(motors, motor_count, normalized_addresses)`. Creates drivers and sets PID/limits only (no calibration, no FOC). Used by `calibrate.py`.
- **`calibrate_motors(motors, motor_count, i2c_addresses, calibration_file="calibration_data.json")`**. Puts each motor in calibration mode, runs physical calibration, reads `ELECANGLEOFFSET` and `SINCOSCENTRE`, and writes them to the JSON file. Call this only when you need to (re)calibrate; use `calibrate.py` as the entry point.
- **`move(direction, speed, rotation, rotation_speed, yaw, motors, motor_modes, diameter, max_yaw_rpm, max_rpm, yaw_correct_threshold)`**. X-drive / 45° omni drive: `direction` is the global translation heading, `rotation` is the desired robot heading, and `yaw` is the current robot heading, all in the project frame where `0` = startup-forward and `90` = startup-right (clockwise positive). `rotation_speed` is a 0.0-1.0 scalar that sets yaw-correction strength. Writes RPM setpoints to `motors[0]`–`motors[3]`. Expects exactly the `motors` and `motor_modes` returned by `init_motors`.
- **`imu_yaw_to_relative_yaw(imu_yaw, startup_yaw)`** → `float`. Convert raw IMU yaw into the project's startup-relative heading frame. The upside-down IMU reports clockwise turns as decreasing yaw, so the correct conversion is `wrap_angle_deg(startup_yaw - imu_yaw)`.
- **`_prompt_i2c_addresses()`** → list of I2C addresses. Interactive prompt (number of motors, then each address). Used by scripts that don't hardcode addresses.

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

## LIDAR localization (`lidar_module.cpp` + `localisation.cpp`)

**Solution — 3-DOF Monte Carlo localization (MCL) in C++:**

- Particles track `(x, y, yaw)` on the known pitch map (walls + goal hardware segments).
- Each LIDAR scan reweights particles using per-ray Gaussian log-likelihood with a 3σ cap.
- Systematic resampling after each scan; random particle injection on weight collapse for kidnap recovery.
- `predict_odometry(vx, vy, omega, dt)` propagates particles between scans (call from Python each control loop). Pass IMU gyro z as `omega_deg_s` (clockwise positive); leave `vx`/`vy` at 0 until wheel odometry exists.
- Estimation runs in a background thread; Python reads the latest pose.
- Fast rotation gate: when `|omega|` exceeds 50 deg/s, MCL skips LIDAR scan updates and dead-reckons via predict only. Scan updates resume after `|omega|` stays below 25 deg/s for 150 ms. `lidar.scan_updates_enabled()` reports whether scans are currently accepted.

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
