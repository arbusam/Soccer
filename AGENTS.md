Any time you don't understand something about how this project works, try and figure it out. If you still don't understand, ask for help. Once you figure it out/get an answer, add an explanation of the problem and how to solve it to this file, so you don't run into the same problem again. Do not just save every change you make here, only add to this file if you didn't understand something and you had to spend time working it out.
Whenever you finish writing code, use `ruff check` to lint it.

## Library (motor / movement)

**Movement module (`movement.py`)** – main API for driving motors:

- **`init_motors(i2c_addresses, calibration_file="calibration_data.json")`** → `(motors, motor_modes)`. Creates up to 8 `PowerfulBLDCDriver` instances, sets PID/limits, loads saved calibration from the JSON file, applies `set_ELECANGLEOFFSET` / `set_SINCOSCENTRE`, and puts motors in FOC speed mode (command mode 12). Exits with an error if the calibration file is missing or has fewer motors than requested. Use this for normal operation.
- **`get_motors_for_calibration(i2c_addresses)`** → `(motors, motor_count, normalized_addresses)`. Creates drivers and sets PID/limits only (no calibration, no FOC). Used by `calibrate.py`.
- **`calibrate_motors(motors, motor_count, i2c_addresses, calibration_file="calibration_data.json")`**. Puts each motor in calibration mode, runs physical calibration, reads `ELECANGLEOFFSET` and `SINCOSCENTRE`, and writes them to the JSON file. Call this only when you need to (re)calibrate; use `calibrate.py` as the entry point.
- **`move(direction, speed, rotation, rotation_speed, yaw, motors, motor_modes, diameter, max_yaw_rpm, max_rpm, yaw_correct_threshold)`**. Mecanum-style 4-wheel drive: `direction` and `speed` (mm/s), `rotation` and `yaw` (degrees). `rotation_speed` is a 0.0-1.0 scalar that sets yaw-correction strength. Writes RPM setpoints to `motors[0]`–`motors[3]` and calls `update_quick_data_readout()` on each. Expects exactly the `motors` and `motor_modes` returned by `init_motors`.
- **`_prompt_i2c_addresses()`** → list of I2C addresses. Interactive prompt (number of motors, then each address). Used by scripts that don't hardcode addresses.

**External driver:** `steelbar_powerful_bldc_driver.PowerfulBLDCDriver(i2c_bus, address)` (e.g. from `git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver`). Driver firmware version must be 3. Important methods: `set_ELECANGLEOFFSET`, `set_SINCOSCENTRE`, `get_calibration_ELECANGLEOFFSET`, `get_calibration_SINCOSCENTRE`, `set_speed`, `get_speed_QDR`, `update_quick_data_readout`, plus PID/limits and mode config.

**Calibration workflow:** Run `python calibrate.py` once (or after hardware change) to create/overwrite `calibration_data.json`. After that, `init_motors(...)` loads that file and does not run physical calibration. Calibration file format: `{"motors": [{"address": <int>, "elecangleoffset": <int>, "sincoscentre": <int>}, ...]}`.

## Movement: speed (mm/s) to motor RPM (`movement.py`)

**Problem:** What are `a_speed`, `b_speed`, `c_speed`, `d_speed`, and `max_trans_rpm` when `speed` is a given value (e.g. 500)?

**How it works:**
- `speed` is in mm/s. Direction and yaw set `local_direction = direction - yaw - 45` (degrees).
- Wheel velocities in mm/s: `a_value = sin(local_direction)*speed`, `b_value = cos(local_direction)*speed`, `c_value = -a_value`, `d_value = -b_value`.
- Conversion to RPM uses wheel diameter (e.g. `WHEEL_DIAMETER = 50` mm in `defence.py`): `mmps_to_rpm = 60 / (diameter * π)`.
- Motor speeds in RPM: `a_speed = a_value * mmps_to_rpm`, and similarly for b, c, d.
- `max_trans_rpm = max(|a_speed|, |b_speed|, |c_speed|, |d_speed|, 1e-6)` (the 1e-6 avoids division by zero later). If this exceeds `max_rpm`, all four are scaled down so the max equals `max_rpm`.
- With diameter 50 mm: `mmps_to_rpm ≈ 0.382`. Example: speed 500, local_direction 0° → b_speed ≈ 191, d_speed ≈ -191, max_trans_rpm = 191.

## Raspberry Pi: "Bus error" when importing `numpy`/`cv2`

**Symptom:** Running `defence.py` or `camera.py` crashes with `Bus error` (SIGBUS), often before any Python traceback.

**Cause (observed on this project):** The Raspberry Pi kernel is configured with **16KB pages** (`getconf PAGE_SIZE` → `16384`). Some `pip` wheels for native libs (notably `numpy`, and therefore `opencv-python`) can **SIGBUS on 16KB-page systems**. In this repo, importing `numpy` inside the project `.venv` triggered the SIGBUS during `numpy._core.multiarray` import.

**Fix:** Prefer OS-packaged builds for native libs on the Pi:

- Install: `sudo apt-get install python3-numpy python3-opencv python3-libcamera python3-picamera2`
- Run scripts with system Python (`python3 ...`), or create a venv that uses system packages (`python3 -m venv .venv --system-site-packages`) and avoid `pip` installing `numpy/opencv-python/picamera2` into the venv.
