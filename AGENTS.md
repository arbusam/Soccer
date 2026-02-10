Any time you don't understand something about how this project works, try and figure it out. If you still don't understand, ask for help. Once you figure it out/get an answer, add an explanation of the problem and how to solve it to this file, so you don't run into the same problem again. Do not just save every change you make here, only add to this file if you didn't understand something and you had to spend time working it out.

## Library (motor / movement)

**Movement module (`movement.py`)** – main API for driving motors:

- **`init_motors(i2c_addresses, calibration_file="calibration_data.json")`** → `(motors, motor_modes)`. Creates up to 8 `PowerfulBLDCDriver` instances, sets PID/limits, loads saved calibration from the JSON file, applies `set_ELECANGLEOFFSET` / `set_SINCOSCENTRE`, and puts motors in FOC speed mode (command mode 12). Exits with an error if the calibration file is missing or has fewer motors than requested. Use this for normal operation.
- **`get_motors_for_calibration(i2c_addresses)`** → `(motors, motor_count, normalized_addresses)`. Creates drivers and sets PID/limits only (no calibration, no FOC). Used by `calibrate.py`.
- **`calibrate_motors(motors, motor_count, i2c_addresses, calibration_file="calibration_data.json")`**. Puts each motor in calibration mode, runs physical calibration, reads `ELECANGLEOFFSET` and `SINCOSCENTRE`, and writes them to the JSON file. Call this only when you need to (re)calibrate; use `calibrate.py` as the entry point.
- **`move(direction, speed, rotation, yaw, motors, motor_modes, diameter, max_yaw_rpm, max_rpm, yaw_correct_threshold)`**. Mecanum-style 4-wheel drive: `direction` and `speed` (mm/s), `rotation` and `yaw` (degrees). Writes RPM setpoints to `motors[0]`–`motors[3]` and calls `update_quick_data_readout()` on each. Expects exactly the `motors` and `motor_modes` returned by `init_motors`.
- **`_prompt_i2c_addresses()`** → list of I2C addresses. Interactive prompt (number of motors, then each address). Used by scripts that don't hardcode addresses.

**External driver:** `steelbar_powerful_bldc_driver.PowerfulBLDCDriver(i2c_bus, address)` (e.g. from `git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver`). Movement uses this, not `motor_driver.MotorDriver`. Driver firmware version must be 3. Important methods: `set_ELECANGLEOFFSET`, `set_SINCOSCENTRE`, `get_calibration_ELECANGLEOFFSET`, `get_calibration_SINCOSCENTRE`, `set_speed`, `get_speed_QDR`, `update_quick_data_readout`, plus PID/limits and mode config.

**`motor_driver.py`:** Defines `MotorDriver`, a lower-level I2C protocol wrapper with a different API (e.g. `set_sensor_angle_offset_direction`, `set_sensor_offset`). Not used by `movement.py`; movement talks to the hardware via `PowerfulBLDCDriver` only.

**Calibration workflow:** Run `python calibrate.py` once (or after hardware change) to create/overwrite `calibration_data.json`. After that, `init_motors(...)` loads that file and does not run physical calibration. Calibration file format: `{"motors": [{"address": <int>, "elecangleoffset": <int>, "sincoscentre": <int>}, ...]}`.