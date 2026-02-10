Any time you don't understand something about how this project works, try and figure it out. If you still don't understand, ask for help. Once you figure it out/get an answer, add an explanation of the problem and how to solve it to this file, so you don't run into the same problem again.

---

## Movement: speed (mm/s) to motor RPM (`movement.py`)

**Problem:** What are `a_speed`, `b_speed`, `c_speed`, `d_speed`, and `max_trans_rpm` when `speed` is a given value (e.g. 500)?

**How it works:**
- `speed` is in mm/s. Direction and yaw set `local_direction = direction - yaw - 45` (degrees).
- Wheel velocities in mm/s: `a_value = sin(local_direction)*speed`, `b_value = cos(local_direction)*speed`, `c_value = -a_value`, `d_value = -b_value`.
- Conversion to RPM uses wheel diameter (e.g. `WHEEL_DIAMETER = 50` mm in `defence.py`): `mmps_to_rpm = 60 / (diameter * π)`.
- Motor speeds in RPM: `a_speed = a_value * mmps_to_rpm`, and similarly for b, c, d.
- `max_trans_rpm = max(|a_speed|, |b_speed|, |c_speed|, |d_speed|, 1e-6)` (the 1e-6 avoids division by zero later). If this exceeds `max_rpm`, all four are scaled down so the max equals `max_rpm`.
- With diameter 50 mm: `mmps_to_rpm ≈ 0.382`. Example: speed 500, local_direction 0° → b_speed ≈ 191, d_speed ≈ -191, max_trans_rpm = 191.