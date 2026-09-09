# Legacy hardware code

These Python hardware implementations are retained for motor calibration and
older standalone utilities. Normal runtime, dashboard driving, and interactive
hardware tests use `lib.hardware_controller`.

`movement.py` remains required by `calibration/motors.py`; it must never run at
the same time as the native controller. `imu.py`, `kicker.py`, `i2c_bus.py`, and
`angles.py` support legacy scripts and reference behavior.
