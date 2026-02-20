"""Fault-injection tests for movement fail-safe (no hardware required)."""
import sys
from unittest.mock import MagicMock, patch

# Mock hardware before importing movement
sys.modules["board"] = MagicMock()
sys.modules["busio"] = MagicMock()
sys.modules["steelbar_powerful_bldc_driver"] = MagicMock()

import movement  # noqa: E402


def test_persistent_failure_raises_and_stops():
    """After PERSISTENT_FAILURE_THRESHOLD consecutive failures on one motor, move() calls stop_all_motors and raises MotorCommsFatalError."""
    movement._motor_consecutive_failures[:] = [0, 0, 0, 0]

    motors = [MagicMock() for _ in range(4)]
    motors[2].set_speed.side_effect = OSError(121, "Remote I/O error")
    motor_modes = [12, 12, 12, 12]

    with patch.object(movement, "stop_all_motors") as stop_all:
        raised = False
        for _ in range(movement.PERSISTENT_FAILURE_THRESHOLD):
            try:
                movement.move(
                    0, 100, 0, 1.0, 0,
                    motors, motor_modes,
                    diameter=50, max_yaw_rpm=100, max_rpm=400, yaw_correct_threshold=3,
                )
            except movement.MotorCommsFatalError as e:
                raised = True
                assert "motor(s)" in str(e)
                break
        assert raised, "Expected MotorCommsFatalError after persistent failures"

    stop_all.assert_called()
    assert stop_all.call_args[0][0] is motors


def test_stop_all_motors_continues_after_failure():
    """stop_all_motors attempts all motors even when one raises."""
    movement._motor_consecutive_failures[:] = [0, 0, 0, 0]

    motors = [MagicMock() for _ in range(3)]
    motors[1].set_speed.side_effect = OSError(121, "Remote I/O error")

    movement.stop_all_motors(motors)

    motors[0].set_speed.assert_called_once_with(0)
    motors[1].set_speed.assert_called_once_with(0)
    motors[2].set_speed.assert_called_once_with(0)


if __name__ == "__main__":
    test_persistent_failure_raises_and_stops()
    test_stop_all_motors_continues_after_failure()
    print("All movement fail-safe tests passed.")
