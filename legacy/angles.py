"""Angle helpers retained by the legacy Python movement controller."""


def wrap_angle_deg(angle):
    """Wrap an angle to the shortest signed equivalent in [-180, 180)."""
    return ((float(angle) + 180.0) % 360.0) - 180.0


def imu_yaw_to_relative_yaw(imu_yaw, startup_yaw):
    """Convert upside-down IMU yaw to clockwise-positive startup-relative yaw."""
    return wrap_angle_deg(float(startup_yaw) - float(imu_yaw))
