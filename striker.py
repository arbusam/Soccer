def striker(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    ball_captured=False,
    steering_state=False,
):
    """Placeholder striker strategy; fill in later."""
    # Keep steering state stable until real striker logic is implemented.
    direction = 0
    speed = 0
    rotation = yaw
    steering = bool(steering_state)
    kick = False
    return direction, speed, rotation, steering, kick
