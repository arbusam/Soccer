import math

from defence import (
    BOUNDARY_STOP_MARGIN,
    ROBOT_RADIUS,
    WHITE_MAX_X,
    WHITE_MIN_X,
    defence,
    goalie,
    keep_motion_inside_white_lines,
)


def boundary_guard_stops_outward_motion():
    x_pos = WHITE_MIN_X + ROBOT_RADIUS + BOUNDARY_STOP_MARGIN

    _, speed = keep_motion_inside_white_lines(x_pos, 910, 180, 600)

    assert speed == 0


def boundary_guard_preserves_tangential_motion():
    x_pos = WHITE_MIN_X + ROBOT_RADIUS

    direction, speed = keep_motion_inside_white_lines(x_pos, 910, 135, 600)

    assert math.isclose(direction, 90)
    assert math.isclose(speed, 600 / math.sqrt(2))


def boundary_defence_does_not_chase_ball_over_line():
    x_pos = WHITE_MAX_X - ROBOT_RADIUS

    _, speed, *_ = defence(x_pos, 910, 0, WHITE_MAX_X + 200, 910)

    assert speed == 0


def boundary_goalie_does_not_chase_ball_over_line():
    x_pos = WHITE_MIN_X + ROBOT_RADIUS

    direction, speed, *_ = goalie(x_pos, 910, 0, WHITE_MIN_X - 200, 910)

    assert speed * math.cos(math.radians(direction)) >= 0
