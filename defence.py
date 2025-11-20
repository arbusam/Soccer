def move(direction, speed): # degrees, mm/s
    # TODO: Move the robot in the given direction at the given speed
    pass

def defence(x_pos, y_pos, yaw, ball_x, ball_y):
    if x_pos - ball_x < 30 and x_pos - ball_x > -30:
        if y_pos - ball_y < 30 and y_pos - ball_y > -30:
            return 0, 0
        return 90, 500
    else:
        return -60, 500


def goalie(x_pos, y_pos, yaw, ball_x, ball_y):
    return 0, 0 # Do nothing

if __name__ == "__main__":
    # TODO: Get the x_pos, y_pos, yaw, ball_x, ball_y from the sensors
    pass