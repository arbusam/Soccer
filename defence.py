import math
def move(direction, speed): # degrees, mm/s
    # TODO: Move the robot in the given direction at the given speed
    pass

# Outputs: direction, speed, rotation
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
def defence(x_pos, y_pos, yaw, ball_x, ball_y):
    vector = (ball_x-x_pos), (ball_y-y_pos)
    angle = math.degrees(math.atan2(vector[1], vector[0]))
    dist = math.sqrt(vector[0]**2 + vector[1]**2)
    print(angle, dist)
    offset = 0
    if dist < 200:
        if angle < 10 and angle > -10:
            offset = 0
        elif angle < 180 and angle > 0:
            offset = 80
        else:
            offset = -80
    return angle + offset, 500, 0


def goalie(x_pos, y_pos, yaw, ball_x, ball_y):
    return 0, 0 # Do nothing

if __name__ == "__main__":
    # TODO: Get the x_pos, y_pos, yaw, ball_x, ball_y from the sensors
    pass