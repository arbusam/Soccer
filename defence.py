import math
from movement import Motor, move

WHEEL_DIAMETER = 50 # mm

KP = 10
KI = 0.1
KD = 0.01

motor_a = Motor()
motor_b = Motor()
motor_c = Motor()
motor_d = Motor()

# Inputs: 
# x_pos: x position of the robot
# y_pos: y position of the robot
# yaw: yaw value of the robot
# ball_x: x position of the ball
# ball_y: y position of the ball
# yellow: True if the bot is scoring towards yellow, False if the bot is scoring towards cyan
# Outputs: direction, speed, rotation
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
# ball_captured: True when the ball is touching the capture zone
def defence(x_pos, y_pos, yaw, ball_x, ball_y, yellow=True, ball_captured=False):
    if yellow:
        vector = (ball_x - x_pos), (ball_y - y_pos)
    else:
        vector = (x_pos - ball_x), (y_pos - ball_y)
    angle = math.degrees(math.atan2(vector[1], vector[0]))
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    angle_to_goal_centre = math.degrees(math.atan2(910 - y_pos, 2200 - x_pos))
    rotation = 0 if yellow else 180
    speed = 500
    offset = 0
    if dist < 200:
        if -10 < angle < 10:
            speed = 700
            if ball_captured:
                if y_pos < 750:
                    offset = 30
                elif y_pos > 1050:
                    offset = -30
        elif 0 < angle < 180:
            offset = 80
        else:
            offset = -80
    
    return angle + offset, speed, rotation


def goalie(x_pos, y_pos, yaw, ball_x, ball_y, yellow=True, ball_captured=False):
    return 0, 500, yaw # Do nothing

if __name__ == "__main__":
    while True:
        # TODO: Get actual time delta
        dt = 0.01
        # TODO: Get the x_pos, y_pos, yaw, ball_x, ball_y from the sensors asynchronously
        x_pos = 0
        y_pos = 0
        yaw = 0
        ball_x = 0
        ball_y = 0
        yellow = True
        ball_captured = False
        direction, speed, rotation = defence(x_pos, y_pos, yaw, ball_x, ball_y, yellow, ball_captured)
        move(direction, speed, rotation, [motor_a, motor_b, motor_c, motor_d], dt, WHEEL_DIAMETER, KP, KI, KD)