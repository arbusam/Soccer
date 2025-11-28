import math
from movement import Motor, run

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
# steering_state: caller-provided flag indicating if this bot is currently steering
def defence(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    yellow=True,
    ball_captured=False,
    steering_state=False,
):
    steering = bool(steering_state)
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
            if steering and y_pos < 850 and dist < 200:
                offset = 30
            elif steering and y_pos > 1050 and dist < 200:
                offset = -30
            if y_pos < 800 and ball_captured:
                offset = 30
                steering = True
            elif y_pos > 1000 and ball_captured:
                offset = -30
                steering = True
            else:
                steering = False
                
            if not yellow:
                offset = -offset
        elif 0 < angle < 180:
            offset = 80
        else:
            offset = -80
    
    return angle + offset, speed, rotation, steering


def goalie(x_pos, y_pos, yaw, ball_x, ball_y, colour, yellow=True, ball_captured=False):

    if yellow:
        vector = (ball_x - x_pos), (ball_y - y_pos)
    else:
        vector = (x_pos - ball_x), (y_pos - ball_y)
    angle = math.degrees(math.atan2(vector[1], vector[0]))
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    rotation = 0 if yellow else 180
    if dist > 400:
        if not (1850 < x_pos < 1950  and 850 < y_pos < 900) or (450 < x_pos < 550 and 850 < y_pos < 900):
            if yellow:
                vector = (500 - x_pos), (900 - y_pos)
            else:
                vector = (x_pos - 1900), (y_pos - 900)
            angle = math.degrees(math.atan2(vector[1], vector[0]))
            speed = 400
        else:
            speed = 0
    else:
        if colour != (0, 0, 0):
            speed = 500
        else:
            if -10 < angle < 10:
                speed = 0
            elif angle > 0:
                angle = 90
                speed = 500
            elif angle < 0:
                angle = -90
                speed = 500
    return angle, speed, rotation # Do nothing

if __name__ == "__main__":
    steering_state = False
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
        direction, speed, rotation, steering_state = defence(
            x_pos,
            y_pos,
            yaw,
            ball_x,
            ball_y,
            yellow,
            ball_captured,
            steering_state=steering_state,
        )
        run(direction, speed, rotation, [motor_a, motor_b, motor_c, motor_d], dt, WHEEL_DIAMETER, KP, KI, KD)
