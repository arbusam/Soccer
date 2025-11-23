import math

WHEEL_DIAMETER = 50 # mm

KP = 10
KI = 0.1
KD = 0.01

a_prev_error = 0
a_integral = 0
b_prev_error = 0
b_integral = 0
c_prev_error = 0
c_integral = 0
d_prev_error = 0
d_integral = 0

def move(direction, speed, a_speed, b_speed, c_speed, d_speed, dt): # degrees, mm/s
    direction -= 45
    a_mult = math.sin(math.radians(direction))
    b_mult = math.cos(math.radians(direction))
    c_mult = -math.sin(math.radians(direction))
    d_mult = -math.cos(math.radians(direction))

    # Values in mm/s
    a_value = int(a_mult * speed)
    b_value = int(b_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    # Values in rpm
    a_target = a_value / (WHEEL_DIAMETER * math.pi) * 60
    b_target = b_value / (WHEEL_DIAMETER * math.pi) * 60
    c_target = c_value / (WHEEL_DIAMETER * math.pi) * 60
    d_target = d_value / (WHEEL_DIAMETER * math.pi) * 60

    a_control, a_error, a_integral = pid_controller(a_target, a_speed, dt, a_prev_error, a_integral)
    b_control, b_error, b_integral = pid_controller(b_target, b_speed, dt, b_prev_error, b_integral)
    c_control, c_error, c_integral = pid_controller(c_target, c_speed, dt, c_prev_error, c_integral)
    d_control, d_error, d_integral = pid_controller(d_target, d_speed, dt, d_prev_error, d_integral)
    a_prev_error = a_error
    b_prev_error = b_error
    c_prev_error = c_error
    d_prev_error = d_error

    # TODO: Rotate A, B, C and D with previous duty cycle + control value

def pid_controller(target, current_speed, dt, prev_error, integral):
    error = target - current_speed
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0
    control = KP * error + KI * integral + KD * derivative
    return control, error, integral

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
# ball_caputed: True when the ball is touching the capture zone
def defence(x_pos, y_pos, yaw, ball_x, ball_y, yellow=True, ball_caputed=False):
    if yellow:
        vector = (ball_x - x_pos), (ball_y - y_pos)
    else:
        vector = (x_pos - ball_x), (y_pos - ball_y)
    angle = math.degrees(math.atan2(vector[1], vector[0]))
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    angle_to_goal_centre = math.degrees(math.atan2(910 - y_pos, 2200 - x_pos))
    rotation = 0
    speed = 500
    offset = 0
    if dist < 200:
        if -10 < angle < 10:
            speed = 700
            if ball_caputed:
                if y_pos < 750:
                    offset = 30
                elif y_pos > 1050:
                    offset = -30
        elif 0 < angle < 180:
            offset = 80
        else:
            offset = -80
    
    return angle + offset, speed, rotation


def goalie(x_pos, y_pos, yaw, ball_x, ball_y, yellow=True, ball_caputed=False):
    return 0, 500, yaw # Do nothing

if __name__ == "__main__":
    # TODO: Get the x_pos, y_pos, yaw, ball_x, ball_y from the sensors
    pass