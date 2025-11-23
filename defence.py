import math

QUADRANT_FUNCS = [
lambda r: (r-1, 1, -1, 1-r),    # 0°‑89° N → E
lambda r: (1, 1-r, r-1, -1),    # 90°‑179° E → S
lambda r: (1-r, -1, 1, r-1),    # 180°‑270° S → W
lambda r: (-1, r-1, 1-r, 1),    # 270°‑359° W → N
]

def move(direction, speed): # degrees, mm/s
    octant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    a_mult, b_mult, c_mult, d_mult = QUADRANT_FUNCS[octant](ratio)
    a_value = int(a_mult * speed)
    b_value = int(b_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)
    pass

def pid_controller(target, pv, kp, ki, kd, prev_error, integral, dt):
    error = target - pv
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0
    control = kp * error + ki * integral + kd * derivative
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