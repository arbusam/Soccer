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


# Outputs: direction, speed, rotation
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
def defence(x_pos, y_pos, yaw, ball_x, ball_y):
    vector = (ball_x - x_pos), (ball_y - y_pos)
    angle = math.degrees(math.atan2(vector[1], vector[0]))
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    print(angle, dist)
    offset = 0
    if dist < 200:
        if -10 < angle < 10:
            offset = 0
        elif 0 < angle < 180:
            offset = 80
        else:
            offset = -80
    return angle + offset, 500, 0


def goalie(x_pos, y_pos, yaw, ball_x, ball_y):
    return 0, 0 # Do nothing

if __name__ == "__main__":
    # TODO: Get the x_pos, y_pos, yaw, ball_x, ball_y from the sensors
    pass