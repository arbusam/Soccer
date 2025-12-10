import math
import time
import numpy as np
import asyncio

import send_log

WHEEL_DIAMETER = 50 # mm
CYAN_GOAL_CENTRE_X = 400
YELLOW_GOAL_CENTRE_X = 1980
GOAL_CENTRE_Y = 910
YELLOW_GOAL_BACK_X = 226
GOAL_BACK_Y_MIN = 700
GOAL_BACK_Y_MAX = 1125
CYAN_GOAL_BACK_X = 2204
YAW_CORRECT_SPEED = 500 # deg/s
LIDAR_PORT = "/dev/ttyUSB1"
LIDAR_BAUDRATE = 460800

# White boundary rectangle taken from simulate.py
WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570
BALL_RADIUS = 21

def is_ball_out(ball_x, ball_y):
    closest_x = max(WHITE_MIN_X, min(ball_x, WHITE_MAX_X))
    closest_y = max(WHITE_MIN_Y, min(ball_y, WHITE_MAX_Y))
    dx = ball_x - closest_x
    dy = ball_y - closest_y
    distance = math.hypot(dx, dy)
    return distance > BALL_RADIUS

# Inputs: 
# x_pos: x position of the robot
# y_pos: y position of the robot
# yaw: yaw value of the robot
# ball_x: x position of the ball
# ball_y: y position of the ball
# yellow: True if the bot is scoring towards yellow, False if the bot is scoring towards cyan
# ball_captured: True when the ball is touching the capture zone
# steering_state: caller-provided flag indicating if this bot is currently steering
# Outputs: direction, speed, rotation
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
# steering_state: caller-provided flag indicating if this bot is currently steering
# kick: True if the bot wants to kick the ball
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
    if ball_x is None or ball_y is None:
        target_x = 1215
        target_y = 910
        vector = (target_x - x_pos), (target_y - y_pos)
        direction = math.degrees(math.atan2(vector[1], vector[0]))
        speed = 600
        rotation = 0 if yellow else 180
        steering = False
        kick = False
        return direction, speed, rotation, steering, kick
    steering = bool(steering_state)
    if yellow:
        vector = (ball_x - x_pos), (ball_y - y_pos)
    else:
        vector = (x_pos - ball_x), (y_pos - ball_y)
    direction = math.degrees(math.atan2(vector[1], vector[0]))
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    rotation = 0 if yellow else 180
    speed = 500
    offset = 0
    if dist < 200:
        if -10 < direction < 10:
            speed = 1000
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
        elif 0 < direction < 180:
            offset = 80
        else:
            offset = -80
    elif dist > 500:
        speed = 1000

    if not yellow:
        direction -= 180
        direction %= 360

    kick = False
    if ball_captured:
        if yellow:
            target_x = CYAN_GOAL_BACK_X
            target_y_min = GOAL_BACK_Y_MIN
            target_y_max = GOAL_BACK_Y_MAX
        else:
            target_x = YELLOW_GOAL_BACK_X
            target_y_min = GOAL_BACK_Y_MIN
            target_y_max = GOAL_BACK_Y_MAX

        yaw_rad = math.radians(yaw % 360)
        dir_x = math.cos(yaw_rad)
        dir_y = math.sin(yaw_rad)
        epsilon = 1e-6

        if abs(dir_x) > epsilon:
            t = (target_x - ball_x) / dir_x
            if t >= 0:
                y_hit = ball_y + t * dir_y
                if target_y_min <= y_hit <= target_y_max:
                    kick = True

    return direction + offset, speed, rotation, steering, kick

# Inputs: 
# x_pos: x position of the robot
# y_pos: y position of the robot
# yaw: yaw value of the robot
# ball_x: x position of the ball
# ball_y: y position of the ball
# yellow: True if the bot is scoring towards yellow, False if the bot is scoring towards cyan
# ball_captured: True when the ball is touching the capture zone
# Outputs: direction, speed, rotation
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
# kick: True if the bot wants to kick the ball
def goalie(x_pos, y_pos, yaw, ball_x, ball_y, yellow=True, ball_captured=False):
    if ball_x is None or ball_y is None:
        target_x = YELLOW_GOAL_CENTRE_X if yellow else CYAN_GOAL_CENTRE_X
        target_y = GOAL_CENTRE_Y
        vector = (target_x - x_pos), (target_y - y_pos)
        direction = math.degrees(math.atan2(vector[1], vector[0]))
        speed = 600
        rotation = 0 if yellow else 180
        steering = False
        kick = False
        return direction, speed, rotation, steering, kick
    if yellow:
        vector = (ball_x - x_pos), (ball_y - y_pos)
    else:
        vector = (x_pos - ball_x), (y_pos - ball_y)
    direction = None
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    angle_to_ball = math.degrees(math.atan2(ball_y - y_pos, ball_x - x_pos))
    rotation = angle_to_ball
    angle_to_ball %= 360
    angle_error = ((angle_to_ball - yaw + 180) % 360) - 180
    speed = 700
    if yellow:
        if y_pos > 1360:
            direction = 270
        elif y_pos < 460:
            direction = 90
        elif x_pos < 420:
            direction = 0
        elif x_pos < 300:
            if y_pos < 910:
                if y_pos < 560:
                    direction = 0
                else:
                    direction = 270
            else:
                if y_pos > 1260:
                    direction = 0
                else:
                    direction = 90
        elif x_pos > 600 and not ball_captured:
            direction = 180
        else:
            if is_ball_out(ball_x, ball_y):
                if abs(y_pos - 910) > 5:
                    if y_pos < 910:
                        direction = 90
                    else:
                        direction = 270
                else:
                    direction = yaw
                    speed = 0
            elif dist < 500 and abs(angle_error) < 10:
                direction = yaw
            else:
                if ball_y == 910 or ball_x == 226:
                    dif_x = CYAN_GOAL_CENTRE_X - x_pos
                    dif_y = GOAL_CENTRE_Y - y_pos
                    direction = math.degrees(math.atan2(dif_y, dif_x))
                else:
                    ball_gradient = (GOAL_CENTRE_Y - ball_y) / (226 - ball_x)
                    ball_line_c = ball_y - (ball_gradient * ball_x)
                    bot_gradient = -1 / ball_gradient
                    bot_line_c = y_pos - (bot_gradient * x_pos)
                    intercept_x = (bot_line_c - ball_line_c) / (ball_gradient - bot_gradient)
                    intercept_y = (bot_gradient * intercept_x) + bot_line_c

                    if intercept_x < 430:
                        intercept_x = 430
                        intercept_y = ball_gradient * intercept_x + ball_line_c
                    if (intercept_y < 910 and intercept_y > 460) or ball_y < 250:
                        direction = 270
                    elif (intercept_y > 910 and intercept_y < 1360) or ball_y > 1570:
                        direction = 90

                    dif_x = intercept_x - x_pos
                    dif_y = intercept_y - y_pos
                    if math.hypot(dif_x, dif_y) < 10:
                        speed = 0
                    direction = math.degrees(math.atan2(dif_y, dif_x))
    else:
        if y_pos > 1360:
            direction = 270
        elif y_pos < 460:
            direction = 90
        elif x_pos > 1980:
            direction = 180
        elif x_pos > 2130:
            if y_pos < 910:
                if y_pos < 560:
                    direction = 180
                else:
                    direction = 270
            else:
                if y_pos > 1260:
                    direction = 180
                else:
                    direction = 90
        elif x_pos < 1830 and not ball_captured:
            direction = 0
        else:
            if is_ball_out(ball_x, ball_y):
                if abs(y_pos - 910) > 5:
                    if y_pos < 910:
                        direction = 90
                    else:
                        direction = 270
                else:
                    direction = yaw
                    speed = 0
            elif dist < 500 and abs(angle_error) < 10:
                direction = yaw
            else:
                if ball_y == 910 or ball_x == 2204:
                    dif_x = YELLOW_GOAL_CENTRE_X - x_pos
                    dif_y = GOAL_CENTRE_Y - y_pos
                    direction = math.degrees(math.atan2(dif_y, dif_x))
                else:
                    ball_gradient = (GOAL_CENTRE_Y - ball_y) / (2204 - ball_x)
                    ball_line_c = ball_y - (ball_gradient * ball_x)
                    bot_gradient = -1 / ball_gradient
                    bot_line_c = y_pos - (bot_gradient * x_pos)
                    intercept_x = (bot_line_c - ball_line_c) / (ball_gradient - bot_gradient)
                    intercept_y = (bot_gradient * intercept_x) + bot_line_c

                    if intercept_x > 2000:
                        intercept_x = 2000
                        intercept_y = ball_gradient * intercept_x + ball_line_c
                    if intercept_y < 910 and intercept_y > 460:
                        direction = 90
                    elif intercept_y > 910 and intercept_y < 1360:
                        direction = 270

                    dif_x = intercept_x - x_pos
                    dif_y = intercept_y - y_pos
                    if math.hypot(dif_x, dif_y) < 10:
                        speed = 0
                    direction = math.degrees(math.atan2(dif_y, dif_x))

    return direction, speed, rotation, False

def get_mean_distance(values):
    if not values:
        return None
    if len(values) < 4:
        return np.mean(values)
    q75 = np.percentile(values, 75)
    q25 = np.percentile(values, 25)
    iqr = q75 - q25
    upper_fence = q75 + 1.5 * iqr
    lower_fence = q25 - 1.5 * iqr
    filtered = [x for x in values if lower_fence <= x <= upper_fence]
    if not filtered:
        return None
    return np.mean(filtered)


def get_coordinates(yaw):
    import lidar
    yaw = float(yaw)

    # Grab the full scan once (far cheaper than hundreds of per-angle calls)
    scan = lidar.get_scan_numpy()
    if scan.size == 0:
        return None, None

    distances = scan[:, 1]
    angles = scan[:, 0]
    valid_mask = distances > 0
    if not np.any(valid_mask):
        return None, None

    distances = distances[valid_mask]
    angles = angles[valid_mask]

    # Convert to signed angles then shift by yaw to align with world frame
    angles_signed = ((angles + 180) % 360) - 180  # [-180, 180)
    world_angles = ((angles_signed + yaw + 180) % 360) - 180
    world_radians = np.radians(world_angles)

    def project_and_mean(mask, projection_values):
        if not np.any(mask):
            return None
        return get_mean_distance(projection_values[mask].tolist())

    # Right wall (world -45°..45°)
    right_mask = (world_angles >= -45) & (world_angles <= 45)
    right_proj = distances * np.cos(world_radians)
    top_x_distance = project_and_mean(right_mask, right_proj)

    # Left wall (world 135°..180° and -180°..-135°)
    left_mask = (world_angles >= 135) | (world_angles <= -135)
    left_proj = distances * -np.cos(world_radians)
    bottom_x_distance = project_and_mean(left_mask, left_proj)

    # Top wall (world -135°..-45°)
    top_mask = (world_angles >= -135) & (world_angles <= -45)
    top_proj = distances * -np.sin(world_radians)
    left_y_distance = project_and_mean(top_mask, top_proj)

    # Bottom wall (world 45°..135°)
    bottom_mask = (world_angles >= 45) & (world_angles <= 135)
    bottom_proj = distances * np.sin(world_radians)
    right_y_distance = project_and_mean(bottom_mask, bottom_proj)

    top_x_valid = top_x_distance is not None and 0 <= top_x_distance <= 2430
    bottom_x_valid = bottom_x_distance is not None and 0 <= bottom_x_distance <= 2430

    if top_x_valid and bottom_x_valid:
        x_pos = (bottom_x_distance + (2430 - top_x_distance)) / 2
    elif bottom_x_valid:
        x_pos = bottom_x_distance
    elif top_x_valid:
        x_pos = 2430 - top_x_distance
    else:
        x_pos = None

    left_y_valid = left_y_distance is not None and 0 <= left_y_distance <= 1820
    right_y_valid = right_y_distance is not None and 0 <= right_y_distance <= 1820

    if left_y_valid and right_y_valid:
        y_pos = (left_y_distance + (1820 - right_y_distance)) / 2
    elif left_y_valid:
        y_pos = left_y_distance
    elif right_y_valid:
        y_pos = 1820 - right_y_distance
    else:
        y_pos = None

    return x_pos, y_pos

if __name__ == "__main__":
    import lidar
    from movement import init_motors, move

    server_task = asyncio.create_task(send_log.init_server())
    time.sleep(0.05)

    print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
    try:
        lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)
    except Exception as e:
        raise RuntimeError(f"Failed to initialize LIDAR: {e}")
    
    print("LIDAR initialized successfully!")
    print()
    
    print("Waiting for first scan data...")
    while not lidar.is_scan_ready():
        time.sleep(0.1)

    motors, motor_modes = init_motors()
    steering_state = False
    while True:
        # TODO: Get the x_pos, y_pos, yaw, ball_x, ball_y from the sensors asynchronously
        yaw = 0
        x_pos, y_pos = get_coordinates(yaw)
        ball_x = None
        ball_y = None
        yellow = True
        ball_captured = False
        send_log.update_latest_log(f"{x_pos},{y_pos},{yaw},{ball_x},{ball_y}")
        direction, speed, rotation, steering_state, _ = defence(
            x_pos,
            y_pos,
            yaw,
            ball_x,
            ball_y,
            yellow,
            ball_captured,
            steering_state=steering_state,
        )
        move(direction, speed, rotation, yaw, motors, motor_modes, WHEEL_DIAMETER, YAW_CORRECT_SPEED)

