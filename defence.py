import math
import time
import numpy as np

WHEEL_DIAMETER = 50 # mm
CYAN_GOAL_CENTRE_X = 400
CYAN_GOAL_CENTRE_Y = 910
YELLOW_GOAL_CENTRE_X = 1980
YELLOW_GOAL_CENTRE_Y = 910
YAW_CORRECT_SPEED = 500 # deg/s
LIDAR_PORT = "/dev/ttyUSB1"
LIDAR_BAUDRATE = 460800

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

    if not yellow:
        angle -= 180
        angle %= 360
    
    return angle + offset, speed, rotation, steering

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
def goalie(x_pos, y_pos, yaw, ball_x, ball_y, yellow=True, ball_captured=False):
    if yellow:
        vector = (ball_x - x_pos), (ball_y - y_pos)
    else:
        vector = (x_pos - ball_x), (y_pos - ball_y)
    # angle = math.degrees(math.atan2(vector[1], vector[0]))
    angle = None
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    angle_to_ball = math.degrees(math.atan2(ball_y - y_pos, ball_x - x_pos))
    rotation = angle_to_ball
    angle_to_ball %= 360
    speed = 700
    if yellow:
        if y_pos > 1360:
            angle = 270
        elif y_pos < 460:
            angle = 90
        elif x_pos < 300:
            angle = 90
        elif x_pos > 800 and not ball_captured:
            angle = 180
        else:
            if dist < 500 and -10 < angle_to_ball - yaw < 10:
                angle = yaw
            else:
                if ball_y == 910 or ball_x == 226:
                    dif_x = CYAN_GOAL_CENTRE_X - x_pos
                    dif_y = CYAN_GOAL_CENTRE_Y - y_pos
                    angle = math.degrees(math.atan2(dif_y, dif_x))
                else:
                    ball_gradient = (CYAN_GOAL_CENTRE_Y - ball_y) / (226 - ball_x)
                    ball_line_c = ball_y - (ball_gradient * ball_x)
                    bot_gradient = -1 / ball_gradient
                    bot_line_c = y_pos - (bot_gradient * x_pos)
                    intercept_x = (bot_line_c - ball_line_c) / (ball_gradient - bot_gradient)
                    intercept_y = (bot_gradient * intercept_x) + bot_line_c

                    if intercept_x < 430:
                        intercept_x = 430
                        intercept_y = ball_gradient * intercept_x + ball_line_c
                    if (intercept_y < 910 and intercept_y > 460) or ball_y < 250:
                        angle = 270
                    elif (intercept_y > 910 and intercept_y < 1360) or ball_y > 1570:
                        angle = 90

                    dif_x = intercept_x - x_pos
                    dif_y = intercept_y - y_pos
                    angle = math.degrees(math.atan2(dif_y, dif_x))
    else:
        if y_pos > 1360:
            angle = 270
        elif y_pos < 460:
            angle = 90
        elif x_pos > 2130:
            angle = 90
        elif x_pos < 1630 and not ball_captured:
            angle = 0
        else:
            if dist < 500 and -10 < angle_to_ball - yaw < 10:
                angle = yaw
            else:
                if ball_y == 910 or ball_x == 2204:
                    dif_x = YELLOW_GOAL_CENTRE_X - x_pos
                    dif_y = YELLOW_GOAL_CENTRE_Y - y_pos
                    angle = math.degrees(math.atan2(dif_y, dif_x))
                else:
                    ball_gradient = (YELLOW_GOAL_CENTRE_Y - ball_y) / (2204 - ball_x)
                    ball_line_c = ball_y - (ball_gradient * ball_x)
                    bot_gradient = -1 / ball_gradient
                    bot_line_c = y_pos - (bot_gradient * x_pos)
                    intercept_x = (bot_line_c - ball_line_c) / (ball_gradient - bot_gradient)
                    intercept_y = (bot_gradient * intercept_x) + bot_line_c

                    if intercept_x > 2000:
                        intercept_x = 2000
                        intercept_y = ball_gradient * intercept_x + ball_line_c
                    if intercept_y < 910 and intercept_y > 460:
                        angle = 90
                    elif intercept_y > 910 and intercept_y < 1360:
                        angle = 270

                    dif_x = intercept_x - x_pos
                    dif_y = intercept_y - y_pos
                    angle = math.degrees(math.atan2(dif_y, dif_x))

    return angle, speed, rotation

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
    from movement import init_motors, move

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
        move(direction, speed, rotation, yaw, motors, motor_modes, WHEEL_DIAMETER, YAW_CORRECT_SPEED)

