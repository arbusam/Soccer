import math

# Coordinates of the centre of the goal zone, which the goalie uses for blocking.
CYAN_GOAL_CENTRE_X = 400
YELLOW_GOAL_CENTRE_X = 1980
# Y is shared between goals because it is the same
GOAL_CENTRE_Y = 910
# Coordinates of the back of the goal zone, which defence uses for aiming.
YELLOW_GOAL_BACK_X = 226
GOAL_BACK_Y_MIN = 700
GOAL_BACK_Y_MAX = 1125
CYAN_GOAL_BACK_X = 2204

# Pitch boundary coordinates. Used to keep bot within the pitch.
WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570

BALL_RADIUS = 21 # mm, radius of the ball

# Wrap to the shortest signed angle in [-180, 180).
def wrap_angle_deg(angle):
    return ((angle + 180) % 360) - 180


def point_to_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    line_len_sq = dx * dx + dy * dy
    if line_len_sq <= 1e-9:
        return math.hypot(px - x1, py - y1)

    t = ((px - x1) * dx + (py - y1) * dy) / line_len_sq
    t = max(0.0, min(1.0, t))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return math.hypot(px - closest_x, py - closest_y)

# Checks if the ball is outside the pitch by using the pitch boundaries, the ball position and the ball radius.
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
# ball_captured: True when the ball is touching the capture zone
# steering_state: caller-provided flag indicating if this bot is currently steering

# friendly_bot_positions: optional iterable of (x, y) positions for friendly robots
# enemy_bot_positions: optional iterable of (x, y) positions for enemy robots
# Outputs: direction, speed, rotation, steering, kick
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
# steering_state: Whether the bot is currently steering. Is not used elsewhere, only exists to persist state for the next call.
# kick: True if the bot should kick the ball
def defence(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    ball_captured=False,
    steering_state=False,
    friendly_bot_positions=None,
    enemy_bot_positions=None,
):
    dribbler = 0 # Whether the dribbler should be on.
    if friendly_bot_positions is None:
        friendly_bot_positions = []
    if enemy_bot_positions is None:
        enemy_bot_positions = []
    # If the ball is not detected, the bot should move to the centre of the pitch.
    if ball_x is None or ball_y is None:
        target_x = 1515
        target_y = 910
        vector = (target_x - x_pos), (target_y - y_pos)
        direction = math.degrees(math.atan2(vector[1], vector[0]))
        speed = 0
        rotation = 0
        steering = False
        kick = False
        return direction, speed, rotation, steering, kick, dribbler
    # Ensure the steering input is a boolean.
    steering = bool(steering_state)
    # Calculate the direction to the ball in vector form. Direction is relative to the bot's ideal heading (the direction towards the goal it should be scoring towards from the goal it is defending)
    vector = (ball_x - x_pos), (ball_y - y_pos)
    direction = math.degrees(math.atan2(vector[1], vector[0])) # Convert the vector to a direction in degrees, relative to the ideal heading.
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2) # Calculate the distance to the ball.
    rotation = 0 # Sets the desired rotation. 0 is always the startup/ideal heading in this frame.
    speed = 500 # mm/s, Default speed of the bot.
    offset = 0 # deg, Offset to the direction to the ball. Used to avoid own goals.
    # Only activate own goal prevention if the ball is close to the bot.
    if dist < 400:
        if -10 < direction < 10:
            speed = 1000
            if steering and y_pos < 850 and dist < 200:
                offset = 40
            elif steering and y_pos > 1050 and dist < 200:
                offset = -40
            if y_pos < 800 and ball_captured:
                offset = 20
                steering = True
            elif y_pos > 1000 and ball_captured:
                offset = -20
                steering = True
            else:
                steering = False
        elif 0 < direction < 180:
            offset = 80
        else:
            offset = -80
    elif dist > 500:
        speed = 600

    # By default, the bot should not kick the ball.
    kick = False

    # Only kick if the ball is captured and lined up with the goal.
    if ball_captured:
        dribbler = 1

        target_x = CYAN_GOAL_BACK_X
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
                    dribbler = False
                    kick = True
    if kick == True:
        dribbler = -1
    return direction + offset, speed, rotation, steering, kick, dribbler

# Inputs: 
# x_pos: x position of the robot
# y_pos: y position of the robot
# yaw: yaw value of the robot
# ball_x: x position of the ball
# ball_y: y position of the ball
# ball_captured: True when the ball is touching the capture zone
# friendly_bot_positions: optional iterable of (x, y) positions for friendly robots
# enemy_bot_positions: optional iterable of (x, y) positions for enemy robots
# Outputs: direction, speed, rotation
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
# kick: True if the bot wants to kick the ball
def goalie(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    ball_captured=False,
    friendly_bot_positions=None,
    enemy_bot_positions=None,
):
    dribbler = 0 # Whether the dribbler should be on.
    if friendly_bot_positions is None:
        friendly_bot_positions = []
    if enemy_bot_positions is None:
        enemy_bot_positions = []
    if ball_x is None or ball_y is None:
        target_x = 400
        target_y = GOAL_CENTRE_Y
        vector = (target_x - x_pos), (target_y - y_pos)
        direction = math.degrees(math.atan2(vector[1], vector[0]))
        speed = 600
        rotation = 0
        kick = False
        return direction, speed, rotation, kick, dribbler
    vector = (ball_x - x_pos), (ball_y - y_pos)
    direction = None
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
    angle_to_ball = math.degrees(math.atan2(ball_y - y_pos, ball_x - x_pos))
    rotation = 0
    angle_to_ball %= 360
    angle_error = ((angle_to_ball - yaw + 180) % 360) - 180
    speed = 700
    kick = False

    if ball_captured and -90 < yaw < 90:
        kick = True
        yaw_rad = math.radians(yaw)
        dir_x = math.cos(yaw_rad)
        dir_y = math.sin(yaw_rad)
        for bot in enemy_bot_positions:
            along_kick = (bot[0] - x_pos) * dir_x + (bot[1] - y_pos) * dir_y
            if abs(along_kick) < 200:
                kick = False
                    

    if y_pos > 1360:
        direction = 270
    elif y_pos < 460:
        direction = 90
    elif x_pos < 420:
        direction = 0
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
            goal_dx = YELLOW_GOAL_BACK_X - ball_x
            goal_dy = GOAL_CENTRE_Y - ball_y
            line_len_sq = goal_dx * goal_dx + goal_dy * goal_dy
            epsilon = 1e-6
            if line_len_sq < epsilon:
                intercept_x = CYAN_GOAL_CENTRE_X
                intercept_y = GOAL_CENTRE_Y
            else:
                t = ((x_pos - ball_x) * goal_dx + (y_pos - ball_y) * goal_dy) / line_len_sq
                intercept_x = ball_x + t * goal_dx
                intercept_y = ball_y + t * goal_dy

            if intercept_x < 430:
                intercept_x = 430
                if abs(goal_dx) > epsilon:
                    t = (intercept_x - ball_x) / goal_dx
                    intercept_y = ball_y + t * goal_dy

            dif_x = intercept_x - x_pos
            dif_y = intercept_y - y_pos
            if math.hypot(dif_x, dif_y) < 10:
                speed = 0
            direction = math.degrees(math.atan2(dif_y, dif_x))
    if kick == True:
        dribbler = -1

    return direction, speed, rotation, kick, dribbler
