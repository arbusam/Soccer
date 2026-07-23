import math

WHEEL_DIAMETER = 50 # mm, used to convert mm/s to RPM
MAX_YAW_RPM = 100 # Maximum rpm that can be added or subtracted from the wheel speeds to correct yaw
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
MAX_MOTOR_RPM = 400 # Maximum rpm that the wheels can spin at
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
#TOF_ADDRESS = 0x50
BALL_CAPTURED_DISTANCE = 27 # mm, distance from the ToF to the ball to consider it captured

# Pitch boundary coordinates. Used to keep bot within the pitch.
WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570

# Short pitch axis (Y): used to pick which sideline to drive toward.
PITCH_WIDTH = 1820
# How close to the white sideline before switching from wall-drive to upfield.
BALL_HIDING_LINE_THRESHOLD = 150

BALL_RADIUS = 21 # mm, radius of the ball

YAW_CORRECT_THRESHOLD = 3 # deg, threshold of allowable yaw error.
OWN_GOAL_PREVENTION_OFFSET = 10 # deg, how much to offset the direction to the ball when preventing own goals.

CAMERA_PORT = 8000
I2C_ADDRESSES = [29, 27, 26, 25]

BALL_TIMEOUT = 1 # seconds, time to extrapolate the ball position from velocity without assuming 'lost' state.


def wrap_angle_deg(angle):
    return ((angle + 180) % 360) - 180


def striker(
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
    """Placeholder striker strategy; fill in later."""
    if friendly_bot_positions is None:
        friendly_bot_positions = []
    if enemy_bot_positions is None:
        enemy_bot_positions = []
    # Keep steering state stable until real striker logic is implemented.
    # Ensure the steering input is a boolean.
    steering = bool(steering_state)
    # Calculate the direction to the ball in vector form. Direction is relative to the bot's ideal heading (the direction towards the goal it should be scoring towards from the goal it is defending)
    vector = (ball_x - x_pos), (ball_y - y_pos)
    direction = math.degrees(math.atan2(vector[1], vector[0])) # Convert the vector to a direction in degrees, relative to the ideal heading.
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2) # Calculate the distance to the ball.
    rotation = 0 # Sets the desired rotation. 0 is always the startup/ideal heading in this frame.
    speed = 400 # mm/s, Default speed of the bot.
    offset = 0 # deg, Offset to the direction to the ball. Used to avoid own goals.
    # Skip approach offset while captured: ball is in front, so direction ≈ yaw and
    # ±80 would clear goal rotation and oscillate against facing forward (rotation=0).
    if not ball_captured and dist < 300:
        if -10 < direction < 10:
            speed = 1000
        elif 0 < direction < 180:
            offset = 80
        else:
            offset = -80
    elif dist > 500:
        speed = 800

    # By default, the bot should not kick the ball.
    kick = False
    dribbler = True # Whether the dribbler should be on.

    # Only kick if the ball is captured and lined up with the goal.
    # Kicks ball into goal when it's close to the goal
    if ball_captured:
        target_x = CYAN_GOAL_BACK_X

        dist_to_goal = math.sqrt((target_x - ball_x) ** 2 + (GOAL_CENTRE_Y - ball_y) ** 2)
        degrees_to_goal = math.degrees(math.atan2(GOAL_CENTRE_Y - ball_y, target_x - ball_x))
        for enemy_bot in enemy_bot_positions:
            degrees_to_enemy = math.degrees(
                math.atan2(enemy_bot[1] - ball_y, enemy_bot[0] - ball_x)
            )
            if abs(wrap_angle_deg(degrees_to_enemy - degrees_to_goal)) < 50:
                if 0 < degrees_to_goal < 90:
                    offset = 80
                elif -90 < degrees_to_goal < 0:
                    offset = -80
        if offset == 0 and dist_to_goal < 850:
            # Stop and turn in place toward the goal, then shoot when lined up.
            speed = 0
            rotation = degrees_to_goal
            if abs(wrap_angle_deg(yaw - degrees_to_goal)) < 10:
                kick = True
        elif dist_to_goal >= 850:
            # Drive perpendicular to the goals toward a sideline until close enough to
            # aim. Keep dribbler on so losing the ball does not drop rotation to 0.
            offset = 0
            speed = 400
            if y_pos > PITCH_WIDTH / 2:
                rotation = 90
                near_line = y_pos >= WHITE_MAX_Y - BALL_HIDING_LINE_THRESHOLD
            else:
                rotation = 270
                near_line = y_pos <= WHITE_MIN_Y + BALL_HIDING_LINE_THRESHOLD
            # Once tucked against the sideline, advance upfield while still facing the wall.
            direction = 0 if near_line else rotation

    return direction + offset, speed, rotation, steering, kick, dribbler
