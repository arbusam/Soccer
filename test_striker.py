import math
import time
import argparse
import select
import sys

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
TOF_ADDRESS = 0x50
BALL_CAPTURED_DISTANCE = 27 # mm, distance from the ToF to the ball to consider it captured

# Pitch boundary coordinates. Used to keep bot within the pitch.
WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570

BALL_RADIUS = 21 # mm, radius of the ball

YAW_CORRECT_THRESHOLD = 3 # deg, threshold of allowable yaw error.
OWN_GOAL_PREVENTION_OFFSET = 10 # deg, how much to offset the direction to the ball when preventing own goals.

CAMERA_PORT = 8000
I2C_ADDRESSES = [28, 32, 31, 30]

BALL_TIMEOUT = 1 # seconds, time to extrapolate the ball position from velocity without assuming 'lost' state.
def capture_startup_yaw(imu, sample_count=25, sample_interval=0.02):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw = imu.get_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))

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
    if friendly_bot_positions is None:
        friendly_bot_positions = []
    if enemy_bot_positions is None:
        enemy_bot_positions = []
    # If the ball is not detected, the bot should move to the centre of the pitch.
    if ball_x is None or ball_y is None:
        target_x = 1215
        target_y = 910
        vector = (target_x - x_pos), (target_y - y_pos)
        direction = math.degrees(math.atan2(vector[1], vector[0]))
        speed = 0
        rotation = 0
        steering = False
        kick = False
        return direction, speed, rotation, steering, kick
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
    if dist < 300:
        if -10 < direction < 10:
            speed = 1000
            if steering and y_pos < 850 and dist < 200:
                offset = OWN_GOAL_PREVENTION_OFFSET
            elif steering and y_pos > 1050 and dist < 200:
                offset = -OWN_GOAL_PREVENTION_OFFSET
            if y_pos < 800 and ball_captured:
                offset = OWN_GOAL_PREVENTION_OFFSET
                steering = True
            elif y_pos > 1000 and ball_captured:
                offset = -OWN_GOAL_PREVENTION_OFFSET
                steering = True
            else:
                steering = False
        elif 0 < direction < 180:
            offset = 80
        else:
            offset = -80
    elif dist > 500:
        speed = 800

    # By default, the bot should not kick the ball.
    kick = False

    # Only kick if the ball is captured and lined up with the goal.
    if ball_captured:
        target_x = CYAN_GOAL_BACK_X
        target_y_min = GOAL_BACK_Y_MIN
        target_y_max = GOAL_BACK_Y_MAX

        dist_to_goal = math.sqrt((target_x - ball_x) ** 2 + (GOAL_CENTRE_Y - ball_y) ** 2)
        degrees_to_goal = math.degrees(math.atan2(GOAL_CENTRE_Y - ball_y, target_x - ball_x))
        for enemy_bot in enemy_bot_positions:
            enemy_bot_vector = (enemy_bot[0] - ball_x, enemy_bot[1] - ball_y)
            degrees_to_enemy_bots = math.degrees(math.atan2(enemy_bot[1] - ball_y, enemy_bot[0] - ball_x))
            if degrees_to_goal + 50 < degrees_to_enemy_bots < degrees_to_goal - 50:
                if 0 < degrees_to_goal < 90:
                    offset = 80
                elif -90 < degrees_to_goal < 0:
                    offset = -80

        yaw_rad = math.radians(yaw % 360)
        dir_x = math.cos(yaw_rad)
        dir_y = math.sin(yaw_rad)
        epsilon = 1e-6

        if abs(dir_x) > epsilon and dist_to_goal < 500:
            t = (target_x - ball_x) / dir_x
            if t >= 0:
                y_hit = ball_y + t * dir_y
                if target_y_min <= y_hit <= target_y_max:
                    kick = True

    return direction + offset, speed, rotation, steering, kick


def _prompt_i2c_addresses():
    print("Please enter the number of motor drivers you want to control:")
    tempuint32 = int(input())
    if tempuint32 == 0 or tempuint32 > 8:
        print("Error motor count out of range, please reboot microcontroller to try again.")
        quit()

    addresses = []
    setup_motor_count = 0
    while setup_motor_count < tempuint32:
        print(f"Please enter the i2c address of motor driver number {setup_motor_count}:")
        address = int(input())
        if address <= 7 or address >= 120:
            print("Error invalid i2c address, please reboot microcontroller to try again.")
            quit()
        addresses.append(address)
        setup_motor_count += 1
    return addresses


def _enter_pressed():
    if not sys.stdin.isatty():
        return False

    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False

    sys.stdin.readline()
    return True

if __name__ == "__main__":
    import board
    import lidar
    import switch

    switch2 = switch.Switch(board.D21)

    run = False

    from movement import (
        MotorCommunicationError,
        MovementController,
        imu_yaw_to_relative_yaw,
    )
    from camera import Camera
    from imu import IMU
    from kicker import Kicker
    from tof import ToF

    parser = argparse.ArgumentParser(
        description="Run defence controller with optional live websocket streaming."
    )
    parser.add_argument(
        "-s",
        "--stream",
        action="store_true",
        help="Enable websocket live log streaming for simulate.py --connect.",
    )
    args = parser.parse_args()

    if args.stream:
        import send_log
        # Start websocket log server in the background (it runs its own asyncio loop).
        send_log.start_server_background()
        time.sleep(0.05)

    camera = None
    imu = None
    kicker = None
    movement_controller = None
    try:
        kicker = Kicker(board.D26, 0.1)
        tof = ToF(address=TOF_ADDRESS)
        print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
        try:
            lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize LIDAR: {e}")

        print("LIDAR initialized successfully!")
        print()

        print("Waiting for first scan data...")
        while not lidar.is_scan_ready():
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                raise KeyboardInterrupt
            time.sleep(0.1)

        camera = Camera(CAMERA_PORT, resolution=(2000, 2000), frame_rate=90)
        camera.start_stream()

        imu = IMU()
        startup_yaw = capture_startup_yaw(imu)
        print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
        yaw_world = imu.get_yaw()
        if yaw_world is not None:
            lidar.set_imu_yaw(imu_yaw_to_relative_yaw(yaw_world, startup_yaw))

        lidar.start_coordinates(2430, 1820)

        print("Waiting for first coordinate estimate...")
        while not lidar.is_coordinates_ready():
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                raise KeyboardInterrupt
            yaw_world = imu.get_yaw()
            if yaw_world is not None:
                lidar.set_imu_yaw(imu_yaw_to_relative_yaw(yaw_world, startup_yaw))
            time.sleep(0.1)

        print(f"Initializing motors at I2C addresses: {I2C_ADDRESSES}")
        movement_controller = MovementController.from_i2c_addresses(
            I2C_ADDRESSES,
            WHEEL_DIAMETER,
            MAX_YAW_RPM,
            MAX_MOTOR_RPM,
            YAW_CORRECT_THRESHOLD,
        )
        print("Press Enter to shut down.")
        steering_state = False

        ball_dx = 0
        ball_dy = 0
        last_ball_update = time.time()
        last_ball_x = None
        last_ball_y = None
        last_camera_frame_id = camera.frame_id

        while True:
            if switch2.read():
                run = not run
                if run:
                    startup_yaw = capture_startup_yaw(imu)
                while switch2.read():
                    time.sleep(0.01)
                if not run:
                    time.sleep(0.5)
            if run:
                if _enter_pressed():
                    print("Shutdown requested, exiting.")
                    break
                yaw_world = imu.get_yaw()
                if yaw_world is None:
                    time.sleep(0.01)
                    continue
                yaw_relative = imu_yaw_to_relative_yaw(yaw_world, startup_yaw)
                lidar.set_imu_yaw(yaw_relative)

                # print(f"Yaw: {yaw_world:.6f} deg (relative {yaw_relative:.6f} deg)")
                lidar.predict_odometry(0.0, 0.0, 0.0, 0.01)
                x_pos, y_pos, mcl_yaw, _confidence = lidar.get_pose()
                if mcl_yaw is not None:
                    yaw_relative = mcl_yaw
                if x_pos is None or y_pos is None:
                    time.sleep(0.01)
                    continue
                enemy_bot_positions = []
                camera_frame_id, ball_direction, ball_distance = camera.get_measurement()
                has_new_camera_frame = camera_frame_id != last_camera_frame_id
                last_camera_frame_id = camera_frame_id
                if has_new_camera_frame:
                    ball_x = x_pos + ball_distance * math.cos(math.radians(ball_direction)) if ball_distance is not None and ball_direction is not None and x_pos is not None else None
                    ball_y = y_pos + ball_distance * math.sin(math.radians(ball_direction)) if ball_distance is not None and ball_direction is not None and y_pos is not None else None
                else:
                    ball_x = None
                    ball_y = None
                now = time.time()
                if ball_x is not None and ball_y is not None:
                    dt = now - last_ball_update
                    if last_ball_x is not None and last_ball_y is not None and dt > 0:
                        ball_dx = (ball_x - last_ball_x) / dt
                        ball_dy = (ball_y - last_ball_y) / dt
                    last_ball_x = ball_x
                    last_ball_y = ball_y
                    last_ball_update = now
                elif (
                    last_ball_x is not None
                    and last_ball_y is not None
                    and now - last_ball_update < BALL_TIMEOUT
                ):
                    dt_lost = now - last_ball_update
                    ball_x = last_ball_x + ball_dx * dt_lost
                    ball_y = last_ball_y + ball_dy * dt_lost
                else:
                    ball_x = None
                    ball_y = None
                distance_to_ball = tof.read()
                # print(f"Distance to ball: {distance_to_ball} mm")
                if distance_to_ball is not None and distance_to_ball < BALL_CAPTURED_DISTANCE:
                    ball_captured = True
                    ball_x = x_pos + 150 * math.cos(math.radians(yaw_relative))
                    ball_y = y_pos + 150 * math.sin(math.radians(yaw_relative))
                else:
                    ball_captured = False
                if args.stream:
                    log_values = [x_pos, y_pos, yaw_relative, ball_x, ball_y]
                    for other_x, other_y in enemy_bot_positions:
                        log_values.extend((other_x, other_y))
                    send_log.update_latest_log(
                        ",".join("None" if value is None else str(value) for value in log_values)
                    )
                direction, speed, rotation, steering_state, kick = striker(
                    x_pos,
                    y_pos,
                    yaw_relative,
                    ball_x,
                    ball_y,
                    ball_captured,
                    steering_state=steering_state,
                    friendly_bot_positions=[],
                    enemy_bot_positions=enemy_bot_positions,
                    )
                if kick:
                    kicker.kick()
                try:
                    movement_controller.move(direction, speed, rotation, 1.0, yaw_relative)
                except MotorCommunicationError as exc:
                    print(exc)
                    raise
                else:
                    ball_x = None
                    ball_y = None
                now = time.time()
                if ball_x is not None and ball_y is not None:
                    dt = now - last_ball_update
                    if last_ball_x is not None and last_ball_y is not None and dt > 0:
                        ball_dx = (ball_x - last_ball_x) / dt
                        ball_dy = (ball_y - last_ball_y) / dt
                    last_ball_x = ball_x
                    last_ball_y = ball_y
                    last_ball_update = now
                elif (
                    last_ball_x is not None
                    and last_ball_y is not None
                    and now - last_ball_update < BALL_TIMEOUT
                ):
                    dt_lost = now - last_ball_update
                    ball_x = last_ball_x + ball_dx * dt_lost
                    ball_y = last_ball_y + ball_dy * dt_lost
                else:
                    ball_x = None
                    ball_y = None
                distance_to_ball = tof.read()
                # print(f"Distance to ball: {distance_to_ball} mm")
                if distance_to_ball is not None and distance_to_ball < BALL_CAPTURED_DISTANCE:
                    ball_captured = True
                    ball_x = x_pos + 150 * math.cos(math.radians(yaw_relative))
                    ball_y = y_pos + 150 * math.sin(math.radians(yaw_relative))
                else:
                    ball_captured = False
                if args.stream:
                    log_values = [x_pos, y_pos, yaw_relative, ball_x, ball_y]
                    for other_x, other_y in enemy_bot_positions:
                        log_values.extend((other_x, other_y))
                    send_log.update_latest_log(
                        ",".join("None" if value is None else str(value) for value in log_values)
                    )
                direction, speed, rotation, steering_state, kick = striker(
                    x_pos,
                    y_pos,
                    yaw_relative,
                    ball_x,
                    ball_y,
                    ball_captured,
                    steering_state=steering_state,
                    friendly_bot_positions=[],
                    enemy_bot_positions=enemy_bot_positions,
                )

                if kick:
                    kicker.kick()
                try:
                    movement_controller.move(direction, speed, rotation, 1.0, yaw_relative)
                except MotorCommunicationError as exc:
                    print(exc)
                    raise
            else:
                time.sleep(0.01)
                if movement_controller is not None:
                    movement_controller.stop()

    finally:
        if movement_controller is not None:
            try:
                movement_controller.stop()
            except Exception as exc:
                print(f"Warning: failed to stop motors cleanly: {exc}")
        if kicker is not None:
            try:
                kicker.deinit()
            except Exception as exc:
                print(f"Warning: failed to deinitialize kicker cleanly: {exc}")
        if camera is not None:
            try:
                camera.stop()
            except Exception as exc:
                print(f"Warning: failed to stop camera cleanly: {exc}")
        if imu is not None:
            try:
                imu.close()
            except Exception as exc:
                print(f"Warning: failed to close IMU cleanly: {exc}")
        try:
            lidar.shutdown()
        except Exception as exc:
            print(f"Warning: failed to shut down lidar cleanly: {exc}")
