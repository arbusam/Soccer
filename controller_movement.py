import pygame
from math import atan2, degrees, hypot
from movement import move, init_motors, stop_all_motors

WHEEL_DIAMETER = 50 # mm
WHEEL_LEVER_ARM = 100 # mm (distance from center to wheel contact)
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3 # deg

MAX_SPEED = 500
SPEED_ADJUST_RATE = 200 # mm/s per second at full trigger press
LEFT_TRIGGER_AXIS = 2
RIGHT_TRIGGER_AXIS = 5

speed = 25.0
pygame.init()
joysticks = []

for i in range(pygame.joystick.get_count()):
    joystick = pygame.joystick.Joystick(i)
    joystick.init()
    joysticks.append(joystick)
    print(f"Detected joystick: {joystick.get_name()}")

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


motors, motor_modes = init_motors(_prompt_i2c_addresses())

direction_degrees = 0.0
direction_magnitude = 0.0
rotation_degrees = 0.0
rotation_magnitude = 0.0
clock = pygame.time.Clock()
try:
    while True:
        dt = clock.tick(60) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        if joysticks:
            joystick = joysticks[0]
            axis_values = [joystick.get_axis(a) for a in range(joystick.get_numaxes())]
            x, y = axis_values[:2]
            direction_magnitude = hypot(x, y)
            if direction_magnitude < 0.1:
                direction_magnitude = 0.0
            direction_degrees = (degrees(atan2(y, x)) + 360) % 360
            direction_degrees = (direction_degrees + 90) % 360
            rotation_x = axis_values[3] if len(axis_values) > 2 else 0.0
            rotation_y = axis_values[4] if len(axis_values) > 3 else 0.0
            rotation_magnitude = hypot(rotation_x, rotation_y)
            rotation_degrees = (degrees(atan2(rotation_y, rotation_x)) + 360) % 360
            rotation_degrees = (rotation_degrees + 90) % 360
            if rotation_magnitude < 0.1:
                rotation_magnitude = 0.0
                rotation_degrees = 0.0

            right_trigger = 0.0
            left_trigger = 0.0
            if len(axis_values) > RIGHT_TRIGGER_AXIS:
                right_trigger = max(0.0, min(1.0, (axis_values[RIGHT_TRIGGER_AXIS] + 1.0) / 2.0))
            if len(axis_values) > LEFT_TRIGGER_AXIS:
                left_trigger = max(0.0, min(1.0, (axis_values[LEFT_TRIGGER_AXIS] + 1.0) / 2.0))

            speed += (right_trigger - left_trigger) * SPEED_ADJUST_RATE * dt
            speed = max(0.0, min(MAX_SPEED, speed))
            # print(f"direction_degrees={direction_degrees:.1f} direction_magnitude={direction_magnitude:.1f} rotation_degrees={rotation_degrees:.1f} rotation_magnitude={rotation_magnitude:.1f} rotation_x={rotation_x:.1f} rotation_y={rotation_y:.1f}")
            # print(f"left_trigger={left_trigger:.2f} right_trigger={right_trigger:.2f} speed={speed:.1f}")

        movement_speed = min(direction_magnitude * speed, speed)
        move(direction_degrees, movement_speed, rotation_degrees, rotation_magnitude, 0.0, motors, motor_modes, WHEEL_DIAMETER, MAX_YAW_RPM, MAX_MOTOR_RPM, YAW_CORRECT_THRESHOLD)
finally:
    stop_all_motors(motors)