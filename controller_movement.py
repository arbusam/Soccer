import pygame
from math import atan2, degrees, hypot
from movement import move, init_motors

WHEEL_DIAMETER = 50 # mm
WHEEL_LEVER_ARM = 100 # mm (distance from center to wheel contact)
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 400
YAW_CORRECT_THRESHOLD = 3 # deg

SPEED = 100
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
while True:
    axis_values = [joystick.get_axis(a) for a in range(joystick.get_numaxes())]
    x, y = axis_values[:2]
    if not x or not y:
        continue
    direction_magnitude = min(1.0, hypot(x, y))
    if direction_magnitude < 0.1:
        direction_magnitude = 0.0
    direction_degrees = (degrees(atan2(y, x)) + 360) % 360
    direction_degrees += 90
    direction_degrees %= 360

    rotation_x = axis_values[2] if len(axis_values) > 2 else 0.0
    rotation_y = axis_values[3] if len(axis_values) > 3 else 0.0
    rotation_magnitude = min(1.0, hypot(rotation_x, rotation_y))
    if rotation_magnitude < 0.1:
        rotation_magnitude = 0.0
    rotation_degrees = (degrees(atan2(rotation_y, rotation_x)) + 360) % 360
    rotation_degrees = (rotation_degrees + 90) % 360
    print(f"Direction: {direction_degrees}, Direction Magnitude: {direction_magnitude}, Rotation: {rotation_degrees}, Rotation Magnitude: {rotation_magnitude}")
    direction_magnitude *= SPEED
    move(direction_degrees, direction_magnitude, rotation_degrees, 0.0, motors, motor_modes, WHEEL_DIAMETER, MAX_YAW_RPM, MAX_MOTOR_RPM, YAW_CORRECT_THRESHOLD)