import pygame
from math import atan2, degrees, hypot
from movement import move, init_motors

WHEEL_DIAMETER = 50 # mm

pygame.init()
joysticks = []

for i in range(pygame.joystick.get_count()):
    joystick = pygame.joystick.Joystick(i)
    joystick.init()
    joysticks.append(joystick)
    print(f"Detected joystick: {joystick.get_name()}")

motors, motor_modes = init_motors()

while True:
    direction_degrees = 0.0
    direction_magnitude = 0.0
    rotation_degrees = 0.0
    rotation_magnitude = 0.0
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            axis_values = [joystick.get_axis(a) for a in range(joystick.get_numaxes())]
            x, y = axis_values[:2]
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
    move(direction_degrees, direction_magnitude, rotation_degrees, motors, motor_modes, WHEEL_DIAMETER)