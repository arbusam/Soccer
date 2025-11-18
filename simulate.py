import argparse
import math
import sys

import pygame

parser = argparse.ArgumentParser(
    description="Visualize a simulated match from a log file.",
    epilog=(
        "Game log format: "
        "x_pos,y_pos,yaw,ball_x,ball_y,bot1_x,bot1_y,bot2_x,bot2_y,..."
    ),
)
parser.add_argument(
    "log_file",
    nargs="?",
    help="Path to the game log file (CSV format) containing frame-by-frame positions.",
)
args = parser.parse_args()

lines = []
log_provided = args.log_file is not None

if log_provided:
    try:
        with open(args.log_file, "r") as f:
            lines = f.readlines()
    except OSError as exc:
        print(f"Unable to read log file '{args.log_file}': {exc}")
        sys.exit(1)

pygame.init()

display = pygame.display.set_mode((1215, 910))

pitch = pygame.Surface((2430, 1820))

green = (20, 110, 44)
white = (255, 255, 255)
black = (0, 0, 0)
cyan = (0, 255, 255)
yellow = (255, 255, 0)
orange = (255, 165, 0)

pitch.fill(green)

#Pitch markings

pygame.draw.rect(pitch, white, pygame.Rect(250, 250, 1930, 1320), 50)
pygame.draw.circle(pitch, black, (1215, 910), 10)
pygame.draw.circle(pitch, black, (1215, 610), 10)
pygame.draw.circle(pitch, black, (1215, 1210), 10)

#Goal Boxes

pygame.draw.line(pitch, black, (300, 460), (610, 460), 20)
pygame.draw.line(pitch, black, (600, 460), (600, 1370), 20)
pygame.draw.line(pitch, black, (600, 1360), (300, 1360), 20)
pygame.draw.line(pitch, black, (2130, 460), (1820, 460), 20)
pygame.draw.line(pitch, black, (1830, 460), (1830, 1370), 20)
pygame.draw.line(pitch, black, (1830, 1360), (2130, 1360), 20)

#Goals

pygame.draw.rect(pitch, cyan, pygame.Rect(226, 685, 74, 450))
pygame.draw.rect(pitch, yellow, pygame.Rect(2130, 685, 74, 450))
pygame.draw.line(pitch, black, (300, 685), (222, 685), 10)
pygame.draw.line(pitch, black, (226, 685), (226, 1140), 10)
pygame.draw.line(pitch, black, (226, 1135), (300, 1135), 10)
pygame.draw.line(pitch, black, (2130, 685), (2208, 685), 10)
pygame.draw.line(pitch, black, (2204, 685), (2204, 1140), 10)
pygame.draw.line(pitch, black, (2204, 1135), (2130, 1135), 10)

base_pitch = pitch.copy()

clock = pygame.time.Clock()

# Game log format: x_pos,y_pos,yaw,ball_x,ball_y,bot1_x,bot1_y,bot2_x,bot2_y,...

def build_frame(surface, x_pos, y_pos, yaw, ball_x, ball_y, bot_coords):
    frame_pitch = surface.copy()
    pygame.draw.circle(frame_pitch, yellow, (x_pos, y_pos), 110)
    radius = 110
    angle = math.radians(yaw)
    end_x = x_pos + radius * math.cos(angle)
    end_y = y_pos + radius * math.sin(angle)
    pygame.draw.line(frame_pitch, black, (x_pos, y_pos), (end_x, end_y), 12)
    head_len = 35
    head_offset = math.radians(25)
    for offset in (-head_offset, head_offset):
        side_angle = angle + math.pi + offset
        side_x = end_x + head_len * math.cos(side_angle)
        side_y = end_y + head_len * math.sin(side_angle)
        pygame.draw.line(frame_pitch, black, (end_x, end_y), (side_x, side_y), 12)

    for bot_x, bot_y in bot_coords:
        pygame.draw.circle(frame_pitch, cyan, (bot_x, bot_y), 55)

    pygame.draw.circle(frame_pitch, orange, (ball_x, ball_y), 21)
    return frame_pitch


def blit_frame(frame_surface):
    scaled = pygame.transform.smoothscale(frame_surface, (1215, 910))
    display.blit(scaled, (0, 0))
    pygame.display.flip()


if log_provided:
    for line in lines:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        line_data = line.strip().split(",")
        x_pos = int(float(line_data[0]))
        y_pos = int(float(line_data[1]))
        yaw = float(line_data[2])
        ball_x = int(float(line_data[3]))
        ball_y = int(float(line_data[4]))
        bots = []
        for i in range(5, len(line_data), 2):
            bot_x = int(float(line_data[i]))
            bot_y = int(float(line_data[i + 1]))
            bots.append((bot_x, bot_y))
        frame_pitch = build_frame(base_pitch, x_pos, y_pos, yaw, ball_x, ball_y, bots)
        blit_frame(frame_pitch)
        clock.tick(30)
else:
    x_pos = 600
    y_pos = pitch.get_height() // 2
    ball_x = pitch.get_width() // 2
    ball_y = pitch.get_height() // 2
    yaw = 0
    frame_pitch = build_frame(base_pitch, x_pos, y_pos, yaw, ball_x, ball_y, [])
    blit_frame(frame_pitch)
    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                waiting = False
        clock.tick(30)

pygame.quit()
exit()
