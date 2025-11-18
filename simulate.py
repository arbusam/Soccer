import argparse
import math
import sys

import pygame

from defence import defence

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
red = (255, 0, 0)

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

# Scale factor: millimeters per pixel
# Pitch playable area is approximately 1930x1320 pixels
# Adjust this constant based on real-world pitch dimensions
MM_PER_PIXEL = 1.0  # Default: 1mm per pixel (adjust as needed)
FPS = 30

# Game log format: x_pos,y_pos,yaw,ball_x,ball_y,bot1_x,bot1_y,bot2_x,bot2_y,...

def is_out_of_white_boundary(x_pos, y_pos, bot_radius):
    white_min_x = 300
    white_max_x = 2180
    white_min_y = 300
    white_max_y = 1570
    
    # Find closest point on rectangle to circle center
    closest_x = max(white_min_x, min(x_pos, white_max_x))
    closest_y = max(white_min_y, min(y_pos, white_max_y))
    
    # Calculate distance from circle center to closest point on rectangle
    dx = x_pos - closest_x
    dy = y_pos - closest_y
    distance = math.sqrt(dx * dx + dy * dy)
    
    # Circle is completely outside if distance > radius
    return distance > bot_radius


def point_to_line_segment_distance(px, py, x1, y1, x2, y2):
    # Vector from line start to end
    dx = x2 - x1
    dy = y2 - y1
    
    # If line segment has zero length, return distance to endpoint
    line_length_sq = dx * dx + dy * dy
    if line_length_sq == 0:
        dist = math.sqrt((px - x1) ** 2 + (py - y1) ** 2)
        return dist, (x1, y1)
    
    # Vector from line start to point
    to_point_x = px - x1
    to_point_y = py - y1
    
    # Project point onto line (t is parameter along line segment)
    t = max(0, min(1, (to_point_x * dx + to_point_y * dy) / line_length_sq))
    
    # Closest point on line segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    
    # Distance from point to closest point on segment
    dist_x = px - closest_x
    dist_y = py - closest_y
    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
    
    return distance, (closest_x, closest_y)


def check_collision_with_goal_lines(x_pos, y_pos, bot_radius, goal_lines):
    for line in goal_lines:
        (x1, y1), (x2, y2), line_width = line
        distance, _ = point_to_line_segment_distance(x_pos, y_pos, x1, y1, x2, y2)
        
        # Collision if distance < (circle_radius + line_width/2)
        if distance < (bot_radius + line_width / 2):
            return True
    return False


def build_frame(x_pos, y_pos, yaw, ball_x, ball_y, bot_coords, robot_color=yellow):
    frame_pitch = base_pitch.copy()
    pygame.draw.circle(frame_pitch, robot_color, (x_pos, y_pos), 110)
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
        frame_pitch = build_frame(x_pos, y_pos, yaw, ball_x, ball_y, bots, yellow)
        blit_frame(frame_pitch)
        clock.tick(FPS)
else:
    x_pos = 600
    y_pos = pitch.get_height() // 2
    ball_x = pitch.get_width() // 2
    ball_y = pitch.get_height() // 2
    yaw = 0
    
    # Window boundaries (accounting for bot radius of 110 pixels)
    bot_radius = 110
    min_x = bot_radius
    max_x = pitch.get_width() - bot_radius
    min_y = bot_radius
    max_y = pitch.get_height() - bot_radius
    
    # Goal lines: ((x1, y1), (x2, y2), line_width)
    goal_lines = [
        ((300, 685), (222, 685), 10),
        ((226, 685), (226, 1140), 10),
        ((226, 1135), (300, 1135), 10),
        ((2130, 685), (2208, 685), 10),
        ((2204, 685), (2204, 1140), 10),
        ((2204, 1135), (2130, 1135), 10),
    ]
    
    out_of_bounds_start_time = None
    RED_DURATION_MS = 5000
    
    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                waiting = False
        
        direction, speed = defence(x_pos, y_pos, yaw, ball_x, ball_y)
        
        # Calculate effective direction relative to yaw
        effective_direction = (yaw + direction) % 360
        effective_direction_rad = math.radians(effective_direction)
        
        # Convert speed from mm/s to pixels per frame
        pixels_per_frame = (speed / MM_PER_PIXEL) / FPS
        
        # Store postion before attempted move
        prev_x = x_pos
        prev_y = y_pos
        
        # Move bot
        x_pos += pixels_per_frame * math.cos(effective_direction_rad)
        y_pos += pixels_per_frame * math.sin(effective_direction_rad)
        
        # Check for collision with goal
        if check_collision_with_goal_lines(x_pos, y_pos, bot_radius, goal_lines):
            # Revert to previous position if collision detected
            x_pos = prev_x
            y_pos = prev_y
        
        # Clamp position to pitch
        x_pos = max(min_x, min(max_x, x_pos))
        y_pos = max(min_y, min(max_y, y_pos))
        
        # Check if robot is out of white boundary
        current_time = pygame.time.get_ticks()
        is_out_of_bounds = is_out_of_white_boundary(x_pos, y_pos, bot_radius)
        
        # Update out-of-bounds state
        if is_out_of_bounds:
            if out_of_bounds_start_time is None:
                # Just entered out-of-bounds state, start timer
                out_of_bounds_start_time = current_time
        
        # Determine robot color - stay red for 5 seconds after going out of bounds
        robot_color = yellow
        if out_of_bounds_start_time is not None:
            elapsed_ms = current_time - out_of_bounds_start_time
            if elapsed_ms < RED_DURATION_MS:
                robot_color = red
            else:
                out_of_bounds_start_time = None
        
        # Rebuild and blit frame
        frame_pitch = build_frame(x_pos, y_pos, yaw, ball_x, ball_y, [], robot_color)
        blit_frame(frame_pitch)
        
        clock.tick(FPS)

pygame.quit()
exit()
