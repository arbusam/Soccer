import argparse
import math
import sys

import pygame

from defence import defence, goalie

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
parser.add_argument(
    "-d",
    "--defence",
    action="store_true",
    help="Use the defence strategy from defence.py instead of keyboard control.",
)
parser.add_argument(
    "-g",
    "--goalie",
    action="store_true",
    help="Use the goalie strategy from defence.py instead of keyboard control.",
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
PITCH_WIDTH = pitch.get_width()
PITCH_HEIGHT = pitch.get_height()

green = (20, 110, 44)
white = (255, 255, 255)
black = (0, 0, 0)
cyan = (0, 255, 255)
yellow = (255, 255, 0)
orange = (255, 165, 0)
red = (255, 0, 0)

WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570

ROBOT_RADIUS = 110
BALL_CAPTURE_ZONE_WIDTH = 50
CAPTURE_OFFSET = ROBOT_RADIUS - 15
CONNECTOR_ANGLE_DEG = 45
CONNECTOR_ANGLE_RAD = math.radians(CONNECTOR_ANGLE_DEG)
CUTOUT_ARC_SEGMENTS = 50
CAPTURE_LINE_WIDTH = 4
CAPTURE_LINE_HALF_WIDTH = CAPTURE_LINE_WIDTH / 2.0
BALL_RADIUS = 21
BALL_DECELERATION_SPEED = 1000  # mm/s^2
YAW_CORRECT_SPEED = 200  # mm/s spin speed used when aligning yaw
WALL_BOUNCE_ENERGY_LOSS = 0.7
EPSILON = 1e-6
ACCELERATION = 2000  # mm/s^2

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


def mmps_to_pixels_per_frame(value_mm_per_s):
    return (value_mm_per_s / MM_PER_PIXEL) / FPS


BALL_DECELERATION_PER_FRAME = mmps_to_pixels_per_frame(BALL_DECELERATION_SPEED / FPS)
YAW_CORRECT_DEG_PER_FRAME = math.degrees(
    mmps_to_pixels_per_frame(YAW_CORRECT_SPEED) / ROBOT_RADIUS
)

# Game log format: x_pos,y_pos,yaw,ball_x,ball_y,bot1_x,bot1_y,bot2_x,bot2_y,...

def normalize_angle_deg(angle):
    return angle % 360


def shortest_angle_delta(current, target):
    return (target - current + 180) % 360 - 180

def is_out_of_white_boundary(x_pos, y_pos, bot_radius):
    # Find closest point on rectangle to circle center
    closest_x = max(WHITE_MIN_X, min(x_pos, WHITE_MAX_X))
    closest_y = max(WHITE_MIN_Y, min(y_pos, WHITE_MAX_Y))
    
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


def calculate_capture_zone_back(x_pos, y_pos, angle_rad):
    line_center_x = x_pos + CAPTURE_OFFSET * math.cos(angle_rad)
    line_center_y = y_pos + CAPTURE_OFFSET * math.sin(angle_rad)

    half_width = BALL_CAPTURE_ZONE_WIDTH / 2.0
    perpendicular_angle = angle_rad + math.pi / 2.0
    dx = half_width * math.cos(perpendicular_angle)
    dy = half_width * math.sin(perpendicular_angle)

    line_start = (line_center_x - dx, line_center_y - dy)
    line_end = (line_center_x + dx, line_center_y + dy)
    return line_start, line_end


def ray_circle_intersection(px, py, angle_rad, circle_center, radius):
    cx, cy = circle_center
    dir_x = math.cos(angle_rad)
    dir_y = math.sin(angle_rad)

    fx = px - cx
    fy = py - cy

    b = fx * dir_x + fy * dir_y
    c = fx * fx + fy * fy - radius * radius
    discriminant = b * b - c

    if discriminant < 0:
        return circle_center

    sqrt_disc = math.sqrt(discriminant)
    t1 = -b - sqrt_disc
    t2 = -b + sqrt_disc

    ts = [t for t in (t1, t2) if t > 0]
    if not ts:
        return circle_center

    t = min(ts)
    hit_x = px + t * dir_x
    hit_y = py + t * dir_y
    return hit_x, hit_y


def build_clockwise_arc_points(start_point, end_point, center, radius, segments):
    if segments < 2 or start_point == center or end_point == center or radius <= 0:
        return []

    cx, cy = center
    two_pi = 2 * math.pi

    start_angle = math.atan2(start_point[1] - cy, start_point[0] - cx) % two_pi
    end_angle = math.atan2(end_point[1] - cy, end_point[0] - cx) % two_pi

    cw_span = (start_angle - end_angle) % two_pi
    ccw_span = (end_angle - start_angle) % two_pi
    use_cw = True
    span = cw_span if use_cw else ccw_span

    if math.isclose(span, 0, abs_tol=1e-5):
        return []

    angle_step = span / segments

    points = []
    for i in range(1, segments):
        if use_cw:
            angle = (start_angle - angle_step * i) % two_pi
        else:
            angle = (start_angle + angle_step * i) % two_pi
        px = cx + radius * math.cos(angle)
        py = cy + radius * math.sin(angle)
        points.append((px, py))
    return points


def get_capture_geometry(x_pos, y_pos, yaw):
    angle = math.radians(yaw)
    line_start, line_end = calculate_capture_zone_back(x_pos, y_pos, angle)
    robot_center = (x_pos, y_pos)

    angle_left = angle - CONNECTOR_ANGLE_RAD
    angle_right = angle + CONNECTOR_ANGLE_RAD
    left_circle = ray_circle_intersection(
        line_start[0], line_start[1], angle_left, robot_center, ROBOT_RADIUS
    )
    right_circle = ray_circle_intersection(
        line_end[0], line_end[1], angle_right, robot_center, ROBOT_RADIUS
    )

    connector_lines = [
        (line_start, left_circle),
        (line_end, right_circle),
        (line_start, line_end),
    ]

    arc_segments = []
    if left_circle != robot_center and right_circle != robot_center:
        arc_points = build_clockwise_arc_points(
            left_circle, right_circle, robot_center, ROBOT_RADIUS, CUTOUT_ARC_SEGMENTS
        )
        arc_points_full = [left_circle] + arc_points + [right_circle]
        for i in range(len(arc_points_full) - 1):
            arc_segments.append((arc_points_full[i], arc_points_full[i + 1]))

    segments = connector_lines + arc_segments

    return {
        "angle": angle,
        "line_start": line_start,
        "line_end": line_end,
        "robot_center": robot_center,
        "connector_lines": connector_lines,
        "arc_segments": arc_segments,
        "segments": segments,
    }


def build_frame(x_pos, y_pos, yaw, ball_x, ball_y, bot_coords, robot_color=yellow):
    frame_pitch = base_pitch.copy()
    pygame.draw.circle(frame_pitch, robot_color, (x_pos, y_pos), ROBOT_RADIUS)
    geometry = get_capture_geometry(x_pos, y_pos, yaw)

    def _int_point(point):
        return (int(point[0]), int(point[1]))

    connector_lines = [
        (_int_point(start), _int_point(end)) for start, end in geometry["connector_lines"]
    ]

    for start, end in geometry["arc_segments"]:
        pygame.draw.line(frame_pitch, black, _int_point(start), _int_point(end), CAPTURE_LINE_WIDTH)

    for start, end in connector_lines:
        pygame.draw.line(frame_pitch, black, start, end, CAPTURE_LINE_WIDTH)


    for bot_x, bot_y in bot_coords:
        pygame.draw.circle(frame_pitch, cyan, (bot_x, bot_y), 110)

    pygame.draw.circle(frame_pitch, orange, (ball_x, ball_y), BALL_RADIUS)
    return frame_pitch


def resolve_circle_segment_penetration(
    circle_x,
    circle_y,
    radius,
    segment_start,
    segment_end,
    line_half_width,
    outward_reference=None,
):
    distance, closest_point = point_to_line_segment_distance(
        circle_x, circle_y, segment_start[0], segment_start[1], segment_end[0], segment_end[1]
    )
    allowed_distance = radius + line_half_width
    if distance >= allowed_distance:
        return circle_x, circle_y, None

    diff_x = circle_x - closest_point[0]
    diff_y = circle_y - closest_point[1]
    norm = math.hypot(diff_x, diff_y)
    if norm < EPSILON:
        if outward_reference is not None:
            diff_x = circle_x - outward_reference[0]
            diff_y = circle_y - outward_reference[1]
            norm = math.hypot(diff_x, diff_y)
    if norm < EPSILON:
        # Default normal if we still cannot determine direction
        diff_x, diff_y = 1.0, 0.0
        norm = 1.0

    normal_x = diff_x / norm
    normal_y = diff_y / norm
    penetration = allowed_distance - distance
    circle_x += normal_x * penetration
    circle_y += normal_y * penetration
    return circle_x, circle_y, (normal_x, normal_y)


def resolve_ball_capture_collisions(ball_x, ball_y, ball_vx, ball_vy, geometry, bot_velocity):
    ball_being_pushed = False
    bot_speed = math.hypot(bot_velocity[0], bot_velocity[1])
    for segment_start, segment_end in geometry["segments"]:
        ball_x, ball_y, normal = resolve_circle_segment_penetration(
            ball_x,
            ball_y,
            BALL_RADIUS,
            segment_start,
            segment_end,
            CAPTURE_LINE_HALF_WIDTH,
            geometry["robot_center"],
        )
        if normal is None:
            continue
        impact = 0.0
        if bot_speed > EPSILON:
            impact = bot_velocity[0] * normal[0] + bot_velocity[1] * normal[1]
        if impact > 0:
            alignment = min(1.0, impact / (bot_speed + EPSILON))
            ball_vx = bot_velocity[0] * alignment
            ball_vy = bot_velocity[1] * alignment
            ball_being_pushed = True
        else:
            vel_dot = ball_vx * normal[0] + ball_vy * normal[1]
            if vel_dot < 0:
                ball_vx -= vel_dot * normal[0]
                ball_vy -= vel_dot * normal[1]

    return ball_x, ball_y, ball_vx, ball_vy, ball_being_pushed


def keep_ball_in_pitch_bounds(ball_x, ball_y, ball_vx, ball_vy):
    ball_min_x = BALL_RADIUS
    ball_max_x = PITCH_WIDTH - BALL_RADIUS
    ball_min_y = BALL_RADIUS
    ball_max_y = PITCH_HEIGHT - BALL_RADIUS

    against_boundary = False

    if ball_x < ball_min_x:
        ball_x = ball_min_x
        ball_vx = abs(ball_vx) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True
    elif ball_x > ball_max_x:
        ball_x = ball_max_x
        ball_vx = -abs(ball_vx) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True

    if ball_y < ball_min_y:
        ball_y = ball_min_y
        ball_vy = abs(ball_vy) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True
    elif ball_y > ball_max_y:
        ball_y = ball_max_y
        ball_vy = -abs(ball_vy) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True

    return ball_x, ball_y, ball_vx, ball_vy, against_boundary


def is_ball_touching_pitch_bounds(ball_x, ball_y):
    ball_min_x = BALL_RADIUS
    ball_max_x = PITCH_WIDTH - BALL_RADIUS
    ball_min_y = BALL_RADIUS
    ball_max_y = PITCH_HEIGHT - BALL_RADIUS
    return (
        ball_x <= ball_min_x + EPSILON
        or ball_x >= ball_max_x - EPSILON
        or ball_y <= ball_min_y + EPSILON
        or ball_y >= ball_max_y - EPSILON
    )


def is_ball_touching_goal_lines(ball_x, ball_y, goal_lines):
    for (x1, y1), (x2, y2), line_width in goal_lines:
        distance, _ = point_to_line_segment_distance(ball_x, ball_y, x1, y1, x2, y2)
        allowed_distance = BALL_RADIUS + line_width / 2.0
        if distance <= allowed_distance + EPSILON:
            return True
    return False


def resolve_ball_goal_line_collisions(ball_x, ball_y, ball_vx, ball_vy, goal_lines):
    against_goal_line = False
    for (x1, y1), (x2, y2), line_width in goal_lines:
        distance, closest_point = point_to_line_segment_distance(ball_x, ball_y, x1, y1, x2, y2)
        allowed_distance = BALL_RADIUS + line_width / 2.0
        if distance >= allowed_distance:
            continue
        diff_x = ball_x - closest_point[0]
        diff_y = ball_y - closest_point[1]
        norm = math.hypot(diff_x, diff_y)
        if norm < EPSILON:
            continue
        normal_x = diff_x / norm
        normal_y = diff_y / norm
        penetration = allowed_distance - distance
        ball_x += normal_x * penetration
        ball_y += normal_y * penetration
        vel_dot = ball_vx * normal_x + ball_vy * normal_y
        if vel_dot < 0:
            ball_vx -= 2 * vel_dot * normal_x
            ball_vy -= 2 * vel_dot * normal_y
            ball_vx *= WALL_BOUNCE_ENERGY_LOSS
            ball_vy *= WALL_BOUNCE_ENERGY_LOSS
        against_goal_line = True
    return ball_x, ball_y, ball_vx, ball_vy, against_goal_line
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
    x_pos = 1500
    y_pos = pitch.get_height() // 2
    ball_x = pitch.get_width() // 2
    ball_y = pitch.get_height() // 2
    ball_vx = 0.0
    ball_vy = 0.0
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
    
    # Tracks when the robot came back in bounds so we can keep it red
    # for a short duration after re-entering.
    red_until_time = None
    was_out_of_bounds = False
    RED_DURATION_MS = 1000

    current_speed = 0
    current_direction = 0
    
    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                waiting = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                ball_x, ball_y = pygame.mouse.get_pos()[0]*2, pygame.mouse.get_pos()[1]*2
                ball_vx = 0.0
                ball_vy = 0.0

        if args.defence:
            # Simulate defence code
            direction, speed, rotation = defence(x_pos, y_pos, yaw, ball_x, ball_y)
        elif args.goalie:
            # Simulate goalie code
            direction, speed, rotation = goalie(x_pos, y_pos, yaw, ball_x, ball_y)
        else:
            keys = pygame.key.get_pressed()
            up = keys[pygame.K_UP]
            down = keys[pygame.K_DOWN]
            left = keys[pygame.K_LEFT]
            right = keys[pygame.K_RIGHT]
            ctrl = keys[pygame.K_LCTRL]
            alt = keys[pygame.K_LALT]

            direction = None
            speed = 0
            rotation = yaw

            if up and not down:
                if left and not right:
                    direction = (270 + 180) / 2  # Up + Left
                elif right and not left:
                    direction = (270 + 360) / 2    # Up + Right
                elif not left and not right:
                    direction = 270              # Up only
            elif down and not up:
                if left and not right:
                    direction = (90 + 180) / 2   # Down + Left
                elif right and not left:
                    direction = (90 + 0) / 2     # Down + Right
                elif not left and not right:
                    direction = 90               # Down only
            elif left and not right and not up and not down: # Left only
                direction = 180
            elif right and not left and not up and not down: # Right only
                direction = 0
            
            if ctrl:
                rotation = (yaw - 10) % 360
            elif alt:
                rotation = (yaw + 10) % 360

        #Acceleration code
        if current_speed > speed + mmps_to_pixels_per_frame(ACCELERATION / FPS):
            current_speed -= mmps_to_pixels_per_frame(ACCELERATION)
        elif current_speed < speed - mmps_to_pixels_per_frame(ACCELERATION / FPS):
            current_speed += mmps_to_pixels_per_frame(ACCELERATION)
        
        speed = current_speed
        if direction is None:
            direction = current_direction
        else:
            current_direction = direction
        

        # Rotate towards target yaw while continuing to move
        rotation_target = rotation if rotation is not None else yaw
        yaw_error = shortest_angle_delta(yaw, rotation_target)
        if abs(yaw_error) > EPSILON:
            yaw_step = math.copysign(
                min(abs(yaw_error), YAW_CORRECT_DEG_PER_FRAME),
                yaw_error,
            )
            yaw = normalize_angle_deg(yaw + yaw_step)

        # Calculate effective direction relative to yaw
        effective_direction = (yaw + direction) % 360
        effective_direction_rad = math.radians(effective_direction)
        
        # Convert speed from mm/s to pixels per frame
        pixels_per_frame = mmps_to_pixels_per_frame(speed)
        
        # Store position before attempted move
        prev_x = x_pos
        prev_y = y_pos
        
        bot_step_x = pixels_per_frame * math.cos(effective_direction_rad)
        bot_step_y = pixels_per_frame * math.sin(effective_direction_rad)
        
        # Move bot
        x_pos += bot_step_x
        y_pos += bot_step_y
        
        # Check for collision with goal
        if check_collision_with_goal_lines(x_pos, y_pos, bot_radius, goal_lines):
            # Revert to previous position if collision detected
            x_pos = prev_x
            y_pos = prev_y
        
        # Clamp position to pitch
        x_pos = max(min_x, min(max_x, x_pos))
        y_pos = max(min_y, min(max_y, y_pos))
        bot_actual_dx = x_pos - prev_x
        bot_actual_dy = y_pos - prev_y
        
        # Update ball position based on current velocity
        ball_x += ball_vx
        ball_y += ball_vy
        
        ball_x, ball_y, ball_vx, ball_vy, boundary_collision_pre = keep_ball_in_pitch_bounds(
            ball_x, ball_y, ball_vx, ball_vy
        )
        ball_x, ball_y, ball_vx, ball_vy, goal_line_collision_pre = resolve_ball_goal_line_collisions(
            ball_x, ball_y, ball_vx, ball_vy, goal_lines
        )
        geometry = get_capture_geometry(x_pos, y_pos, yaw)
        ball_x, ball_y, ball_vx, ball_vy, ball_being_pushed = resolve_ball_capture_collisions(
            ball_x, ball_y, ball_vx, ball_vy, geometry, (bot_actual_dx, bot_actual_dy)
        )
        ball_x, ball_y, ball_vx, ball_vy, boundary_collision_post = keep_ball_in_pitch_bounds(
            ball_x, ball_y, ball_vx, ball_vy
        )
        (
            ball_x,
            ball_y,
            ball_vx,
            ball_vy,
            goal_line_collision_post,
        ) = resolve_ball_goal_line_collisions(ball_x, ball_y, ball_vx, ball_vy, goal_lines)
        ball_against_surface = (
            boundary_collision_pre
            or goal_line_collision_pre
            or boundary_collision_post
            or goal_line_collision_post
            or is_ball_touching_pitch_bounds(ball_x, ball_y)
            or is_ball_touching_goal_lines(ball_x, ball_y, goal_lines)
        )
        
        if (
            ball_against_surface
            and ball_being_pushed
            and (abs(bot_actual_dx) > EPSILON or abs(bot_actual_dy) > EPSILON)
        ):
            # Ball is pinned; stop the bot and hold the ball in place
            x_pos = prev_x
            y_pos = prev_y
            bot_actual_dx = 0.0
            bot_actual_dy = 0.0
            ball_vx = 0.0
            ball_vy = 0.0
            geometry = get_capture_geometry(x_pos, y_pos, yaw)
            ball_x, ball_y, ball_vx, ball_vy, _ = resolve_ball_capture_collisions(
                ball_x, ball_y, ball_vx, ball_vy, geometry, (0.0, 0.0)
            )
            ball_being_pushed = False
        
        if not ball_being_pushed:
            ball_speed = math.hypot(ball_vx, ball_vy)
            if ball_speed > BALL_DECELERATION_PER_FRAME:
                scale = (ball_speed - BALL_DECELERATION_PER_FRAME) / ball_speed
                ball_vx *= scale
                ball_vy *= scale
            else:
                ball_vx = 0.0
                ball_vy = 0.0
        
        # Check if robot is out of white boundary
        current_time = pygame.time.get_ticks()
        is_out_of_bounds = is_out_of_white_boundary(x_pos, y_pos, bot_radius)

        # Determine robot color:
        # - Stay red the entire time the robot is out of bounds.
        # - Once it returns in bounds, start a 5 second timer during which it stays red.
        if is_out_of_bounds:
            # While out of bounds, always red and cancel any post-return timer.
            robot_color = red
            red_until_time = None
        else:
            # Just came back in bounds this frame: start the red timer.
            if was_out_of_bounds:
                red_until_time = current_time + RED_DURATION_MS

            # Default color in bounds.
            robot_color = yellow

            # If a red timer is active, stay red until it expires.
            if red_until_time is not None and current_time <= red_until_time:
                robot_color = red
            elif red_until_time is not None and current_time > red_until_time:
                red_until_time = None

        # Remember last frame's out-of-bounds state.
        was_out_of_bounds = is_out_of_bounds
        
        # Rebuild and blit frame
        frame_pitch = build_frame(x_pos, y_pos, yaw, ball_x, ball_y, [], robot_color)
        blit_frame(frame_pitch)
        
        clock.tick(FPS)

pygame.quit()
exit()
