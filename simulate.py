import pygame
import math

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
f = open("test_game_log.txt", "r")
lines = f.readlines()
f.close()

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
    frame_pitch = base_pitch.copy()
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
    
    for i in range(5, len(line_data), 2):
        bot_x = int(float(line_data[i]))
        bot_y = int(float(line_data[i+1]))
        pygame.draw.circle(frame_pitch, cyan, (bot_x, bot_y), 55)
    
    pygame.draw.circle(frame_pitch, orange, (ball_x, ball_y), 21)
    scaled = pygame.transform.smoothscale(frame_pitch, (1215, 910))
    display.blit(scaled, (0, 0))
    pygame.display.flip()
    clock.tick(30)

pygame.quit()
exit()