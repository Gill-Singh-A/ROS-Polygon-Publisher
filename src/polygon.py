#! /usr/bin/env python3

import pygame, rospy, math
from pygame.locals import *
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon

background_color = (0, 0, 0)
line_color = (255, 255, 255)
line_width = 5
WIDTH, HEIGHT = 600, 600
frequency = 10
points = []

polygon_publisher = rospy.Publisher("/polygon", Polygon, queue_size=10)

def distance(point_1, point_2):
    return math.sqrt((point_1[0]-point_2[0])**2+(point_1[1]-point_2[1])**2)

if __name__ == "__main__":
    rospy.init_node("polygon")
    rate = rospy.Rate(frequency)
    pygame.init()
    logo = pygame.image.load("assets/images/ariitk.jpg")
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("Polygon")
    pygame.display.set_icon(logo)
    print("Left Click to Add a Point")
    print("Right Click to Delete the Nearest Point")
    print("Press Backspace on Keyboard to delete the Lastest Added Point")
    print("Press 'C' on Keyboard to delete all the points")
    print("Points Published on /polygon with message type as geometry_msgs/Polygon")
    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_BACKSPACE:
                    if len(points) > 0:
                        points.pop()
                if event.key == pygame.K_c:
                    points.clear()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    points.append(pygame.mouse.get_pos())
                if event.button == 3:
                    mouse_position = pygame.mouse.get_pos()
                    point_index = -1
                    width, height = screen.get_size()
                    min_distance = width+height
                    for index, point in enumerate(points):
                        current_distance = distance(point, mouse_position)
                        if current_distance < min_distance:
                            min_distance = current_distance
                            point_index = index
                    if point_index != -1:
                        points.pop(point_index)
        screen.fill(background_color)
        ros_publisher_points = Polygon()
        for draw_point_index, point in enumerate(points):
            ros_publisher_point = Point32(point[0], point[1], 0)
            ros_publisher_points.points.append(ros_publisher_point)
            pygame.draw.line(screen, line_color, point, points[(draw_point_index+1)%len(points)], line_width)
        polygon_publisher.publish(ros_publisher_points)
        pygame.display.update()