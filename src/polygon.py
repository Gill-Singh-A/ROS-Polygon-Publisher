#! /usr/bin/env python3

import pygame, rospy
from pygame.locals import *
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon

WIDTH, HEIGHT = 600, 600
frequency = 10
points = []

polygon_publisher = rospy.Publisher("/polygon", Polygon, queue_size=10)

if __name__ == "__main__":
    rospy.init_node("polygon")
    rate = rospy.Rate(frequency)
    pygame.init()
    logo = pygame.image.load("assets/images/ariitk.jpg")
    pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("Polygon")
    pygame.display.set_icon(logo)
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