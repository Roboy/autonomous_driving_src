#!/usr/bin/python

import rospy
import pygame
from math import pi
import sys

from robike.msg import Controls


commands = [pygame.K_LEFT,
        pygame.K_RIGHT,
        pygame.K_UP]

VELOCITY = 0.3 # m/s
STEERING = pi/4

def get_commands():
    if any(map(lambda e: e.type == pygame.QUIT, pygame.event.get())):
        pygame.quit()
        sys.exit()
    is_pressed = pygame.key.get_pressed()
    return filter(lambda cmd: is_pressed[cmd], commands)


def teleop():
    pygame.init()
    pygame.display.set_mode((640, 480))
    rospy.init_node('teleop', anonymous=True)
    pub = rospy.Publisher('robike/controls', Controls, queue_size=0)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        cmds = get_commands()
        controls = Controls()
        if pygame.K_UP in cmds:
            controls.vel = VELOCITY
        if pygame.K_LEFT in cmds and pygame.K_RIGHT not in cmds:
            controls.steering = -STEERING
        if pygame.K_LEFT not in cmds and pygame.K_RIGHT in cmds:
            controls.steering = STEERING
        pub.publish(controls)
        rate.sleep()


if __name__ == '__main__':
    teleop()
