#!/usr/bin/python3

# NAME: turtle_controller.py
# PURPOSE: randomly moves the turtle around the screen
# AUTHOR: Emma Bethel


import rospy
import numpy as np
import math
import random

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

COLLISION_RADIUS = 0.5
SPEED = 5
HORIZONTAL_BOUNDARIES = (0, 11)
VERTICAL_BOUNDARIES = (0, 11)
CENTER = (5.5, 5.5)

class TurtleController:

    def __init__(self):
        self.current_coords = np.array(CENTER)
        self.rate = rospy.Rate(10)

        # initializing publisher for sending turtle velocity commands
        self.vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

        # initializing subscriber for receiving and storing turtle position messages
        self.pose_sub = rospy.Subscriber('turtle1/pose', Pose, self.update_current_coords)

        # setting random starting velocity
        self.current_vel = self.pick_random_vel()

        # loop forever (until node is shut down)
        while not rospy.is_shutdown():
            # detecting and handling wall collisions
            self.handle_collisions()

            # sending velocity command
            #   3 possible components of velocity message (as per turtlesim 
            #   documentation):
            #       linear.x - forward (direction turtle is facing) velocity
            #       linear.y - strafe (perpendicular to direction turtle is 
            #                  facing) velocity
            #       anglular.z - angular velocity
            msg = Twist()
            msg.linear.x = self.current_vel[0]
            msg.linear.y = self.current_vel[1]
            self.vel_pub.publish(msg)

            # wait 10 miliseconds
            self.rate.sleep()


    # PURPOSE: randomly generates a velocity vector for turtle
    # PARAMETERS: N/A
    # RETURNS: the vector, as numpy array
    @staticmethod
    def pick_random_vel():
        x = random.uniform(-SPEED, SPEED)
        y = random.choice([-1, 1]) * math.sqrt(math.pow(SPEED, 2) - math.pow(x, 2))

        return np.array((x, y))
    
    # PURPOSE: parses and stores turtle coordinate messages
    # PARAMETERS: msg - message containing the turtle's current coordinates
    # RETURNS: N/A
    def update_current_coords(self, msg):
        self.current_coords = np.array((msg.x, msg.y))

    # PURPOSE: detects collisions with walls and updates direction accordingly
    # PARAMETERS: N/A
    # RETURNS: N/A
    def handle_collisions(self):
        # if vertical collision, reverse vertical component of velocity
        if self.hit_top() or self.hit_bottom():
            self.current_vel[1] *= -1
        # if horizontal collision, reverse horizontal component of velocity
        if self.hit_left() or self.hit_right():
            self.current_vel[0] *= -1


    # PURPOSE: checks whether turtle has hit top of screen
    # PARAMETERS: N/A
    # RETURNS: true if collision has occurred, false otherwise
    def hit_top(self) -> bool:
        return self.current_coords[1] + COLLISION_RADIUS >= VERTICAL_BOUNDARIES[1] and SPEED * self.current_vel[1] > 0
    
    # PURPOSE: checks whether turtle has hit bottom of screen
    # PARAMETERS: N/A
    # RETURNS: true if collision has occurred, false otherwise
    def hit_bottom(self) -> bool:
        return self.current_coords[1] - COLLISION_RADIUS <= VERTICAL_BOUNDARIES[0] and SPEED * self.current_vel[1] < 0

    # PURPOSE: checks whether turtle has hit left edge of screen
    # PARAMETERS: N/A
    # RETURNS: true if collision has occurred, false otherwise
    def hit_left(self) -> bool:
        return self.current_coords[0] - COLLISION_RADIUS <= HORIZONTAL_BOUNDARIES[0] and SPEED * self.current_vel[0] < 0

    # PURPOSE: checks whether turtle has hit right edge of screen
    # PARAMETERS: N/A
    # RETURNS: true if collision has occurred, false otherwise
    def hit_right(self) -> bool:
        return self.current_coords[0] + COLLISION_RADIUS >= HORIZONTAL_BOUNDARIES[1] and SPEED * self.current_vel[0] > 0


if __name__=="__main__":
    rospy.init_node('turtle_controller')
    
    TurtleController()
