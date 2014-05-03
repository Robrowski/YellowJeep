#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


# Makes a twist message vien angular and forward velocities
def createTwist(u,w):
	twist = Twist()
	twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0		# Sets x,y,z speed
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w	# Sets angular speed
	return twist