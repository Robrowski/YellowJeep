#!/usr/bin/env python

import rospy, tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose


# Jeep Positioning System
class JPS:

	def __init__(self):
		# For keeping Updates on robot position
		self.tfListener = tf.TransformListener()
		self.hasMap = False
	
	# Reads the tf stack and returns the location of the robot on the map
	def getCurrentPosition(self):		
		try:
			(trans,rot) = self.tfListener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			return  Point(trans[0], trans[1], 0)			
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "TF FAIL"
			return None
	
	# Reads the tf stack and returns the orientation of the robot on the map	
	def getCurrentOrientation(self):	
		try:
			(trans,rot) = self.tfListener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			return  euler_from_quaternion( [rot[0],rot[1],rot[2],rot[3]])[2]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "TF FAIL"
			return None	
			