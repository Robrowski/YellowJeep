#!/usr/bin/env python

import rospy, tf, math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose
from mapUtils import *


# Jeep Positioning System - encapsulated usage of the TF Listener for getting 
# position and orientation of the robot in the map frame. 
class JPS:

	def __init__(self):
		# For keeping Updates on robot position
		self.tfListener = tf.TransformListener()
	
	# Reads the tf stack and returns the location of the robot on the map
	def getCurrentPosition(self):		
		(trans,rot) = self.getTFData()
		return  Point(trans[0], trans[1], 0)			
	
	# Reads the tf stack and returns the orientation of the robot on the map	
	def getCurrentOrientation(self):	
		(trans,rot) = self.getTFData()
		return  euler_from_quaternion( [rot[0],rot[1],rot[2],rot[3]])[2]

	# Reads the TF listener for datas. Returns None upon error
	# Three errors can result and are caught as exceptions here
	def getTFData(self):
		try:
			(trans,rot) = self.tfListener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			return (trans,rot)
		except tf.LookupException, e:
			print "Orientation: LookupException"
			print e
			return None,None
		except tf.ExtrapolationException, e:
			print "Orientation: ExtrapolationException"
			print e
			return None,None
		except 	tf.ConnectivityException, e:
			print "Orientation: ConnectivityException"
			print e
			return None,None

	# Calculates the desired theta from the current robot positon to "next"
	def calcDesiredTheta(self, nextPoint):
		current = self.getCurrentPosition()
		pathVector = unitVector(current,nextPoint)
		return math.atan2(pathVector.y, pathVector.x)


	#Calculates the theta to travel to goal
	# By checking the difference between desrired and current thetas, the 
	# direction may be extrapolated once the difference is scaled between
	# +/- pi
	def thetaToGoal(self, desiredOrientation):
		thetaToTravel = desiredTheta - self.getCurrentOrientation()
		
		# Scale it to be within +/- pi
		if thetaToTravel > math.pi:
			thetaToTravel -= 2*math.pi
		if thetaToTravel < -math.pi:
			thetaToTravel += 2*math.pi
		
		return thetaToTravel


