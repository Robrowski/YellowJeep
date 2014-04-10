#!/usr/bin/env python
import rospy
from nav_msgs.msg import GridCells #because publishers have to know
from MapHolster import *

## A class for publishing stuff for TeamYellowJeep
class YellowPublisher:    
	# initializes all publishers and a MapHolster instance
	def __init__(self):
		self.frontPub = rospy.Publisher('/frontier', GridCells)
		self.expandedPub = rospy.Publisher('/expanded', GridCells)    
		self.pathPub = rospy.Publisher('/path', GridCells)
		self.wpPub = rospy.Publisher('/wayPoints', GridCells)
		self.holster = MapHolster() # Needed to convert points to cells

	# Send points to the frontier topic
	def sendToFrontier(self,  points):
		self.frontPub.publish(self.holster.makeMessage(points))

	# Send points to the path topic
	def sendToPath(self,  points):
		self.pathPub.publish(self.holster.makeMessage(points))

	# Send points to the expanded topic
	def sendToExpanded(self,  points):
		self.expandedPub.publish(self.holster.makeMessage(points))

	# Send points to the expanded topic
	def sendToWaypoints(self, points):
		self.wpPub.publish(self.holster.makeMessage(points))
	
	# Clears each topic
	def clearTopics(self):
		self.sendToWaypoints( [])
		self.sendToExpanded( [])
		self.sendToPath( [])
		self.sendToFrontier( [])