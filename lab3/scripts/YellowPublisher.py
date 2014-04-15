#!/usr/bin/env python
import rospy
from nav_msgs.msg import GridCells 
from MapHolster import *

## A class for publishing stuff for TeamYellowJeep
class YellowPublisher:    
	# initializes all publishers and a MapHolster instance
	def __init__(self, mapTopicName = '/map'):
		self.frontPub = rospy.Publisher('/frontier', GridCells, latch=True)
		self.expandedPub = rospy.Publisher('/expanded', GridCells, latch=True)   
		self.pathPub = rospy.Publisher('/path', GridCells, latch=True)
		self.wpPub = rospy.Publisher('/wayPoints', GridCells, latch=True)
		self.holster = MapHolster(mapTopicName) # Needed to convert points to cells
		rospy.sleep(rospy.Duration(1.1, 0)) # need to be damn sure teh holster has a map

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