#!/usr/bin/env python
import rospy
from nav_msgs.msg import GridCells #because publishers have to know


## A class for publishing stuff for TeamYellowJeep
class YellowPublisher:    

	# initializes frontier, expanded and path publishers
	def __init__(self):
		self.frontPub = rospy.Publisher('/frontier', GridCells)
		self.expandedPub = rospy.Publisher('/expanded', GridCells)    
		self.pathPub = rospy.Publisher('/path', GridCells)
		self.wpPub = rospy.Publisher('/wayPoints', GridCells)

	# Send points to the frontier topic
	def sendToFrontier(self, holster, points):
		self.frontPub.publish(holster.makeMessage(points))

	# Send points to the path topic
	def sendToPath(self, holster, points):
		self.pathPub.publish(holster.makeMessage(points))

	# Send points to the expanded topic
	def sendToExpanded(self, holster, points):
		self.expandedPub.publish(holster.makeMessage(points))


	# Send points to the expanded topic
	def sendToWaypoints(self, holster, points):
		self.wpPub.publish(holster.makeMessage(points))
		
	def clearTopics(self, holster):
		self.sendToWaypoints(holster, [])
		self.sendToExpanded(holster, [])
		self.sendToPath(holster, [])
		self.sendToFrontier(holster, [])