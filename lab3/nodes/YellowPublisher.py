#!/usr/bin/env python
import rospy
from nav_msgs.msg import GridCells #because publishers have to know


## A class for publishing stuff for TeamYellowJeep
class YellowPublisher:    
        
    # initializes frontier, expanded and path publishers
    def __init__(self):
        self.frontPub = rospy.Publisher('/frontier', GridCells)
        self.expandedPub = rospy.Publisher('/expandedPub', GridCells)    
        self.pathPub = rospy.Publisher('/pathPub', GridCells)
    
        
    # Send points to the frontier topic
    def sendToFrontier(self, holster, points):
        self.frontPub.publish(holster.makeMessage(points))
        
    # Send points to the path topic
    def sendToPath(self, holster, points):
        self.pathPub.publish(holster.makeMessage(points))
    
    # Send points to the expanded topic
    def sendToExpanded(self, holster, points):
        self.expandedPub.publish(holster.makeMessage(points))
    
        
