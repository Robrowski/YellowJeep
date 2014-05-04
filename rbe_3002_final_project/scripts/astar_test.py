#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Point, PoseStamped
from YellowPublisher import *
from MapHolster import *
from lab3.srv import *
from mapUtils import *
from nav_msgs.msg import OccupancyGrid, Odometry
from AStar 			 import  *

######################################################
# Sends service request to get A* going
def gotGoal(msg):	
	global pub, start, globalMapHolster
	pub.clearTopics()
	print "Got a goal from RVIZ!!"
	goal = globalMapHolster.convertCellToPoint(msg.pose.position)
	print goal
	
	
	try:
		path =  astar(start, goal, globalMapHolster, globalMapHolster)
		pub.sendToPath( path)
		
		waypoints = extractWaypoints(path)
		pub.sendToWaypoints( waypoints)
	except AStarException, e:
		print "A* crashed: %s"%e
		
def gotStart(msg):
	global pub, start,globalMapHolster
	print "Got start"
	start = globalMapHolster.convertCellToPoint(msg.pose.pose.position)
	print start
	pub.clearTopics()

	
# Main function that sets up a subscriber that waits for RViz to publish goals
if __name__ == '__main__':
	rospy.init_node('Path_Planner', anonymous=True)
	global pub, start	
	global globalMapHolster
	globalMapHolster = MapHolster('/map_yellow') 

	start = Point( 55,55,0)

	pub = YellowPublisher('/map_yellow')
	rospy.Subscriber('/move_base_simple/yellowgoal',  PoseStamped, gotGoal)
	rospy.Subscriber('/yellowinitialpose',  PoseWithCovarianceStamped, gotStart)

	
	print "Ready for RVIZ to set start and goal"
	rospy.spin()
	