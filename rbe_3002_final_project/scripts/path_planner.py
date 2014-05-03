#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Point, PoseStamped
from YellowPublisher import *
from MapHolster import *
from lab3.srv import *
from mapUtils import *
from nav_msgs.msg import OccupancyGrid, Odometry
from AStar 			 import  AStarException

######################################################
# Sends service request to get A* going
def gotGoal(msg):	
	global pub

	pub.clearTopics()
	print "Got a goal from RVIZ!!"

	rospy.wait_for_service('calculate_path')
	try:
		calculate_path = rospy.ServiceProxy('calculate_path', CalculatePath)
		resp = calculate_path(None, None) # None = use RViz topics as set points
		
		path = resp.path	
		if len(path)  > 1:
			pub.sendToPath( path)
			
			waypoints = extractWaypoints(path)
			if len(waypoints) > 1: # want to send SOMETHING
				pub.sendToWaypoints( waypoints)	
	
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		
def clearGridCells(msg):
	global pub
	pub.clearTopics()


	
# Main function that sets up a subscriber that waits for RViz to publish goals
if __name__ == '__main__':
	rospy.init_node('Path_Planner', anonymous=True)
	global pub
	pub = YellowPublisher('/map_yellow')
	
	rospy.Subscriber('/map', OccupancyGrid, gotGoal, queue_size=None)
	rospy.Subscriber('/move_base_simple/yellowgoal',  PoseStamped, gotGoal, queue_size=None)
	rospy.Subscriber('/yellowinitialpose',  PoseWithCovarianceStamped, clearGridCells, queue_size=None)

	# Call A* on a timer
	# rospy.Timer(rospy.Duration(15), gotGoal)
	
	print "Ready for RVIZ to set start and goal"
	rospy.spin()
	