#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Point, PoseStamped
from YellowPublisher import *
from rbe_3002_final_project.srv import *
from mapUtils import *
from nav_msgs.msg import OccupancyGrid, Odometry
from AStar 			 import  AStarException

######################################################
# Sends service request to get A* going
def gotGoal(msg):	
	global pub

	pub.clearTopics()
	print "Got a goal: Requesting a new path..."

	rospy.wait_for_service('calculate_path')
	try:
		calculate_path = rospy.ServiceProxy('calculate_path', CalculatePath)
		path = calculate_path(None, None).path # None = use RViz topics as set points
		
		# Don't send insignificant paths
		if len(path)  > 1:
			pub.sendToPath( path)
			
			waypoints = extractWaypoints(path)
			waypoints.reverse() # put in correct order
			if len(waypoints) > 1: # want to send SOMETHING
				pub.sendToWaypoints( waypoints)	

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		
	
# Main function that sets up a subscriber that waits for RViz to publish goals
if __name__ == '__main__':
	rospy.init_node('Path_Planner', anonymous=True)
	global pub
	pub = YellowPublisher('/map_yellow')
	
	rospy.Subscriber('/map_yellow', OccupancyGrid, gotGoal)
	rospy.Subscriber('/move_base_simple/yellowgoal',  PoseStamped, gotGoal)

	print "Ready for goals to be set!"
	rospy.spin()
	