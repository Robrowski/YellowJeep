#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math

from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *
from lab3.srv import *
from geometry_msgs.msg import PoseStamped
from mapUtils import *
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

######################################################
# Sends service request to get A* going
def gotGoal(msg):	
	global pub

	pub.clearTopics()
	print "Got a goal from RVIZ!!"

	rospy.wait_for_service('calculate_path')
	try:
		calculate_path = rospy.ServiceProxy('calculate_path', CalculatePath)
		#resp = calculate_path(Point(1,3,0), Point(10,10,0)) # Could send an actual point
		resp = calculate_path(None, None) # None = use RViz topics as setpoints
		
		path = resp.path	
		pub.sendToPath( path)
		
		waypoints = extractWaypoints(path)
		pub.sendToWaypoints( waypoints)	
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def gotStart(msg):
	global pub
	pub.clearTopics()

# def replan(msg):
# 	global pub, currentPosition
# 	pub.clearTopics()
# 	
# 	print "map update, time to replan!"
# 	
# 	rospy.wait_for_service('calculate_path')
# 	try:
# 		calculate_path = rospy.ServiceProxy('calculate_path', CalculatePath)
# 		#resp = calculate_path(Point(1,3,0), Point(10,10,0)) # Could send an actual point
# 		resp = calculate_path(Point(currentPosition.x,currentPosition.y,0), None) # None = use RViz topics as setpoints
# 		
# 		path = resp.path	
# 		pub.sendToPath(path)
# 		
# 		waypoints = extractWaypoints(path)
# 		pub.sendToWaypoints( waypoints)	
# 	except rospy.ServiceException, e:
# 		print "Service call failed: %s"%e
	
def read_odometry(msg):
	global currentPosition
	
# 	flag = 1 # Used for other functions to know when first odometry reading was executed
	
	currentPosition = msg.pose.pose.position
	
# Main function that sets up a subscriber that waits for RViz to publish goals
if __name__ == '__main__':
	rospy.init_node('Path_Planner', anonymous=True)
	global pub
	global currentPosition
# 	currentPosition = 
	pub = YellowPublisher('/newMap')
	
	rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1)
	rospy.Subscriber('/map', OccupancyGrid, gotGoal, queue_size=None)
	rospy.Subscriber('/move_base_simple/yellowgoal',  PoseStamped, gotGoal, queue_size=None)
	rospy.Subscriber('/yellowinitialpose',  PoseWithCovarianceStamped, gotStart, queue_size=None)

	print "Ready for RVIZ to set start and goal"
	rospy.spin()
	


