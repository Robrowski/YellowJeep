#!/usr/bin/env python

#Purpose: Extra Credit (+15 pts) 
# On the real robot, you will need to constantly rerun A* as the robot learns more about the world. Design 
# your code to continuously run as a dedicated path-planning node. It should provide a service which 
# takes in a start and end position, and returns a path (or an error if no path exists).
from lab3.srv import *
import rospy, math
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from YellowPublisher import *
from MapHolster      import *
from AStar 			 import astar


######################################################
#### TO USE THIS SERVER
#	rospy.wait_for_service('calculate_path')
#	try:
#		calculate_path = rospy.ServiceProxy('calculate_path', CalculatePath)
#		print calculate_path(Point(1,3,0), Point(1,1,0))
#	except rospy.ServiceException, e:
#		print "Service call failed: %s"%e
		

######################################################
def handleCalculatePath(h):
	global holster
		
	start = h.start
	goal = h.goal


	print "start from client" + str(h.start)
	print "start from Rviz " + str(holster.start)
	print "start from currentPose" + str(holster.convertCellToPoint(currentPosition))
	print currentPosition


	print "calculating a path"
	# If None's are received, that is the signal to use the 
	# start and goal cached in the MapHolster set by Rviz
	if h.start == Point(0,0,0) or  h.goal == Point(0,0,0):
		print "Using goal from MapHolster"
		start = holster.start
		goal = holster.goal
		

	else: # use the given start and goals
		pass
	
	global currentPosition
	if flag == 2:
		print "using robot position as start of A*"
		start = holster.convertCellToPoint(currentPosition)
		print start
######################################################	
	## INSERT A* HERE
	path = astar(start,goal, holster)
	
	
	
	# path = resultOfA*
######################################################	
	# # Dummy path
	# x = h.start.x
	# y = h.start.y
	# path = [Point(x+1,y+0,0),Point(x+1,y+1,0),Point(x+1,y+2,0),Point(x+1,y+3,0),Point(x+2,y+3,0),Point(x+3,y+3,0)]
	
#####################################################
	# send path back
	return CalculatePathResponse(path)

def gotOdom(msg):
	global  currentPosition, flag
	flag = 1
	currentPosition = msg.pose.pose.position
	#print currentPosition
	
if __name__ == '__main__':
	rospy.init_node('PathPlanningServer', anonymous=True)
    # Need a reference to the holster so that the map is ready
	global holster, flag
	holster = MapHolster('/map') 
	
	
	# Setup up server
	s = rospy.Service('calculate_path', CalculatePath, handleCalculatePath);
	rospy.Subscriber('/odom', Odometry, gotOdom, queue_size=1) 
	flag = 0

	print "Ready to calculate paths!"
	
	rospy.spin()
