#!/usr/bin/env python

from lab3.srv import *
import rospy, math, tf
from geometry_msgs.msg import Point
from nav_msgs.msg import *
from YellowPublisher import *
from MapHolster      import *
from AStar 			 import astar, AStarException

# Calculates a path when told to
def handleCalculatePath(h):
	global globalMapHolster, costMapHolster
		
	start = h.start
	goal = h.goal

	print "calculating a path"
	# If None's are received, that is the signal to use the 
	# start and goal cached in the MapHolster set by Rviz
	if h.start == Point(0,0,0) or  h.goal == Point(0,0,0):
		print "Using goal from MapHolster"
		start = globalMapHolster.start
		goal = globalMapHolster.goal
		
	#use the robots actual position as the start	
	if globalMapHolster.getCurrentPosition() != None: 
		print "Using robot position as start of A*"
		start = globalMapHolster.convertCellToPoint(globalMapHolster.getCurrentPosition())
		
	
	
	path = astar(start,goal, globalMapHolster,costMapHolster)
	return CalculatePathResponse(path)

# Initialize a server that only handles A* requests
if __name__ == '__main__':
	rospy.init_node('PathPlanningServer', anonymous=True)
	global globalMapHolster, costMapHolster
	globalMapHolster = MapHolster('/map_yellow') 
	costMapHolster = MapHolster('/move_base/local_costmap/costmap_yellow')
		
	# Setup up server
	rospy.Service('calculate_path', CalculatePath, handleCalculatePath);
		
	print "Ready to calculate paths!"
	rospy.spin()