#!/usr/bin/env python

#Purpose: Extra Credit (+15 pts) 
# On the real robot, you will need to constantly rerun A* as the robot learns more about the world. Design 
# your code to continuously run as a dedicated path-planning node. It should provide a service which 
# takes in a start and end position, and returns a path (or an error if no path exists).
from lab3.srv import *
import rospy, math, tf
from geometry_msgs.msg import Point
from nav_msgs.msg import *
from YellowPublisher import *
from MapHolster      import *
from AStar 			 import astar


def handleCalculatePath(h):
	global holster
	global currentPosition, pub
		
	start = h.start
	goal = h.goal

	print "calculating a path"
	# If None's are received, that is the signal to use the 
	# start and goal cached in the MapHolster set by Rviz
	if h.start == Point(0,0,0) or  h.goal == Point(0,0,0):
		print "Using goal from MapHolster"
		start = holster.start
		goal = holster.goal
		
	else: # use the given start and goals
		pass
		
	if holster.getCurrentPosition() != None:
		print "using robot position as start of A*"
		start = holster.convertCellToPoint(holster.getCurrentPosition())
		print start
			
		newHeader = Header(1,rospy.get_rostime(),'map')
		p = rospy.Publisher('/poop', GridCells,latch=True)
		p.publish(GridCells(newHeader, .2,.2, [holster.getCurrentPosition()]))
		
######################################################	
	## INSERT A* HERE
	path = astar(start,goal, holster,costMap)
	
	
	return CalculatePathResponse(path)

	
if __name__ == '__main__':
	rospy.init_node('PathPlanningServer', anonymous=True)
    # Need a reference to the holster so that the map is ready
	global holster, flag, pub,currentPosition,costMap
	holster = MapHolster('/map_yellow') 
	pub = YellowPublisher('/map_yellow')
	costMap = MapHolster('/move_base/local_costmap/costmap_yellow')
	
	
	
	# Setup up server
	rospy.Service('calculate_path', CalculatePath, handleCalculatePath);
		
	currentPosition = Point(0,0,0)
	flag = 0

	print holster.getCurrentOrientation()
	print "Ready to calculate paths!"
	
	rospy.spin()
