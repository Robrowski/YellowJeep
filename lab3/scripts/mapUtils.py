#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Point

# Calculates the unit vector connecting two points
def unitVector(origin, there):
	magnitude = distance(origin, there)
	x = (there.x - origin.x)/magnitude
	y = (there.y - origin.y)/magnitude
	return Point(x,y,0)

# Takes the distance between two Points
def distance(aPoint, bPoint):
	sq1 = math.pow(aPoint.x -  bPoint.x, 2 )
	sq2 = math.pow(aPoint.y - bPoint.y, 2)
	return math.sqrt(sq1 + sq2)

# A heuristic that calculates the distance assuming that the 
# robot can only move straight and at 45 degree angles to 
# get to its goal
def newHeuristic(start, goal):
	# vector from start to goal
	v = Point( math.fabs(goal.x - start.x), math.fabs(goal.y - start.y), 0)
	
	# max distance going at 45 degree angles
	d45 = math.sqrt(2)*min(v.x, v.y)
	
	# min distance going straight
	dS = math.fabs(v.x - v.y)
	
	return d45 + dS

# Extracts way points to simplify path	
def extractWaypoints(path):
	path.remove(path[0])	# Known bug: goal is in path twice
	i = 0
	while i + 2 < len(path):
		
		# Check three points at a time
		unitVec1 = unitVector(path[i]  , path[i+1])
		unitVec2 = unitVector(path[i+1], path[i+2])
		 
		# If the unit vectors are identical, we can delete the middle point
		if math.atan2(unitVec1.x, unitVec1.y) == math.atan2(unitVec2.x, unitVec2.y): 
			path.remove(path[i+1])
		else: #check next set of points
		#	print "Failed to remove: " + str(i)
			i += 1
	
	return path
