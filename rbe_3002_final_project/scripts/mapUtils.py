#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Point

# Print a point
def printPoint(pt):
	print "X: " + str(pt.x) +"  Y: " + str(pt.y) 

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

# A heuristic function that calculates the distance assuming that the 
# robot can only move straight and at 45 degree angles to 
# get to its goal when moving on an 8-connected grid.
#
# This heuristic dominates straight line distance heuristics and
# is still admissible when used on its own.
def newHeuristic(start, goal):
	# vector from start to goal
	v = Point( math.fabs(goal.x - start.x), math.fabs(goal.y - start.y), 0)
	
	# max distance travelable going at 45 degree angles to map grid
	d45 = math.sqrt(2)*min(v.x, v.y)
	
	# min distance going straight in either x or y
	dS = math.fabs(v.x - v.y)
	
	return d45 + dS

# Extracts way points to simplify a path path represented as a list of Points	
def extractWaypoints(path):
	path.remove(path[0])	# Known bug: goal is in path twice
	ptIndex = 0
	while ptIndex + 2 < len(path):
		# Check three points at a time
		unitVec1 = unitVector(path[ptIndex]  , path[ptIndex+1])
		unitVec2 = unitVector(path[ptIndex+1], path[ptIndex+2])
		 
		# If the unit vectors are identical, we can delete the middle point
		if math.atan2(unitVec1.x, unitVec1.y) == math.atan2(unitVec2.x, unitVec2.y): 
			path.remove(path[ptIndex+1])
		else: #check next set of points
			ptIndex += 1
	return path
