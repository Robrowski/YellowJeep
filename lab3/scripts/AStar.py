#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math

from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *
from mapUtils import *


#A* Exception - for when A* needs to stop
class AStarException(Exception):
	def __init__(self, value = "A*'s goal changed..."):
		self.value = value
	
	def __str__(self):
		return repr(self.value)



######################################################
# A-Star Helper Functions
######################################################
# given the current frontier and a dictionary of all 
# f_scores, return the lowest scoring node on the frontier
def getLowestF(frontier,f_score):
	values = {}
	for point in frontier:
		if point in f_score:
			values[point] = f_score[point]
	return min(values,key=values.get)

# consider implementing a priority queue for the frontier, to
# avoid going back and forth from frontier list to f_score
# dictionary back to a point, like currently used in
# getLowestF()
def frontierPriorityPush():
	pass #TODO

# returns the weight/cost to go from point a to point b
# for simplicity, all weights are equal for the time being 
def weightBetween(a,b):
	return distance(a,b)


# Calculates the h(n) between two points
# Two holsters are given to get map data from each
def calcHeuristic(current, goal,globalMapHolster,costMapHolster):
	PID = 1 # H should be between 0 and 1 to be admissible
	PIDprime = .9
		
	currentInWorldCoords = costMapHolster.convertCellToPoint(  globalMapHolster.newGridCell(current))
	costOfCurrent = costMapHolster.readMapPoint(currentInWorldCoords)
	if costOfCurrent == -1:
		costOfCurrent = 0 # Set to zero to not make h(n) lower
	
	# newHeuristic is an optimized distance function to account for constraints
# 	return distance(current,goal)
	return PID*newHeuristic(current,goal)  + PIDprime*costOfCurrent


# given the list of parents and the start and goal points,
# return a reconstructed path going from the goal to the start
# 
# ****
# The output of the path is a list of points to go to in order 
# starting at the goal
# ****
def reconstructPath(parents,start,goal):
	current = goal
	path = [goal]

	while current != start:
		current = parents[current]
		path.append(current)
		
	return path

#start and goal are Points from gemoetry_msgs.msg
def astar(start, goal, holster, costMap):
	pub = YellowPublisher('/map_yellow')
	
	explored = [] 		#list of explored nodes
	frontier = [start]	#list of nodes to explore

	g_score = {start:0} #dictionary for best cost to any node from start

	#dictionary of best final scores ( = with heuristic) for each node from start
	f_score = {start:(g_score[start] + calcHeuristic(start, goal,holster,costMap))} 

	parents = {} #dictionary representing node to parent relationship. ex: {node: parent}

	while frontier:
		
		# Raise an exception if the goal changes mid calculation
		if goal != holster.goal:
			raise AStarException
		
		#get the lowest node from the frontier to explore next
		current = getLowestF(frontier,f_score)

		#if at the goal, add the goal to the parents list and end
		if current == goal:
			frontier.remove(goal)
			parents[goal] = current  #this does result in an extra goal in parents..
			break

		#add the node to the explored list, and remove it from frontier
		explored.append(current)
		frontier.remove(current)

		#for animating, publish on every iteration
		pub.sendToFrontier(frontier) # only need to see frontier to understand progress

		##########################################################
		### THE LAST VALUE PUT INTO THIS FUNCTION DETERMINES HOW 
		### CRAPPY A PATH A* MIGHT TAKE
		##########################################################
		neighbors = holster.getEightAdjacentPoints(current.x, current.y, 65)
		for neighbor in neighbors:
			if (neighbor in explored): 
				continue
			
			better = False
			temp_g = g_score[current] + weightBetween(current,neighbor)
			temp_f = temp_g + calcHeuristic(neighbor,goal,holster,costMap) 

##########################################################
			## Add to lists if not added
			if neighbor not in frontier:
				frontier.append(neighbor)
			
			if neighbor not in parents:
				parents[neighbor] = current
			
			if neighbor not in g_score:
				g_score[neighbor] = temp_g
			
			if neighbor not in f_score:
				f_score[neighbor] = temp_f

##########################################################
			## updates
			# update parents
			if (temp_g < g_score[neighbor]):
				print "poop A*"
				parents[neighbor] = current
				
				
			g_score[neighbor] = min(temp_g, g_score[neighbor])
			f_score[neighbor] = min(temp_f, f_score[neighbor])
		#	print g_score[neighbor] 
		#	print f_score[neighbor] 


#	print "Nodes Expanded by A*: " + str( len(explored))
	pub.sendToFrontier(frontier)
	path = reconstructPath(parents,start,goal)
	return path