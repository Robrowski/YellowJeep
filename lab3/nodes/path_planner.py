#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math

from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *

######################################################
# Prints back map data
def mapRecieved(newMap):
	## Default stuff
	global pub
	global holster

	## Random stuff
	# pub.sendToExpanded(holster, holster.getEightAdjacentPoints(17,17))

	#sendToPath( [Point(36,36,0)])
	# mapAnimationDemo();
	astar(Point(3,3,0),Point(11,15,0))

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
	pass

# returns the weight/cost to go frome point a to point b
# for simplicity, all weights are equal for the time being
def weightBetween(a,b):
	return 1

# given the list of parents and the start and goal points,
# return a reconstructed path going from the goal to the start
# 
# ****
# The output of the path is a list of points to go to in order 
# starting at the goal
# ****
def reconstructPath(parents,start,goal):
	global holster
	global pub
	current = goal
	path = [goal]
	flag = 1
	r = rospy.Rate(100)
	while flag:
		current = parents[current]
		path.append(current)
		pub.sendToPath(holster,path)
		print current.x, current.y
		if current == start:
			flag = 0
		r.sleep()

	return path

#start and goal are Points from gemoetry_msgs.msg
def astar(start, goal):
	global holster
	global pub

	explored = [] 		#list of explored nodes
	frontier = [start]	#list of nodes to explore

	g_score = {start:0} #dictionary for best cost to any node from start

	#dictionary of best final scores ( = with heuristic) for each node from start
	f_score = {start:g_score[start] + distance(start, goal)} 

	parents = {} #dictionary representing node to parent relationship. ex: {node: parent}
	
	r = rospy.Rate(100)# 100hz for animation

	while frontier:
		#get the lowest node from the frontier to exlore next
		current = getLowestF(frontier,f_score)
		print current.x, current.y

		#if at the goal, add the goal to the parents list and end
		if current == goal:
			parents[goal] = current
			break

		#add the node to the explored list, and remove it from frontier
		explored.append(current)
		frontier.remove(current)

		#for animating, publish on every iteration
		pub.sendToExpanded(holster,explored)
		pub.sendToFrontier(holster,frontier)

		#get all eight connected neighbors
		neighbors = holster.getEightAdjacentPoints(current)
		for neighbor in neighbors:

			temp_g = g_score[current] + weightBetween(current,neighbor)
			temp_f =  temp_g + distance(neighbor,goal)

			print neighbor.x, neighbor.y, temp_g

			if neighbor not in f_score:
				f_score[neighbor] = temp_f

			#if neighbor is already explored and not lower than the value
			# already in f_score, continue on to the next neighbor
			if (neighbor in explored) and temp_f >= f_score[neighbor]:
				# print "already in explored and is bad", neighbor.x, neighbor.y
				continue

			if (neighbor not in frontier) or temp_f < f_score[neighbor]:
				# print "not in frontier"
				parents[neighbor] = current
				g_score[neighbor] = temp_g
				f_score[neighbor] = temp_f
				if neighbor not in frontier:
					frontier.append(neighbor)
			r.sleep()

	pub.sendToExpanded(holster,explored)
	pub.sendToFrontier(holster,frontier)
	path = reconstructPath(parents,start,goal)
	pub.sendToPath(holster,path)
	return






#demo for map reading and animation
def mapAnimationDemo():
	#### EXAMPLE FOR COLORING THE MAP 
	global holster
	global pub
	front = []
	exp = []
	r = rospy.Rate(50)# 50hz
	for x in range(holster.mapInfo.width):
		for y in range(holster.mapInfo.height):
			ptVal = holster.readMapPoint(x,y)
			if ptVal == 0:
				front += [Point(x,y,0)]
			else:
				exp   += [Point(x,y,0)]
			print x,y
		pub.sendToExpanded(holster,exp)
		pub.sendToFrontier(holster,front)
		r.sleep()
	pub.sendToExpanded(holster,exp)
	pub.sendToFrontier(holster,front)	
	
# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('Path_Planner', anonymous=True)

	global pub
	pub = YellowPublisher()
	global holster
	holster = MapHolster()  # This map is ready to go forever

	# THIS IS STILL HERE TO make sure the map is initialized
  	# Subscribing to the map
	rospy.Subscriber('/map',  OccupancyGrid, mapRecieved, queue_size=1)
	  

	# Wait, then spin. 
	rospy.sleep(rospy.Duration(.5, 0))
	rospy.spin()

