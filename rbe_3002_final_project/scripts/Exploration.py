#!/usr/bin/env python

import rospy, math, random, Queue as q
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import Header
from YellowPublisher import *
from MapHolster import *
from lab3.srv import *
from mapUtils import *
from nav_msgs.msg import OccupancyGrid, Odometry
from AStar 			 import  AStarException
from kobuki_msgs.msg import BumperEvent

# Takes a cluster (list) of points and returns the centroid
def calculateCentroid(cluster):
	global holster,obstacleTol
	xSum = 0
	ySum = 0
	numPoints = len(cluster)
	
	for pt in cluster:
		xSum += pt.x
		ySum += pt.y
	
	centroid =  Point( math.trunc(xSum/numPoints) , math.trunc(ySum/numPoints),0)
	
	# If the centroid is in an unknown area...
	centroidValue = holster.readMapPoint(centroid)
	if centroidValue == -1 or centroidValue > obstacleTol:
		possibleNewPoints = holster.getEightAdjacentPoints(centroid,y = None, tol = obstacleTol)
		if len(possibleNewPoints) > 1:
			return possibleNewPoints[0] # Just give the first one
		
		return cluster[0] # Else give the first		
	
	return centroid


def centroidValue(cluster, centroid):
	global robotPosition
	alpha = .8
	beta  = 8
	return  (alpha * distance(robotPosition, centroid) ) + beta* len(cluster)


# Checks 8 points adjacent to center for unknowns. 
# Returns true if unknown is found
# This is how we define our frontier points
def isFrontier(center):
	global holster
	return  len(holster.getEightAdjacentPoints(center,y = None, tol = 101)) < 8

def hasOpenNeighbors(center):
	global holster
	return  len(holster.getEightAdjacentPoints(center,y = None, tol = obstacleTol)) > 1


# Callback function to find unknowns
def findUnknowns(aPoseWithCovarianceStamped):
	global pub,holster, yellowPub, obstacleTol, robotPosition,sortedGoals,nextGoal,prevGoal
	
	if aPoseWithCovarianceStamped == "notused":
		try:
			robotPosition = holster.convertCellToPoint(holster.getCurrentPosition())
		except TypeError:
			print "TF didn't work for Exploration node"
			return "error"
	else:
		robotPosition = holster.convertCellToPoint(aPoseWithCovarianceStamped.pose.pose.position)

	
	print "Searching for unknowns from: " 
	printPoint(robotPosition)
	
############################################
###### Tuning Variables
	clusterSize = 6
	obstacleTol = 10
	
############################################
###### Data structures	
	goals = []
	mapQ   = []
	
	mapClosed   = []
	frontClosed = []
	
	newFrontiers = []
############################################
######## Initialization
	mapQ.append(robotPosition)

############################################
######## Main loop
	# While there is shit to check
	while mapQ:
		
		# Handle queing shit
		current = mapQ.pop(0)	
		# Skip if closed already
		if current in mapClosed:
			continue
		
		# Checking out Frontier
		if isFrontier(current):
			frontQ = []
			cluster = []
			frontQ.append(current)
			
			while frontQ:
				pt = frontQ.pop(0)
				if pt in mapClosed and pt in frontClosed:
					continue
				if isFrontier(pt): #Line 17 from pseudo code
					cluster.append(pt)
					mapClosed.append(pt) #line 25 from pseudo code NOT WHERE SUPPOSED TO BE
					adjacentPoints = holster.getEightAdjacentPoints(pt, None, tol = obstacleTol)
					for pt in adjacentPoints:
						if pt not in frontClosed and pt not in mapClosed and pt not in frontQ:
							frontQ.append(pt)
							
				frontClosed.append(pt)		
			if len(cluster) > clusterSize:
				newFrontiers.append(cluster)
				goals.append(calculateCentroid(cluster))
				yellowPub.sendToGoals( goals)
#		yellowPub.sendToFrontier(frontClosed)  # Animation
#		yellowPub.sendToExpanded(mapClosed) # Animation
	
		# Adding points to queue
		adjacentPoints = holster.getEightAdjacentPoints(current, None, tol = obstacleTol)
		for pt in adjacentPoints:
			if pt not in mapClosed and pt not in mapQ and hasOpenNeighbors:
				mapQ.append(pt)
				
		mapClosed.append(current)

###################
### Show all important frontiers
	goodFrontiers = []
	for f in newFrontiers:
		goodFrontiers += f
	
	#yellowPub.sendToFrontier(goodFrontiers)


############################
#### Sort Goals
	goalPriorityQ  =  q.PriorityQueue(len(goals))
	
	
	for i in range(len(goals)):
		goalPriorityQ.put([ 1/centroidValue(newFrontiers[i], goals[i]) , goals[i] ])
		
	#print "Sorted Goals:  "
	sortedGoals = []
	while goalPriorityQ.qsize() > 0:
		point = goalPriorityQ.get()[1]
		sortedGoals.append(point)

	nextGoal = sortedGoals[0]
	prevGoal = sortedGoals[0]
	yellowPub.sendToGoals( sortedGoals)
	return sortedGoals

# sends a goal
def sendGoal(notUsed):
	global sortedGoals, pub, holster, nextGoal
	
	if len(sortedGoals) > 0:
		ptToSend = holster.convertPointsToCells([nextGoal])[0]
		pub.publish(  PoseStamped(Header(1,rospy.get_rostime(),'map'), Pose(ptToSend, None)))
		print "Goal Sent!!!!!!!!!!!!!"

# Picks a new random goal on a timer
def newRandomGoal(notUsed):
	global sortedGoals, nextGoal, prevGoal, goalIndex

	print "Generating random goal"
	print "prevGoal: " + str(prevGoal.x) + str(prevGoal.y)
	prevGoal = nextGoal
	while nextGoal == prevGoal and len(sortedGoals) > 1:
		nextGoal = random.choice(sortedGoals)

	print "Next Goal is: " + str(nextGoal.x) + str(nextGoal.y)


#I think this needs be looked at when i'm not drunk....
def atGoal(currentGoal):
	global holster
	robotPosition = holster.convertCellToPoint(holster.getCurrentPosition())

	if robotPosition == currentGoal:
		return True
	else:
		return False


# if the bumper is pressed, pick a new random goal #YOLO
def readBumper(msg):
	if (msg.state == 1):
		print "hump"
		newRandomGoal("notused")
		sendGoal("notused")

#Main function that sets up a subscriber that waits for RViz to publish goals
if __name__ == '__main__':
	rospy.init_node('Path_Planner', anonymous=True)
	global pub,holster, yellowPub, sortedGoals, nextGoal, prevGoal, goalIndex
	sortedGoals = []
	holster = MapHolster() # the map to explore
	yellowPub = YellowPublisher('/map_yellow')

	goalIndex = -1
	
	rospy.Subscriber('/yellowinitialpose',  PoseWithCovarianceStamped, findUnknowns)
	print "Ready to explore the map!"
	
	
	if findUnknowns("notused") != "error":
		nextGoal = sortedGoals[0]
		prevGoal = sortedGoals[0]
		newRandomGoal("notused")
		
		# Call findUnknowns on a 45 second timer
		rospy.Timer(rospy.Duration(45), findUnknowns)
		rospy.Timer(rospy.Duration(120), newRandomGoal)
	
		bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) 
	
	pub = rospy.Publisher('/move_base_simple/yellowgoal',  PoseStamped,latch=True)
	rospy.Timer(rospy.Duration(30), sendGoal)
		
		

	rospy.spin()
	