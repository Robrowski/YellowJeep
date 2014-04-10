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

######################################################
# Useful if we wanted to do stuff 
def mapRecieved(newMap):
	print "Map Received" 
	global pub
	global holster


	## Random stuff
	# pub.sendToExpanded(holster, holster.getEightAdjacentPoints(17,17))

	#sendToPath( [Point(36,36,0)])
	# mapAnimationDemo();
	
# Extracts way points to simplify path	
def extractWaypoints(path):
	
	path.remove(path[0])	
		
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
	
			#print unitVec1
			#print unitVec2		
#	print "Done extracting"
	return path

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

def gotGoal(msg):	
	global holster
	global pub

	pub.clearTopics(holster)
	print "Got a goal from RVIZ!!"

	rospy.wait_for_service('calculate_path')
	try:
		calculate_path = rospy.ServiceProxy('calculate_path', CalculatePath)
		resp = calculate_path(Point(1,3,0), Point(10,10,0))
		#resp = calculate_path(None, None)
		
		path = resp.path	
		pub.sendToPath(holster, path)
		
		waypoints = extractWaypoints(path)
		pub.sendToWaypoints(holster, waypoints)	
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e




	
# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your i
	rospy.init_node('Path_Planner', anonymous=True)

	global pub
	pub = YellowPublisher()
	global holster
	holster = MapHolster()  # This map is ready to go forever


	# THIS IS STILL HERE TO make sure the map is initialized
  	# Subscribing to the map
	#rospy.Subscriber('/map',  OccupancyGrid, mapRecieved, queue_size=1)
	rospy.Subscriber('/move_base_simple/goal',  PoseStamped, gotGoal, queue_size=1)
	
	print "Ready for RVIZ to set start and goal"
	
	rospy.spin()
	


