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
	pub.sendToExpanded(holster, holster.getEightAdjacentPoints(17,17))

	#sendToPath( [Point(36,36,0)])
	mapAnimationDemo();


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

