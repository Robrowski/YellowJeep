#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math, time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from std_msgs.msg import Header
from geometry_msgs.msg import Point



#####################################################
##########   Grid Cell Utilities   ##################
#####################################################
# makes a grid cell with the x/y indices from the map
# can also take a Point object
def newGridCell(x, y = None):
	global gridOrigin
	global gridResolution
	
	#X is actually a Point object in this case
	if y == None: 
		y = x.y
		x = x.x

	gx = gridOrigin.x + x*gridResolution	
	gy = gridOrigin.y + y*gridResolution
	return Point(gx, gy, 0)


# Takes cells as a list of Points and sends them to rviz
def makeGridCells(cells):
	global gridResolution
	newHeader = Header(1,rospy.get_rostime(),'map')
	return GridCells(newHeader, gridResolution, gridResolution, cells)

#####################################################
################   Map Utilities   ##################
#####################################################
# Reads map meta data and saves to globals
def readMapMetaData(aMap):
	global mapOrigin
	mapOrigin  = aMap.info.origin.position 

	global mapInfo
	mapInfo = aMap.info

	global gridResolution  
	gridResolution = aMap.info.resolution # assume square map

	global gridOrigin
	# Our grid has double the resolution of the map, so the /2
	# centers grid squares on the map
	gridOrigin = Point( mapOrigin.x + gridResolution/2  , mapOrigin.y + gridResolution/2  ,  0)


# returns the value at the given point
# can also take a Point object
def readMapPoint(x,  y = None):
	global currentMap
	
	#X is actually a Point object in this case
	if y == None: 
		y = x.y
		x = x.x
	
	width = currentMap.info.width	
	return currentMap.data[y*width + x]

#####################################################
###################   PUBLISHERS   ##################
#####################################################
# Send cells to the frontier topic
def sendToFrontier(cells):
	global frontPub
	frontPub.publish(makeGridCells(cells))
	

# Send cells to the path topic
def sendToPath(cells):
	global pathPub
	pathPub.publish(makeGridCells(cells))

# Send expanded to the path topic
def sendToExpanded(cells):
	global expandedPub
	expandedPub.publish(makeGridCells(cells))


	
######################################################
# Prints back map data
def mapRecieved(newMap):
	global currentMap
	currentMap = newMap

	readMapMetaData(newMap);

	
	######################################################
	# OPTIONAL ARGUMENTS RULE
	print "Point 1"
	print newGridCell(           7, 5)
 	
 	print "Point 2"
 	print newGridCell(Point(7,5,0)   )
 
	# Read map Point
	print "read 1"
	print readMapPoint(0,0)
	
	print "read 2"
	print readMapPoint(   Point(0,0,0) )
	######################################################

	#mapAnimationDemo();

	
# initializes frontier, expanded and path publishers
def initGridCellPublishers():
	global frontPub
	frontPub = rospy.Publisher('/frontier', GridCells)

	global expandedPub
	expandedPub = rospy.Publisher('/expandedPub', GridCells)

	global pathPub
	pathPub = rospy.Publisher('/pathPub', GridCells)



#demo for map reading and animation
def mapAnimationDemo():
	#### EXAMPLE FOR COLORING THE MAP 
	global mapInfo
	front = []
	exp = []
	r = rospy.Rate(50)# 50hz
	for x in range(mapInfo.width):
		for y in range(mapInfo.height):
			ptVal = readMapPoint(x,y)
			if ptVal == 0:
				front += [newGridCell(x,y)]
			else:
				exp   += [newGridCell(x,y)]
			sendToExpanded(exp)
			sendToFrontier(front)
			r.sleep()





# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Path_Planner', anonymous=True)
    
    initGridCellPublishers()
   
  	# Subscribing to the map
    rospy.Subscriber('/map',  OccupancyGrid, mapRecieved, queue_size=1)
    
  
    
    # Wait, then spin. 
    rospy.sleep(rospy.Duration(.5, 0))       
    rospy.spin()

