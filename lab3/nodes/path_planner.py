#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math, time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from std_msgs.msg import Header
from geometry_msgs.msg import Point



# makes a grid cell with the x/y indices from the map
def newGridCell(x, y):
    global gridOrigin
    global gridResolution
    
    gx = gridOrigin.x + x*gridResolution
    gy = gridOrigin.y + y*gridResolution
    return Point(gx, gy, 0)


# Takes cells as a list of Points and sends them to rviz
def makeGridCells(cells):
	global gridResolution
	newHeader = Header(1,rospy.get_rostime(),'map')
	return GridCells(newHeader, gridResolution, gridResolution, cells)


# Reads map meta data and saves to globals
def readMapMetaData(aMap):
	global mapOrigin
	mapOrigin  = aMap.info.origin.position 

	global gridResolution  
	gridResolution = aMap.info.resolution # assume square map

	global gridOrigin
	# Our grid has double the resolution of the map, so the /2
	# centers grid squares on the map
	gridOrigin = Point( mapOrigin.x + gridResolution/2  , mapOrigin.y + gridResolution/2  ,  0)


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





	a = []
	b = []
	c = []
	for x in range(0, 37, 3):
		for y in range(0,37, 1):
			a += [newGridCell(x,y)]
			b += [newGridCell(x+1,y+2)]
			c += [newGridCell(x+2,y+2)]

	sendToPath(a)
	sendToFrontier( b )
	sendToExpanded(c)
	print "poop bitchz"


# initializes frontier, expanded and path publishers
def initGridCellPublishers():
 	global frontPub
    frontPub = rospy.Publisher('/frontier', GridCells);
    
    global expandedPub
    expandedPub = rospy.Publisher('/expandedPub', GridCells);
    
    global pathPub
    pathPub = rospy.Publisher('/pathPub', GridCells);




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

