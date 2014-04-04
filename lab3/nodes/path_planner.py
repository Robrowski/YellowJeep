#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math, time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from std_msgs.msg import Header
from geometry_msgs.msg import Point


# handy!
# roslaunch turtlebot_rviz_launchers view_navigation.launch


def newGridCell(x, y):
    global gridOrigin
    global gridResolution
    
    gx = gridOrigin.x + x*gridResolution
    gy = gridOrigin.y + y*gridResolution
    return Point(gx, gy, 0)


# Prints back map data
def printMap(newMap):
	global frontPub
	global currentMap
	global mapOrigin


	currentMap = newMap
	readMapMetaData(newMap);

	newGrid = makeGridCells([newGridCell( 15,30 ),newGridCell( 30 ,30 ),newGridCell( 27 , 20 ), newGridCell(0,0)])
	frontPub.publish(newGrid)
	print "poop bitchz"


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




# This is the program's main function
if __name__ == '__main__':

    # Change this node name to include your username
    rospy.init_node('Path_Planner', anonymous=True)


    # Subscribing to the map
    rospy.Subscriber('/map',  OccupancyGrid, printMap, queue_size=1)


    global frontPub
    frontPub = rospy.Publisher('/frontier', GridCells);


    # Wait, then spin. 
    rospy.sleep(rospy.Duration(.5, 0))       
    rospy.spin()

