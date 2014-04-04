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

# Prints back map data
def printMap(newMap):
    global frontPub
    global currentMap
    global mapOrigin
    global gridResolution
    currentMap = newMap
    mapOrigin  = newmap.info.origin.position   
    gridResolution = newMap.info.resolution/2 # assume square map
    
    # Our grid has double the resolution of the map, so the /2
    # centers grid squares on the map
    gridOrigin = Point( mapOrigin.x + gridResolution/2  , mapOrigin.y + gridResolution/2  ,  0);
   
    
    x = mapOrigin.x + gridResolution/2
    y = mapOrigin.y + gridResolution/2
    

    newGrid = GridCells(Header(1,rospy.get_rostime(),'map'), gridResolution, gridResolution, [Point(x,y,0)])

    frontPub.publish(newGrid)
    print "poop bitchz"






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

