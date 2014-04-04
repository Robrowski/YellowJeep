#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math
from nav_msgs.msg import OccupancyGrid



# handy!
# roslaunch turtlebot_rviz_launchers view_navigation.launch

# Prints back map data
def printMap(data):
	print data
	global pub
	pub.publish(data)




# This is the program's main function
if __name__ == '__main__':
    
	# Change this node name to include your username
	rospy.init_node('Rob made a map reading Node', anonymous=True)


	# Subscribing to the map
	rospy.Subscriber('/map',  OccupancyGrid, printMap, queue_size=1)

	global pub
	pub = rospy.Publisher('/poop', OccupancyGrid);

	# Wait, then spin. 
	rospy.sleep(rospy.Duration(.5, 0))       
	rospy.spin()

