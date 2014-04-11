#!/usr/bin/env python
import rospy, math

from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *
from mapUtils import *



def followPath(waypoints):
	waypoints.reverse()
	initialTheta = getTheta()
	for i in range(waypoints.length() - 1):
		current = waypoints[i]
		next = waypoints[i+1]
		unitVector = unitVector(current,next)
		desiredTheta = math.atan2(unitVector.x, unitVector.y)
		currentTheta = getTheta()

		rotate(desiredTheta - currentTheta)
		driveStraight(distance(current,next))

if __name__ == '__main__':
    
    # Change this node name to include your username
    rospy.init_node('pathFollow', anonymous=True)

	# Wait, then spin. Exectute trajectory activated by bumper events

	wayPoints = [Point(0,0,0), Point(2,3,0), Point(7,8,0),  Point(6,6,0)]
	followPath(wayPoints)
    rospy.sleep(rospy.Duration(.5, 0))       
    rospy.spin()




