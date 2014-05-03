#!/usr/bin/env python
import rospy, math
from JPS import *
from MapHolster import *
from geometry_msgs.msg import Twist, Point
from mapUtils,driveUtils.py import *
from nav_msgs.msg import  GridCells

###################################
# This node is for following paths published on the /waypoints topic
#
# When not following a path, the default behavior is to rotate
###################################

# Handler for recieving way points		
def gotWaypoints(waypointMsg):
	global waypoints
	waypoints = waypointMsg.cells

# Sends the given values in a Twist to the topic
def sendTwist(forwardVelocity, angularVelocity):
    global cmdPub
    cmdPub.publish(createTwist(forwardVelocity,angularVelocity))



# Rotates the robot using gobal orientation measurements
def rotate(desiredTheta):
	global  robotPose, turningTolerance, rotationSpeed
	rotateUpdate = rospy.Rate(20)
	
	# Rotate until desired theta lines up with current orientation
	while math.fabs(desiredTheta - robotPose.getCurrentOrientation()) > turningTolerance and not rospy.is_shutdown():
		direction = math.copysign(1, robotPose.thetaToGoal(desiredTheta))
		sendTwist(0,rotationSpeed*direction)
		rotateUpdate.sleep()



if __name__ == '__main__':
	rospy.init_node('pathFollow', anonymous=True)
	
	global cmdPub, waypoints, holster, robotPose
	cmdPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
	waypoints = []
	holster = MapHolster('/map_yellow')
	robotPose = JPS()
	
	rospy.Subscriber('/wayPoints', GridCells, gotWaypoints)

	print "Ready to follow a path"


	# TUNING Constants
	global    turningTolerance, rotationSpeed
	forwardSpeed = 0.22
	rotationSpeed = 0.7
	turningTolerance = math.pi/10    # radians
	distanceToGoalTolerance = 0.08   #  cm
	directionUpdateRate = 2
	pathCorrectionSpeed = 1.5

	r = rospy.Rate(directionUpdateRate)
	while  not rospy.is_shutdown():
		try: 
			# Get next waypoint. wapoints[0] = current position
			next = waypoints[1] 

			#rotate until at desired theta within tolerance
			rotate(robotPose.calcDesiredTheta(next)) 
				
			# Send robot forward if not at the next waypoint 
			# with some angular path correction to form an arc
			angularPathCorrection = pathCorrectionSpeed*robotPose.thetaToGoal(robotPose.calcDesiredTheta(next))
			if distance(robotPose.getCurrentPosition(), next) > distanceToGoalTolerance:
				sendTwist(forwardSpeed,angularPathCorrection)
			else:
				sendTwist(0,0) #stop at waypoint
				waypoints.pop(0)
				print "Arrived at a way point"
			r.sleep()
	
		# Exception caught because waypoints list often has race conditions or empties
		except  IndexError:	
			sendTwist(0,0.3)	# Send a rotate 
			r.sleep()				