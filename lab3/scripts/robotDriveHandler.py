#!/usr/bin/env python
import rospy, math
from JPS import *
from MapHolster import *
from geometry_msgs.msg import Twist, Point
from mapUtils import *
from nav_msgs.msg import  GridCells

# Handler for recieving way points		
def gotWaypoints(waypointMsg):
	global waypoints
	waypoints = waypointMsg.cells
	waypoints.reverse()
	print "Got Waypoints in driver"

# Makes a twist message vien angular and forward velocities
def createTwist(u,w):
	twist = Twist()
	twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0		# Sets x,y,z speed
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w	# Sets angular speed
	return twist

# Sends the given values in a Twist to the topic
def sendTwist(forwardVelocity, angularVelocity):
    global cmdPub
    cmdPub.publish(createTwist(forwardVelocity,angularVelocity))

# Calculates the desired theta from the current robot positon to "next"
def calcDesiredTheta(next):
	global   robotPose
	current = robotPose.getCurrentPosition()
	pathVector = unitVector(current,next)
	return math.atan2(pathVector.y, pathVector.x)

#Calculates the theta to travel to goal
# By checking the difference between desrired and current thetas, the 
# direction may be extrapolated once the difference is scaled between
# +/- pi
def thetaToGoal(desiredTheta):
	global robotPose
	thetaToTravel = desiredTheta - robotPose.getCurrentOrientation()
	if thetaToTravel > math.pi:
		thetaToTravel -= 2*math.pi
	if thetaToTravel < -math.pi:
		thetaToTravel += 2*math.pi
	return thetaToTravel

# Rotates the robot using gobal orientation measurements
def globalRotate(desiredTheta):
	global cmdPub, robotPose, turningTolerance, rotationSpeed
	rotateUpdate = rospy.Rate(20)
	# Rotate until desired theta lines up with current orientation
	while math.fabs(desiredTheta - robotPose.getCurrentOrientation()) > turningTolerance and not rospy.is_shutdown():
		direction = math.copysign(1, thetaToGoal(desiredTheta))
		sendTwist(0,rotationSpeed*direction)
		rotateUpdate.sleep()

# Main function to follow a path
def followPath():
	global waypoints,  holster, cmdPub, robotPose, turningTolerance, rotationSpeed
	# TUNING FUNCTIONS
	fast = 0.13
	rotationSpeed = 0.7
	turningTolerance = math.pi/10    # radians
	distanceToGoalTolerance = 0.08   #  cm
	directionUpdateRate = 2
	pathCorrectionSpeed = 1.5

	r = rospy.Rate(directionUpdateRate)
	while  not rospy.is_shutdown():
		try: 
			next = waypoints[1] #could get set at any time by callbacks
			print "About to rotate..."
			globalRotate(calcDesiredTheta(next)) #rotate until at desired theta
				
			# Send robot forward if not at the next waypoint
			angularPathCorrection = pathCorrectionSpeed*thetaToGoal(calcDesiredTheta(next))
			if distance(robotPose.getCurrentPosition(), next) > distanceToGoalTolerance:
				sendTwist(fast,angularPathCorrection)
			else:
				sendTwist(0,0) #stop at waypoint
				waypoints.pop(0)
				print "Arrived at a way point"
			r.sleep()
		except  IndexError:	
			r.sleep()				
		
if __name__ == '__main__':
	rospy.init_node('pathFollow', anonymous=True)
	
	global cmdPub, waypoints, holster, robotPose
	cmdPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
	waypoints = []
	holster = MapHolster('/map_yellow')
	robotPose = JPS()
	
	rospy.Subscriber('/wayPoints', GridCells, gotWaypoints)

	print "Ready to follow a path"
	followPath()