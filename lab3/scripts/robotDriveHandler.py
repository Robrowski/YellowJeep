#!/usr/bin/env python
import rospy, math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells
from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *
from mapUtils import *
from tf.transformations import euler_from_quaternion

# Reads the odometry
def read_odometry(msg):
	global currentOrientation, currentPosition, flag
	
	flag = 1 # Used for other functions to know when first odometry reading was executed
	
	currentPosition = msg.pose.pose.position
	currentOrientation = msg.pose.pose.orientation

def getYaw():
	global currentOrientation	
	while flag == 0: #wait until an odometry message comes in
		pass
	quaternion = setArray(currentOrientation)
	currentYaw = euler_from_quaternion(quaternion)[2]	
	return currentYaw
			
def gotWaypoints(waypointMsg):
	global waypoints
	waypoints = waypointMsg.cells
	waypoints.reverse()
	

# Accepts an angle and makes the robot rotate around it.
def setArray(currentOrientation):
	quaternion = [currentOrientation.x,currentOrientation.y,currentOrientation.z,currentOrientation.w]
	return quaternion


def createTwist(u,w):
	twist = Twist()
	twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0		# Sets x,y,z speed
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w	# Sets angular speed
	return twist

def rotate(angle):
	totalAngle = 0
	
	# Change angular direction based on input angle
	if angle > 0:
		speed = 0.7
	else:
		speed = -0.7

	# Wait for first odometry reading
	while flag == 0:
		# Do Nothing	
		pass
	
	# Initilize yaw
	initYaw = getYaw()	
	previousAngle = initYaw

	# Keep turning until angle is met
	while (totalAngle < abs(angle)):
		
		twist = createTwist(0,speed)
		pub.publish(twist)
		currentAngle = getYaw()
		
		# Required logic because odometry reading "resets" at 180 degrees 
		if angle > 0:
			deltaAngle = currentAngle - previousAngle	# Calculate change in current and previous angles
		
			if deltaAngle < -3:	# If odoemtry "resets"
				totalAngle = totalAngle + (3.14159+getYaw()) # Compensate for "reset"
			else:		
				totalAngle = totalAngle + deltaAngle 
		else:
			deltaAngle = previousAngle - currentAngle	# Calculate change in current and previous angles
			
			if deltaAngle < -3: # If odoemtry "resets"
				totalAngle = totalAngle + (3.14159-getYaw()) # Compensate for "reset"
			else:		
				totalAngle = totalAngle + deltaAngle 
		
		previousAngle = currentAngle
		
		r.sleep() # Required to ensure publishing is not too fast and queue does not mess up

# This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global currentPosition,flag	
	
	# Wait for first odometry reading
	while flag == 0:
		pass

	initialPosition = currentPosition		# get position from first odom reading
	distanceTraveled = 0					# used to calculate total distance traveled

	# If distance traveled has not been met, keep running
	while (distanceTraveled < distance):
		twist = createTwist(speed,0)
		pub.publish(twist)
		distanceTraveled = (((currentPosition.x-initialPosition.x)**2)+((currentPosition.y-initialPosition.y)**2))**0.5	# Distance formula
		r.sleep()	# Required to ensure publishing is not too fast and queue does not mess up
		

# Main function to follow a path
def followPath():
	global waypoints, currentPosition, holster, pub
	# TUNING FUNCTIONS
	fast = 0.13
	turningTolerance = math.pi/10    # radians
	distanceToGoalTolerance = 0.08   #  cm
	directionUpdateRate = 2


	r = rospy.Rate(directionUpdateRate)
	while  not rospy.is_shutdown():
		try: 
			next = waypoints[1] #could get set at any time by callbacks
			current = holster.getCurrentPosition()
			currentTheta = holster.getCurrentOrientation()

			pathVector = unitVector(current,next)
			desiredTheta = math.atan2(pathVector.y, pathVector.x)
	
			goal_theta = desiredTheta - currentTheta
			
			# make sure goal theta is within bounds of +/- pi
			if goal_theta > math.pi:
				goal_theta -= 2*math.pi
			if goal_theta < -math.pi:
				goal_theta += 2*math.pi
			
			# Rotate if the current orientation is out of tolerances
			if math.fabs(goal_theta) > turningTolerance:
				rotate(goal_theta)

			# Send robot forward if not at the next waypoint
			if distance(holster.getCurrentPosition(), next) > distanceToGoalTolerance:
				pub.publish(createTwist(fast,0))	
			else:
				pub.publish(createTwist(0,0)) #stop
				waypoints.pop(0)
				print "Arrived at a way point"
			r.sleep()
		except  IndexError:	
			continue		
		
		
if __name__ == '__main__':
	rospy.init_node('pathFollow', anonymous=True)

	global currentPosition,currentOrientation, holster
	global pub, waypoints, flag
	flag = 0
	currentOrientation = 0
	currentPosition = Point()
	waypoints = []
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
	
	r = rospy.Rate(10)

	holster = MapHolster('/map_yellow')

	rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) 	
	rospy.Subscriber('/wayPoints', GridCells, gotWaypoints)

	print "Ready to follow a path"
	followPath()

	rospy.spin()
