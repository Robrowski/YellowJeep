#!/usr/bin/env python
import rospy, math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells
from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *
from mapUtils import *
from tf.transformations import euler_from_quaternion


def read_odometry(msg):
	global currentOrientation, currentPosition, flag
	
	flag = 1 # Used for other functions to know when first odometry reading was executed
	
	currentPosition = msg.pose.pose.position
	currentOrientation = msg.pose.pose.orientation


def followPath():
	global waypoints, currentPosition, holster, pub
	fast = 0.13
	turningTolerance = math.pi/10    # radians
	distanceToGoalTolerance = 0.08   #  cm
	directionUpdateRate = 2

	waypoints.reverse()
	# for i in range(len(waypoints) - 1):	
	r = rospy.Rate(directionUpdateRate)
	while  not rospy.is_shutdown():
		try: 
			next = waypoints[1]
			#print "following a path " + str(len(waypoints)) + " points long"
			current = holster.getCurrentPosition()
			currentTheta = holster.getCurrentOrientation()

			positionVector = unitVector(current,next)
			desiredTheta = math.atan2(positionVector.y, positionVector.x)
	
			#print "desired theta: " + str(desiredTheta)
			#print "current theta: " + str(currentTheta)
			
			goal_theta = desiredTheta - currentTheta
			
			# make sure goal theta is within bounds of +/- pi
			if goal_theta > math.pi:
				goal_theta -= 2*math.pi
			if goal_theta < -math.pi:
				goal_theta += 2*math.pi
			
			#print "desired-current: " + str(goal_theta)
			
			if math.fabs(goal_theta) > turningTolerance:
				rotate(goal_theta)

			#print "position: "
			print holster.getCurrentPosition()
			#print "next"
			#print holster.newGridCell(next)
			print next
			print 	"Distance to next: " + str(distance(holster.getCurrentPosition(), (next)))
	
			if distance(holster.getCurrentPosition(), next) > distanceToGoalTolerance:
				pub.publish(createTwist(fast,0))	
			else:
				# stop
				pub.publish(createTwist(0,0))
				# check if at waypoint (or close enough, then pop)
				waypoints.pop(0)
			r.sleep()
		except  IndexError:	
			continue
			
def gotWaypoints(waypointMsg):
	global waypoints
	waypoints = waypointMsg.cells
	waypoints.reverse()
	print "Drive got a path"
	#print waypoints
	

# Accepts an angle and makes the robot rotate around it.
def setArray(currentOrientation):
	quaternion = [currentOrientation.x,currentOrientation.y,currentOrientation.z,currentOrientation.w]
	return quaternion

def getYaw():
	global currentOrientation	
	while flag == 0:
		# Do Nothing	
		pass
	quaternion = setArray(currentOrientation)
	currentYaw = euler_from_quaternion(quaternion)[2]	
	return currentYaw

def createTwist(u,w):
	twist = Twist()
	twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0		# Sets x,y,z speed
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w	# Sets angular speed
	return twist

def rotate(angle):
	totalAngle = 0
	
	# Changle angular direction based on input angle
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

	holster = MapHolster('/newMap')

	rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) 	
	rospy.Subscriber('/wayPoints', GridCells, gotWaypoints)

	print "Ready to follow a path"
	followPath()

	rospy.spin()
