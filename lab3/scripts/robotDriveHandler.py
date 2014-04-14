#!/usr/bin/env python
import rospy, math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, GridCells
from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *
from mapUtils import *
from tf.transformations import euler_from_quaternion
#from drive import *


# def read_odometry(data):
# 	global current_pose
# 	global current_theta
# 	global robotOrientation
# 	current_pose = data.pose.pose
# 	robotOrientation = data.pose.pose.orientation
#     
# 	quat = data.pose.pose.orientation # gota convert them quats
# 	q = [quat.x, quat.y, quat.z, quat.w]
# 	roll, pitch, current_theta = euler_from_quaternion(q)
# 	#print current_theta

def read_odometry(msg):
	global currentOrientation, currentPosition, flag
	
	flag = 1 # Used for other functions to know when first odometry reading was executed
	
	currentPosition = msg.pose.pose.position
	currentOrientation = msg.pose.pose.orientation


def followPath():
	global current_theta, waypoints
	current_theta = 1
	fast = .4
#	waypoints.reverse()
	# for i in range(len(waypoints) - 1):	
	while True and not rospy.is_shutdown():
		if len(waypoints) <= 1:
			continue
		print "following a path " + str(len(waypoints)) + " points long"
		current = waypoints[0]
		next = waypoints[1]
		print current
		print next			
		positionVector = unitVector(current,next)
		desiredTheta = math.atan2(positionVector.x, positionVector.y)
		print desiredTheta
		print current_theta
		rotate(desiredTheta - current_theta)
		driveStraight(fast,distance(current,next))
		waypoints.pop(0)
		
def gotWaypoints(waypointMsg):
	global waypoints
	waypoints = waypointMsg.cells
	waypoints.reverse()
	print "got a path"
	print waypoints
	


#######################################################################
#
#		Drive Helper Functions
#
#######################################################################

# Helper function to make a Twist object with the given vlaues
# def makeTwist( forwardVelocity, angularVelocity):
#     twist = Twist()
#     
#     # Set Default values
#     twist.linear.y = 0; twist.linear.z = 0
#     twist.angular.x = 0; twist.angular.y = 0; 
#       
#     # Set Given values
#     twist.linear.x = forwardVelocity; 
#     twist.angular.z = angularVelocity;
#   
#     return twist
# 
# def angleDelta(prev,current):
# 	return abs(prev - current)
# 
# #takes in a normalized w from robotOrientation Quaternion, 
# #and converts it to a Yaw angle in radians.
# def calcYaw(robotW):
# 	return 2*(m.acos(robotW))
# 
# def getYaw(Quaternion):
# 	global robotOrientation
# 	angles = euler_from_quaternion([robotOrientation.x,robotOrientation.y,robotOrientation.z,robotOrientation.w])
# 	angles = [(180.0/math.pi)*i for i in angles]
# 	return angles[2]
# 
# def calcTotalDegs(prevAngle,current,totalAngle):
# 	global robotOrientation
# 	if angleDelta(prevAngle,current) > 20:
# 		totalAngle += 360 - angleDelta(prevAngle,current)
# 	else:
# 		totalAngle += angleDelta(prevAngle,getYaw(robotOrientation))
# 	return totalAngle
# 
# def rotateDegs(angle):
# 	global turnSpeed, robotOrientation
# 	r = rospy.Rate(30)
# 	totalAngle = 0
# 	prevAngle = getYaw(robotOrientation)
# 
# 	if angle > 0:
# 		omega = .8
# 	else:
# 		omega = -.8
# 	while not rospy.is_shutdown() and totalAngle <= abs(angle):
# 		totalAngle = calcTotalDegs(prevAngle,getYaw(robotOrientation),totalAngle)
# 		prevAngle = getYaw(robotOrientation)
# 		pub.publish(makeTwist(0,omega))
# 		r.sleep()
# 
#Takes a speed and a distance for the robot to move in a straight line
# def driveStraight(speed, distance):
#     global current_pose
#     start_pose = current_pose
#      
#     r = rospy.Rate(10) # 10hz
#     timePassed = 0.0
#      
#     while not rospy.is_shutdown() and math.fabs(calcDistance(start_pose, current_pose) - distance) > .05:
#        sendTwist(math.copysign(speed, distance), 0)
#        timePassed += 0.1;
#        r.sleep()
#     sendTwist(0,0)#stop 
# 
# def rotate(radians):
# 	degs = (radians * 180) / math.pi
# 	rotateDegs(degs)
# 
# # Sends the given values in a Twist to the topic
# def sendTwist(forwardVelocity, angularVelocity):
#     global pub
#     pub.publish(makeTwist(forwardVelocity,angularVelocity))
# 
# # Calculates the magnitude of the distance between two points in the x-y plane
# def calcDistance(start, dst):
#     return math.sqrt( math.pow(dst.position.x - start.position.x,2) + math.pow(dst.position.y - start.position.y,2))


# Accepts an angle and makes the robot rotate around it.
def setArray(currentOrientation):
	quaternion = [currentOrientation.x,currentOrientation.y,currentOrientation.z,currentOrientation.w]
	return quaternion

def getYaw():
	global currentOrientation	
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
		# Do Nothing	
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
    
    # Change this node name to include your username\
	rospy.init_node('pathFollow', anonymous=True)
# 	global current_theta
# 	global robotOrientation
	global currentPosition, flag
	global pub
	global waypoints
	flag = 0
	currentPosition = Point()
	waypoints = []
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
	
	r = rospy.Rate(10)
	rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) 
	rospy.Subscriber('/wayPoints', GridCells, gotWaypoints)
	# Wait, then spin. Exectute trajectory activated by bumper events
# 	rospy.sleep(rospy.Duration(.5, 0)) 
	# wayPoints = [Point(4,2,0), Point(5,2,0), Point(5,3,0), Point(4,3,0),  Point(4,2,0)]
	print "Ready to follow a path"
	followPath()
	      

	rospy.spin()
