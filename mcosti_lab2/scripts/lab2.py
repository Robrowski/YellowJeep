#!/usr/bin/env python

import rospy, tf 
import math as m
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used


def makeTwist(u,omega):
	twist = Twist()
	twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = omega
	return twist
radius = .035
base_width = .40
def makeRobotSpeed(u1,u2):
	# global radius, base_width, pub, timePassed

	u = (radius/2) * (u1 + u2)
	print u
	omega = (radius/base_width) * (u1 - u2)
	print omega
	return u,omega

def calcVectorLength(x,y):
	return m.sqrt(x**(2) + y**(2))

def calcDistance(x1, y1, x2, y2):
	return m.sqrt((x2-x1)**(2) + (y2-y1)**(2))

#takes in a normalized w from robotOrientation Quaternion, 
#and converts it to a Yaw angle in radians.
def calcYaw(robotW):
	return 2*(m.acos(robotW))

def getYaw(Quaternion):
	angles = euler_from_quaternion([robotOrientation.x,robotOrientation.y,robotOrientation.z,robotOrientation.w])
	angles = [(180.0/m.pi)*i for i in angles]
	return angles[2]

def angleDelta(prev,current):
	return abs(prev - current)

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	driveStraight(.60,.3)
	rotateDegs(90)
	driveArc(.15,.3, -180)
	rotateDegs(135)
	driveStraight(.42,.3)


def waitForOdom():
	global odomFlag
	while not odomFlag:
		print odomFlag
		pass



#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	global timePassed
	u,omega = makeRobotSpeed(u1,u2)
	pub.publish(makeTwist(u,omega))
	r = rospy.Rate(10) #10hz

	while not rospy.is_shutdown() and timePassed < time:
		timePassed = timePassed + .1
		pub.publish(makeTwist(u,omega))
		r.sleep()

	pub.publish(makeTwist(0,0))


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global pub, robotPosition, robotOrientation

	waitForOdom()

	initX = robotPosition.x
	initY = robotPosition.y

	while not rospy.is_shutdown() and (calcDistance(initX,initY,robotPosition.x,robotPosition.y) < distance):
		twist = makeTwist(speed,0)
		print "orientation:"
		# print robotOrientation
		# print "position"
		# print robotPosition
		pub.publish(twist)
		r.sleep()

	pub.publish(makeTwist(0,0))

def calcTotalDegs(prevAngle,current,totalAngle):
	if angleDelta(prevAngle,current) > 20:
		print "POOOOOOOOOPPPPP"
		print prevAngle
		print getYaw(robotOrientation)
		print angleDelta(prevAngle,current)
		totalAngle += 360 - angleDelta(prevAngle,current)
		print totalAngle
	else:
		print totalAngle
		totalAngle += angleDelta(prevAngle,getYaw(robotOrientation))
	return totalAngle

def rotateDegs(angle):
	global turnSpeed, robotOrientation

	totalAngle = 0
	prevAngle = getYaw(robotOrientation)

	if angle > 0:
		omega = .3
	else:
		omega = -.3
	while not rospy.is_shutdown() and totalAngle <= abs(angle):
		totalAngle = calcTotalDegs(prevAngle,getYaw(robotOrientation),totalAngle)
		prevAngle = getYaw(robotOrientation)
		pub.publish(makeTwist(0,omega))
		r.sleep()

def rotate(radians):
	degs = (radians * 180) / m.pi
	rotateDegs(degs)

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	omega = speed/radius * (abs(angle)/angle)
	totalAngle = 0
	prevAngle = getYaw(robotOrientation)

	while not rospy.is_shutdown() and totalAngle <= abs(angle):
		totalAngle = calcTotalDegs(prevAngle,getYaw(robotOrientation),totalAngle)
		prevAngle = getYaw(robotOrientation)
		pub.publish(makeTwist(speed,omega))
		r.sleep()

def kickStart():
	while not bumpFlag and not rospy.is_shutdown():
		print 'not kicked'
		r.sleep()
	# executeTrajectory()
	# driveStraight(.2,1)
	executeTrajectory()

#Odometry Callback function.
def read_odometry(msg):
	global robotPosition, robotAngle, robotOrientation, totalAngle, prevAngle,odomFlag
	robotPosition = msg.pose.pose.position
	robotOrientation = msg.pose.pose.orientation
	odomFlag = 1

#Bumper Event Callback function
def readBumper(msg):
	global bumpFlag
	if (msg.state == 1):
		bumpFlag = 1
		print "BOOOOOOOP"



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	global timePassed
	timePassed += .01









# This is the program's main function
if __name__ == '__main__':
	# Change this node name to include your username
	# rospy.init_node('sample_Lab_2_node')


	# These are global variables. Write "global <variable_name>" in any other function
	#  to gain access to these global variables
	
	global pub
	global pose
	global odom_tf
	global odom_list
	# global radius
	# global base_width
	global timePassed
	global robotPosition
	global robotOrientation
	global turnSpeed
	global odomFlag
	global bumpFlag
	# global prevAngle, totalAngle

	# radius = 3.5
	# base_width = 40
	timePassed = 0
	turnSpeed = 3
	odomFlag = 0
	bumpFlag = 0
	# prevAngle = getYaw(robotOrientation)
	# totalAngle = 0

	
	# Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion
	sub = rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
	bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

	rospy.init_node('lab2_matt')
	# Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()

	r = rospy.Rate(30) #10hz
	
	# Use this command to make the program wait for some seconds
	# rospy.sleep(rospy.Duration(1, 0))



	print "Starting Lab 2"

	# spinWheels(2,2,2) 
	# driveStraight(.2,1)
	# prevAngle = 9000
	# rotateDegs(720)
	# driveArc(.2,.1,180)
	# executeTrajectory();
	kickStart()
	# Make the robot do stuff...

	print "Lab 2 complete!"

