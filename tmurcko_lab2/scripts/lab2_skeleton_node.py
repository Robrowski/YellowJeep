#!/usr/bin/env python

import rospy, tf, math as m
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion  
from kobuki_msgs.msg import BumperEvent

# Tim Murcko
# RBE3002 - Navigation
# Lab 2: Mobile Robot Kinematics and Odometry
# This code contains everything required for lab 2 submission. Skeleton code
# filled out, and additional helper functions added as necessary.  


# Physical robot controls
radius = 0.035				# Wheel radius (m)
base = 0.40					# Wheel base (m)

# Returns array of orientation daya - required for quaternion function
def setArray(currentOrientation):
	quaternion = [currentOrientation.x,currentOrientation.y,currentOrientation.z,currentOrientation.w]
	return quaternion

# Gets angle of robot
def getYaw():
	global currentOrientation	
	quaternion = setArray(currentOrientation)
	currentYaw = euler_from_quaternion(quaternion)[2]	
	return currentYaw

# Makes twist for driving robot
def createTwist(u,w):
	twist = Twist()
	twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0		# Sets x,y,z speed
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w	# Sets angular speed
	return twist

# This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	driveStraight(0.2,0.6)
	rotate(m.pi/2)
	driveArc(0.15,0.2,-m.pi)
	rotate(0.75*m.pi)
	driveStraight(0.2,0.42)    

# This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
	global timerValue			# Fixed period timer 	

	u = (radius/2)*(u1 + u2)	# Robot velocity in u direction
	w = (radius/base)*(u1 - u2) # Angular velocity about Z axis 		
	
	rospy.Timer(rospy.Duration(0.01), timerCallback)	# Start timer 

	# Publish calculated velocities
	while (timerValue < time): 		
		twist = createTwist(u, w)
		pub.publish(twist)    

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

# Accepts an angle and makes the robot rotate around it.
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

# This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	totalAngle = 0

	# Wait for first odometry reading
	while flag == 0:
		# Do Nothing	
		pass
	
	# Initilize yaw
	initYaw = getYaw()	
	previousAngle = initYaw	

	deltaAngle = 0	
	w = (speed/radius)*(abs(angle)/angle)	# Calculates angular velocity

	# Keep turning until angle is met
	while (totalAngle < abs(angle)):
		twist = createTwist(speed,w)
		pub.publish(twist)
		currentAngle = getYaw()
		
		# Required logic because odometry reading "resets" at 180 degrees  - same as rotate function above
		if angle > 0:
			deltaAngle = currentAngle - previousAngle
		
			if deltaAngle < -3:
				totalAngle = totalAngle + (3.14159+getYaw())
			else:		
				totalAngle = totalAngle + deltaAngle 
		else:
			deltaAngle = previousAngle - currentAngle
			
			if deltaAngle < -3:
				totalAngle = totalAngle + (3.14159-getYaw())
			else:		
				totalAngle = totalAngle + deltaAngle 
		
		previousAngle = currentAngle
		print totalAngle
		r.sleep()	# Required to ensure publishing is not too fast and queue does not mess up

# Odometry Callback function.
def read_odometry(msg):
	global currentOrientation, currentPosition, flag
	
	flag = 1 # Used for other functions to know when first odometry reading was executed
	
	currentPosition = msg.pose.pose.position
	currentOrientation = msg.pose.pose.orientation

# Bumper Event Callback function
def readBumper(msg):    
	global flagBump	

	# Bumper pressed
	if (msg.state == 1):
		flagBump = 1 # pressed state


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
# rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	global timerValue	
	timerValue = timerValue + 0.01 # Used for spin wheels function   

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
	rospy.init_node('Tim_Murcko_lab2_skeleton', anonymous=True)
    
    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
	global pub
	global pose
	global odom_tf
	global odom_list
	global timerValue			# Fixed period timer 
	global currentPosition
	global currentOrientation
	global flag					# Odom flag
	global flagBump
	flagBump = 0
	flag = 0
	currentOrientation = 0
	currentPosition = Point()
	timerValue = 0
    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion
	sub = rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

	r = rospy.Rate(10)	# 10Hz
	 
    # Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()
    
	print "Starting Lab 2..."

	#spinWheels(5, 5, 3)
	#driveStraight(-0.1, 1)
	#rotate(-3.14)
	#driveArc(0.3, 0.2, 6.28)
	
	# Wait for front bumper to be pressed 
	while flagBump == 0:
		pass
		r.sleep()		
	
	# Execture trajectory when bumper is pressed 	
	executeTrajectory()

	print "Lab 2 complete!"

