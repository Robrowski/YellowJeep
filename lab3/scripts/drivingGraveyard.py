
# Reads the odometry
def read_odometry(msg):
	global currentOrientation, currentPosition, flag
	
	flag = 1 # Used for other functions to know when first odometry reading was executed
	
	currentPosition = msg.pose.pose.position
	currentOrientation = msg.pose.pose.orientation

def getYaw():
	global robotPose

	global currentOrientation	
	while flag == 0: #wait until an odometry message comes in
		pass
	quaternion = setArray(currentOrientation)
	currentYaw = euler_from_quaternion(quaternion)[2]	
	return currentYaw
			
def gotWaypoints(waypointMsg):
	global robotPose

	global waypoints
	waypoints = waypointMsg.cells
	waypoints.reverse()
	

# Accepts an angle and makes the robot rotate around it.
def setArray(currentOrientation):
	global robotPose

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

