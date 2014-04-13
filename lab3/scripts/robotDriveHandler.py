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


def read_odometry(data):
	global current_pose
	global current_theta
	global robotOrientation
	current_pose = data.pose.pose
	robotOrientation = data.pose.pose.orientation
    
	quat = data.pose.pose.orientation # gota convert them quats
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, current_theta = euler_from_quaternion(q)
	#print current_theta


def followPath():
	global current_theta, waypoints
	fast = .4
	waypoints.reverse()
	# for i in range(len(waypoints) - 1):	
	while True and not rospy.is_shutdown():
		if len(waypoints) <= 1:
			continue
		current = waypoints[0]
		next = waypoints[1]
		print current
		print next			
		positionVector = unitVector(current,next)
		desiredTheta = math.atan2(positionVector.x, positionVector.y)
		rotate(desiredTheta - current_theta)
		driveStraight(fast,distance(current,next))
		waypoints.pop(0)
		
def gotWaypoints(waypointMsg):
	global wayPoints
	wayPoints = waypointMsg.cells


#######################################################################
#
#		Drive Helper Functions
#
#######################################################################

# Helper function to make a Twist object with the given vlaues
def makeTwist( forwardVelocity, angularVelocity):
    twist = Twist()
    
    # Set Default values
    twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; 
      
    # Set Given values
    twist.linear.x = forwardVelocity; 
    twist.angular.z = angularVelocity;
  
    return twist

def angleDelta(prev,current):
	return abs(prev - current)

#takes in a normalized w from robotOrientation Quaternion, 
#and converts it to a Yaw angle in radians.
def calcYaw(robotW):
	return 2*(m.acos(robotW))

def getYaw(Quaternion):
	global robotOrientation
	angles = euler_from_quaternion([robotOrientation.x,robotOrientation.y,robotOrientation.z,robotOrientation.w])
	angles = [(180.0/math.pi)*i for i in angles]
	return angles[2]

def calcTotalDegs(prevAngle,current,totalAngle):
	global robotOrientation
	if angleDelta(prevAngle,current) > 20:
		totalAngle += 360 - angleDelta(prevAngle,current)
	else:
		totalAngle += angleDelta(prevAngle,getYaw(robotOrientation))
	return totalAngle

def rotateDegs(angle):
	global turnSpeed, robotOrientation
	r = rospy.Rate(30)
	totalAngle = 0
	prevAngle = getYaw(robotOrientation)

	if angle > 0:
		omega = .8
	else:
		omega = -.8
	while not rospy.is_shutdown() and totalAngle <= abs(angle):
		totalAngle = calcTotalDegs(prevAngle,getYaw(robotOrientation),totalAngle)
		prevAngle = getYaw(robotOrientation)
		pub.publish(makeTwist(0,omega))
		r.sleep()

#Takes a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global current_pose
    start_pose = current_pose
    
    r = rospy.Rate(10) # 10hz
    timePassed = 0.0
    
    while not rospy.is_shutdown() and math.fabs(calcDistance(start_pose, current_pose) - distance) > .05:
       sendTwist(math.copysign(speed, distance), 0)
       timePassed += 0.1;
       r.sleep()
    sendTwist(0,0)#stop 

def rotate(radians):
	degs = (radians * 180) / math.pi
	rotateDegs(degs)

# Sends the given values in a Twist to the topic
def sendTwist(forwardVelocity, angularVelocity):
    global pub
    pub.publish(makeTwist(forwardVelocity,angularVelocity))

# Calculates the magnitude of the distance between two points in the x-y plane
def calcDistance(start, dst):
    return math.sqrt( math.pow(dst.position.x - start.position.x,2) + math.pow(dst.position.y - start.position.y,2))


if __name__ == '__main__':
    
    # Change this node name to include your username\
	rospy.init_node('pathFollow', anonymous=True)
	global current_theta
	global robotOrientation
	global pub
	global wayPoints
	wayPoints = [Point(0,0,0),Point(0,0,0)]
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
	
	rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) 
	rospy.Subscriber('/waypoints', GridCells, gotWaypoints
		)
	# Wait, then spin. Exectute trajectory activated by bumper events
	rospy.sleep(rospy.Duration(.5, 0)) 
	# wayPoints = [Point(4,2,0), Point(5,2,0), Point(5,3,0), Point(4,3,0),  Point(4,2,0)]
	followPath(wayPoints)
	      
	rospy.spin()
