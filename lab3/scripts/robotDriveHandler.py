#!/usr/bin/env python
import rospy, math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from YellowPublisher import *
from MapHolster import *
from mapUtils import *
from tf.transformations import euler_from_quaternion
#from drive import *


def read_odometry(data):
	global current_pose
	global current_theta
	current_pose = data.pose.pose
    
	quat = data.pose.pose.orientation # gota convert them quats
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, current_theta = euler_from_quaternion(q)
	#print current_theta


def followPath(waypoints):
	global current_theta
	fast = .4
	waypoints.reverse()
	for i in range(len(waypoints) - 1):	
		current = waypoints[i]
		next = waypoints[i+1]
		print current
		print next			
		positionVector = unitVector(current,next)
		desiredTheta = math.atan2(positionVector.x, positionVector.y)
		rotate(desiredTheta - current_theta)
		driveStraight(fast,distance(current,next))
		

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

	
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global current_theta
   
    goal_theta =  current_theta + angle

    # make sure goal theta is within bounds of +/- pi
    if goal_theta > math.pi:
        goal_theta -= 2*math.pi
        
    if goal_theta < -math.pi:    
        goal_theta += 2*math.pi
    
    # Rotate in optimal direction until at setpoint
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown() and  math.fabs(current_theta - goal_theta )  > math.pi/100  : 
       sendTwist(0, math.copysign(.4, angle))
       r.sleep()
    
    sendTwist(0,0)#stop

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
	
	global pub
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
	
	rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) 
	# Wait, then spin. Exectute trajectory activated by bumper events
	rospy.sleep(rospy.Duration(.5, 0)) 
	wayPoints = [Point(4,2,0), Point(6,2,0), Point(6,4,0), Point(4,4,0),  Point(4,2,0)]
	followPath(wayPoints)
	      
	rospy.spin()




