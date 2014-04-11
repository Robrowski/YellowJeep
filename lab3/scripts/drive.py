#!/usr/bin/env python

##############################################################################
# Ros Imports
##############################################################################
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion

##############################################################################
# Twist Helpers... what a twist!
##############################################################################
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

# Converts wheel velocities to robot forward and angular velocities in a Twist
def wheelSpeedToTwist(u1, u2): 	
    cm_in_meter = 100.0;
    wheelRadius = 3.5/cm_in_meter; #3.5cm
    wheelBase   =  23.0/cm_in_meter;  #23cm
    
    angularVelocity = (wheelRadius/wheelBase)*(u1-u2)
    forwardVelocity = wheelRadius/2*(u1 + u2)
    return makeTwist(forwardVelocity, angularVelocity)

# Sends the given values in a Twist to the topic
def sendTwist(forwardVelocity, angularVelocity):
    global pub
    pub.publish(makeTwist(forwardVelocity,angularVelocity))

# Sends the given values to the topic after converting to Twist
def sendWheelVelocities(u1, u2):
    newTwist = wheelSpeedToTwist(u1,u2)
    sendTwist(newTwist.linear.x, newTwist.angular.z)

##############################################################################
# Random Helpers
##############################################################################
# Calculates the magnitude of the distance between two points in the x-y plane
def calcDistance(start, dst):
    return math.sqrt( math.pow(dst.position.x - start.position.x,2) + math.pow(dst.position.y - start.position.y,2))


##############################################################################
# Lab 2 Functions
##############################################################################
#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    r = rospy.Rate(10) # 10hz
    timePassed = 0.0
    
    # Send the command to move at the given speed until time is up
    while not rospy.is_shutdown() and timePassed < time:
       sendWheelVelocities(u1,u2)
       timePassed += 0.1;
       r.sleep()
     
    sendTwist(0,0)#stop

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


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global current_theta
    goal_theta =  current_theta + angle
    angular_velocity = math.copysign(speed/radius, angle)

    # make sure goal theta is within bounds 
    if goal_theta > math.pi:
        goal_theta -= 2*math.pi
        
    if goal_theta < -math.pi:    
        goal_theta += 2*math.pi
    
    # Arc until desired rotation is achieved #todo make robust
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown() and  math.fabs(current_theta - goal_theta )  > math.pi/100  :
        sendTwist(speed, angular_velocity)
        r.sleep
    sendTwist(0,0)#stop

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    print "Driving Straight 60cm"
    driveStraight(.2,.6)
     
    print "rotating right 90 degrees"
    rotate(-math.pi/2)
    
    print "-180 degree arc, radius = .15cm"
    driveArc(.15, .2, -math.pi)
    
    print "rotating"
    rotate(135*math.pi/180)
  
    print "Driving Straight 42cm"
    driveStraight(.2,.42)
   
    print "DONE!"        

# Used for robotDriveHandler
def getTheta():
	global current_theta
	return current_theta

#Odometry Callback function.
def read_odometry(data):
    global current_pose
    global current_theta
    current_pose = data.pose.pose
    
    quat = data.pose.pose.orientation # gota convert them quats
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, current_theta = euler_from_quaternion(q)
     

#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        #executeTrajectory()

   
# This is the program's main function
if __name__ == '__main__':
    
    # Change this node name to include your username
    rospy.init_node('rpdabrowski_Lab_2_node', anonymous=True)
    
    # Publisher for commanding robot
    global pub
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)  
    
    # Subscribers for odometry and bumper events
    rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) 
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1)   

	# Wait, then spin. Exectute trajectory activated by bumper events
    rospy.sleep(rospy.Duration(.5, 0))       
    rospy.spin()

