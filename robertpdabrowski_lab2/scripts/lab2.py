#!/usr/bin/env python
from math import degrees, radians



##############################################################################
# Ros Imports
##############################################################################
import rospy, tf, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent, CliffEvent
from tf.transformations import euler_from_quaternion



# Helper function to make a Twist object with the given vlaues
def makeTwist( speed, rotation):
    twist = Twist()
    twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = rotation
    return twist

# Converts wheel velocities to robot forward and angular velocities
# exports in m/s
def wheelSpeedToTwist(u1, u2): 	
	global wheelBase, wheelRadius
	angular = (wheelRadius/wheelBase)*(u1-u2)
	translational = wheelRadius/2*(u1 + u2)
	return makeTwist(translational, angular)


def sendTwist(forward, angular):
    global pub
    pub.publish(makeTwist(forward,angular))


def sendWheelVelocities(u1, u2):
    newTwist = wheelSpeedToTwist(u1,u2)
    sendTwist(newTwist.linear.x, newTwist.angular.z)



#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
      
    sendWheelVelocities(u1,u2)
    
    r = rospy.Rate(10) # 10hz
    timePassed = 0.0
    
    while not rospy.is_shutdown() and timePassed < time:
        sendWheelVelocities(u1,u2)
        timePassed += 0.1;
       	r.sleep()
     
    sendTwist(0,0)#stop
	

def calcDistance(start, dst):
    return math.sqrt( math.pow(dst.position.x - start.position.x,2) + math.pow(dst.position.y - start.position.y,2))




#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global current_pose
    start_pose = current_pose
    
    r = rospy.Rate(10) # 10hz
    timePassed = 0.0
    
    while not rospy.is_shutdown() and calcDistance(start_pose, current_pose) < distance:
       sendTwist(speed, 0)
       timePassed += 0.1;
       r.sleep()
    
    
    

    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global current_theta
    start_theta = current_theta
    
    r = rospy.Rate(10) # 10hz
    timePassed = 0.0
    
    while not rospy.is_shutdown() and  (current_theta - start_theta )  < angle:
       sendTwist(0, .1)
       timePassed += 0.1;
       r.sleep()
    



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented



#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    pass  # Delete this 'pass' once implemented



#Odometry Callback function.
def read_odometry(data):
    global current_pose
    global current_theta
    current_pose = data.pose.pose
    
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    current_theta = yaw
    
    


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        pass  # Delete this 'pass' once implemented



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	pass # Delete this 'pass' once implemented






# This is the program's main function
if __name__ == '__main__':
    
    # Change this node name to include your username
    rospy.init_node('rpdabrowski_Lab_2_node', anonymous=True)
    
    global wheelBase
    global wheelRadius
    cm_in_meter = 100.0;
    wheelRadius = 3.5/cm_in_meter; #3.5cm
    wheelBase   =  23.0/cm_in_meter;  #23cm
    
    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global pose
    global odom_tf
    global odom_list
    global current_pose
    
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot 
    sub = rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    
# https://github.com/yujinrobot/kobuki/blob/hydro-devel/kobuki_testsuite/src/kobuki_testsuite/motion_wander.py
#    odom_subscriber = rospy.Subscriber(odom_topic, Odometry, read_odometry)
 
 
 #   bumper_sub = rospy.Subscriber('...', ..., readBumper, queue_size=1) # Callback for bumper events

    # Use this object to get the robot's Odometry 
    # why is this here!?
 #   odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))


    print "Starting Lab 2"
  #  print "Spinning kinda"
  #  spinWheels(5, .7, 3)
    
   
   # print "Driving Straight"
    #driveStraight(.3,1)
   
    print "rotating"
    rotate(math.pi)
   
    print "Lab 2 complete!"
   
'''
    try:
    	print "Publishing to make robot drive in a circle..."
    	r = rospy.Rate(10) # 10hz
    	while not rospy.is_shutdown():
    		pub.publish(makeTwist(0.2,.2))
        	r.sleep()
    	
    except:
    	print e
    
    finally:
    # publish a stop command
    	print "Finally.. crap"
    	pub.publish(makeTwist(0,0))
  
'''



