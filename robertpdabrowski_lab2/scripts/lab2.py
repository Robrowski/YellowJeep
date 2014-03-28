#!/usr/bin/env python

import rospy, tf 
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
#from std_msgs.msg import String





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


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub
      
    pub.publish(wheelSpeedToTwist(u1,u2))
    
    r = rospy.Rate(10) # 10hz
    timePassed = 0.0
    
    while not rospy.is_shutdown() and timePassed < time:
        pub.publish(wheelSpeedToTwist(u1,u2))
        timePassed += 0.1;
       	r.sleep()
     
    pub.publish(makeTwist(0,0))#stop
	


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    
    pass  # Delete this 'pass' once implemented


    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    pass  # Delete this 'pass' once implemented



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented



#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    pass  # Delete this 'pass' once implemented



#Odometry Callback function.
def read_odometry(msg):
    pass  # Delete this 'pass' once implemented
  


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

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot 
    sub = rospy.Subscriber('...', ..., read_odometry, queue_size=1) # Callback function to read in robot Odometry messages

 #   bumper_sub = rospy.Subscriber('...', ..., readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))


    print "Starting Lab 2"
    spinWheels(1, .1, 10)
    
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



