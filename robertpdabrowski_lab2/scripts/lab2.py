#!/usr/bin/env python

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
    
    while not rospy.is_shutdown() and math.fabs(calcDistance(start_pose, current_pose) - distance) > .05:
       sendTwist(math.copysign(speed, distance), 0)
       timePassed += 0.1;
       r.sleep()
    
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global current_theta
   
    goal_theta =  current_theta + angle

    # make sure goal theta is within bounds 
    if goal_theta > math.pi:
        goal_theta -= 2*math.pi
        
    if goal_theta < -math.pi:    
        goal_theta += 2*math.pi
    
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
   
    rospy.sleep(rospy.Duration(.5, 0))
    print "restarting"
    rotate(45*math.pi/180)
    driveStraight(.2,-.3)
    rotate(90*math.pi/180)
  
        


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
        executeTrajectory()



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	pass  # Delete this 'pass' once implemented
  
   




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
    global odom_tf
    global odom_list
   
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot 
    sub = rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback for bumper events


  


    rospy.sleep(rospy.Duration(.5, 0))
    
    executeTrajectory()
    
    rospy.spin()


