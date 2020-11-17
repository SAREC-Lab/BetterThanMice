#!/usr/bin/env python
# Software License Agreement (BSD License)
# Modified from https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
# to work with TurtleBot3-Burger

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math

x = 0.0
y = 0.0 
theta = 0.0

# Callback function
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Initialize mode
rospy.init_node("speed_controller")

# Subscribe to the odom topic to get information about the current position and velocity
# of the robot
sub = rospy.Subscriber("/odom", Odometry, newOdom)

# Publish linear and angular velocities to cmd_vel topic
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

# Establish target coordinates
goal = Point()
goal.x = 2
goal.y = 0

# Strategy is to first turn to face the target coordinates
# and then move towards them
while not rospy.is_shutdown():

    # Compute difference between current position and target position
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_x)
    dis = math.sqrt(math.pow(inc_x, 2) + math.pow(inc_y, 2))
    difference_angle = abs(angle_to_goal - theta)
    print("the distance is", round(dis,3))
    msg2 = ("Difference: %s " % difference_angle)
#    rospy.loginfo(msg2)

    # Check whether to turn or move.
    #if abs(angle_to_goal - theta) > 0.1:
    #    speed.linear.x = 0.0
    #    speed.angular.z = 0.3
    #else:
    #    rospy.loginfo("Ready to move")
    #    speed.linear.x = 1.6
    #    speed.angular.z = 0.0

    if dis < 0.5:
        # close but wrong direction
        if abs(difference_angle) >= 0.1 and abs(inc_x) > 0.2 and abs(inc_y) > 0.2:
            speed.linear.x = 0.05
            speed.angular.z = 0.4
            print("")
            print("close but wrong direction")
            print("current coordinates:",round(x,1), round(y,3))
            print("currently linear and angular speed are:", speed.linear.x, speed.angular.z)
            print("current angle to destination is :", round(angle_to_goal, 3))

        # close and right direction
        elif abs(difference_angle) < 0.1 and abs(inc_x) > 0.2 and abs(inc_y) > 0.2:
            speed.linear.x = 0.05
            speed.angular.z = 0
            print("")
            print("close but right direction!")
            print("current coordinates:",round(x,1), round(y,3))
            print("currently linear and angular speed are:\n", speed.linear.x, speed.angular.z)
            print("current angle to destination is:", round(angle_to_goal, 3))

        # you have arrived, stop functioning 
        elif abs(difference_angle) < 0.1 and abs(inc_x) < 0.2 and abs(inc_y) < 0.2:
            speed.linear.x = 0
            speed.angular.z = 0
            print("")
            print("you'r here!")
            print("current coordinates:",round(x,1), round(y,3))
            print("currently linear and angular speed are:\n", speed.linear.x, speed.angular.z)
            print("current angle to destination is:", round(angle_to_goal, 3))

########################################################################

    elif dis >= 0.5:
        # too far but right direction
        if abs(difference_angle) < 0.1:
            #rospy.loginfo("Ready to move")
            speed.linear.x = 0.15
            speed.angular.z = 0.0
            print("")
            print("too far but right direction!")
            print("current coordinates:",round(x,1), round(y,3))
            print("currently linear and angular speed are:", speed.linear.x, speed.angular.z)
            print("current angle to destination is:", round(angle_to_goal, 3))

        # too far and wrong direction
        elif (difference_angle) > 0.1:
            speed.linear.x = 0.15
            speed.angular.z = 0.4
            print("")
            print("too far and wrong direction!")
            print("current coordinates:",round(x,1), round(y,3))
            print("currently linear and angular speed are:", speed.linear.x, speed.angular.z)
            print("current angle to destination is:", round(angle_to_goal, 3))
    else:
        speed.linear.x = 0
        speed.angular.z = 0
        print("something is wrong")

    pub.publish(speed)
    r.sleep()
   
