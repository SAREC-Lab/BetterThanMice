#!/usr/bin/env python3

# import rospy

# from std_msgs.msg import Empty
# from time import time 

# rospy.init_node('reset_odom')

# reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty)

# timer = time()
# while time() - timer < 0.25:
#     print('Resetting Odom')
#     reset_odom.publish(Empty())

import rospy 
import tf 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from time import time

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0 

current_time = rospy.Time.now()
last_time = rospy.Time.now() 
timer = time()

r = rospy.Rate(1.0)
while time() - timer < 0.25:
    current_time = rospy.Time.now()

    odom_quat = tf.transformations.quaternion_from_euler(0,0,0)

    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(x, y, 0), Vector3(0, 0, th))

    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()