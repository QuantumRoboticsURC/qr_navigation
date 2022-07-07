#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import time

roll = 0
pitch = 0
yaw = 0
target = 90
kp=-0.1
sat = 0.1

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print (yaw*180/math.pi)
    #time.sleep(1)

rospy.init_node('rotate_with_angle')
sub = rospy.Subscriber ('/combined_odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10)
command = Twist()

while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    target_rad = target*math.pi/180
    error = target_rad -yaw
    if abs(error) < 0.04:
	error = 0 
    command.angular.z =max(-sat, min(kp * (error),sat))
    pub.publish(command)
    print("target: ", target, ", current angle: ", yaw/math.pi*180, ", angular_z output: ", command.angular.z)
    r.sleep()
