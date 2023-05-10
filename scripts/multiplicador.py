#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def get_rotation (msg):
    global vel_z, vel_x
    vel_x = msg.linear.x 
    vel_z = msg.angular.z * 1.0
     
vel_z = 0.0
vel_x = 0.0
rospy.init_node('cmd_vel_multiplier')

sub = rospy.Subscriber ('/center_and_approach_cmd_vel', Twist, get_rotation)
pub = sub = rospy.Publisher ('/mr/cmd_vel', Twist, queue_size=1)

r = rospy.Rate(5)
cmd_vel_msg = Twist()
while not rospy.is_shutdown():
    cmd_vel_msg.angular.z = vel_z
    cmd_vel_msg.linear.x = vel_x
    pub.publish(cmd_vel_msg)	
    r.sleep()
