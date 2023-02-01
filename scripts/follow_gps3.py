#!/usr/bin/env python

"""Made by:
	Leonardo Javier Nava
        navaleonardo40@gmail.com
    Jose Angel del Angel Dominguez
        joseangeldelangel10@gmail.com
Code description:
TO-DO - Cleaning code from follow_gps2
Notes:
"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix
from gps_tranforms import alvinxy as gps_tranforms


class FollowGPS():
    def __init__(self):
        rospy.init_node("follow_gps")
        rospy.Subscriber("/combined_odom", Odometry, self.imu_and_gps_data_callback)
        self.goal_position_publisher = rospy.Publisher("ublox/gps_goal", NavSatFix, queue_size=1) # publishes goal so it can be displayed on mapviz        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.imu_and_gps_data = NavSatFix()
        self.vel_msg = Twist()

        self.current_position_2d = (None, None)

    def imu_and_gps_data_callback(self, data):
        pass

    def main(self):
        pass

if __name__ == "__main__":
    follow_gps = FollowGPS()
    follow_gps.main()