#!/usr/bin/env python3

"""Made by:
	Leonardo Javier Nava
        navaleonardo40@gmail.com
    Jose Angel del Angel Dominguez
        joseangeldelangel10@gmail.com
Code description:
TO-DO - Cleaning code from follow_gps2
Notes:
"""

import csv
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix
from gps_tranforms import alvinxy as gps_tranforms


class FollowGPS():
    def __init__(self):
        rospy.init_node("follow_gps")
        rospy.Subscriber("/combined_odom", Odometry, self.imu_and_gps_data_callback)
        #self.goal_position_publisher = rospy.Publisher("ublox/gps_goal", NavSatFix, queue_size=1) # publishes goal so it can be displayed on mapviz        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        #self.imu_and_gps_data = NavSatFix()
        self.vel_msg = Twist()

        self.initial_position_ll_2d = (None, None)
        # we dont save initial position in x y meters since inital position will always be 0
        self.current_position_xy_2d = (None, None)
        self.target_postition_xy_2d = (None, None)
        self.current_angle = None
        self.started = False
        self.first_time = True    

        self.angular_kp = 0.3

    def read_target(self):
        file = open("/home/jose/Documents/quantum/quantum_ws/src/qr_navigation/scripts/csv_files/joses_tests.csv") 
        csvreader = csv.reader(file)
        first_coord = list(csvreader)[0]
        target_lat = first_coord[0]
        target_long = first_coord[1]        
        self.target_postition_xy_2d = gps_tranforms.ll2xy( target_lat,
                                                           target_long,
                                                           self.initial_position_ll_2d[0],
                                                           self.initial_position_ll_2d[1])

    def turn_checker_callback(self, data):
        control_node_in_turn = data.data
        if control_node_in_turn == "follow_gps":
            self.started = True
        else:
            self.started = False        

    def imu_and_gps_data_callback(self, data):
        if self.started:
            if self.first_time:
                self.initial_position_ll_2d = (data.pose.pose.position.x, data.pose.pose.position.y)
                self.first_time = False
            else:
                self.current_position_xy_2d = gps_tranforms.ll2xy( data.pose.pose.position.x,
                                                                   data.pose.pose.position.y, 
                                                                   self.initial_position_ll_2d[0],
                                                                   self.initial_position_ll_2d[1])
                self.current_angle = self.calculate_angle( data.pose.pose.orientation )
    
    def calculate_angle(self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)            
        if np.sign(yaw) == -1.0:
            yaw = 2*math.pi + yaw            
        return yaw        
    
    def euclidean_distance_2d(self, p1, p2):
        return math.sqrt( (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 )

    def main(self):
        if not self.first_time:
            angle_error = self.current_angle - math.atan2(self.target_postition_xy_2d[0], self.target_postition_xy_2d[1])
            distance_error = self.euclidean_distance_2d( self.current_position_xy_2d, self.target_postition_xy_2d )
            if abs(angle_error) < 0.6:
                self.vel_msg.angular.z = self.angular_kp*angle_error



if __name__ == "__main__":
    follow_gps = FollowGPS()
    follow_gps.main()