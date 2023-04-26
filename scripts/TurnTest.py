#!/usr/bin/env python3

"""Made by:    
    Jose Angel del Angel Dominguez
        joseangeldelangel10@gmail.com
Code description:
TO-DO - Cleaning code from follow_gps2
Notes:
"""

import math
import rospy
import numpy as np
import pandas as pd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from constants import PlatformConstants
from gps_tranforms import alvinxy as gps_tranforms
from nav_helpers import nav_functions

class FollowGPS():
    def __init__(self):
        rospy.init_node("follow_gps")
        rospy.Subscriber("/combined_odom", Odometry, self.imu_and_gps_data_callback)
        rospy.Subscriber("/control_node_in_turn", String, self.turn_checker_callback, queue_size=1) 
        
        self.gps_target_file = PlatformConstants.GPS_TARGET_CSV_PATH                                        
        self.initial_position_ll_2d = (None, None)
        # we dont save initial position in x y meters since inital position will always be 0
        self.current_position_xy_2d = (None, None)
        self.target_postition_xy_2d = (None, None)
        self.current_angle = None
        self.started = False
        self.first_time = True            

        self.rate = rospy.Rate(20)

    def reset_values(self):                
        self.initial_position_ll_2d = (None, None)        
        self.current_position_xy_2d = (None, None)
        self.target_postition_xy_2d = (None, None)
        self.current_angle = None
        self.started = False
        self.first_time = True

    def turn_checker_callback(self, data):
        control_node_in_turn = data.data
        if control_node_in_turn == "follow_gps":
            self.started = True
        else:
            self.reset_values() 

    def read_target(self):        
        df = pd.read_csv(self.gps_target_file, index_col=False)        
        target_lat = float( df["latitude"][0] )
        target_long = float( df["longitude"][0] )
        new_target_position_xy_2d = gps_tranforms.ll2xy( target_lat,
                                                target_long,
                                                self.initial_position_ll_2d[0],
                                                self.initial_position_ll_2d[1])
        if new_target_position_xy_2d != self.target_postition_xy_2d:
            self.target_postition_xy_2d = new_target_position_xy_2d
        else:
            raise Exception("target hasn't been updated")                         

    def imu_and_gps_data_callback(self, data):
        if self.started:
            if self.first_time:
                self.initial_position_ll_2d = (data.pose.pose.position.x, data.pose.pose.position.y)                
                self.current_position_xy_2d = gps_tranforms.ll2xy( data.pose.pose.position.x,
                                                                   data.pose.pose.position.y, 
                                                                   self.initial_position_ll_2d[0],
                                                                   self.initial_position_ll_2d[1])
                self.current_angle = nav_functions.calculate_yaw_angle( data.pose.pose.orientation )
                                
                while True:
                    try:
                        self.read_target()
                        break
                    except:
                        pass
                self.first_time = False
            else:
                self.current_position_xy_2d = gps_tranforms.ll2xy( data.pose.pose.position.x,
                                                                   data.pose.pose.position.y, 
                                                                   self.initial_position_ll_2d[0],
                                                                   self.initial_position_ll_2d[1])
                self.current_angle = nav_functions.calculate_yaw_angle( data.pose.pose.orientation )
                
                
            print("CURRENT_ANGLE: {}".format(self.current_angle))
    
    def main(self):
        while not rospy.is_shutdown():                        
            if self.started and not self.first_time:
                target_vector_minus_robot_vector = ( self.target_postition_xy_2d[0] - self.current_position_xy_2d[0],
                                                     self.target_postition_xy_2d[1] - self.current_position_xy_2d[1]  )
                angle_error = math.atan2( target_vector_minus_robot_vector[1], target_vector_minus_robot_vector[0]) - self.current_angle
                print("ANGLE ERROR IS : {a}".format(a = angle_error))
                distance_error = nav_functions.euclidean_distance_single_point_2d( target_vector_minus_robot_vector )                                          

            self.rate.sleep()          

if __name__ == "__main__":
    follow_gps = FollowGPS()
    follow_gps.main()
