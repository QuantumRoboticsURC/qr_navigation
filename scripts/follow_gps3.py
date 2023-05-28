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
import pandas as pd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from constants import PlatformConstants
from gps_tranforms import alvinxy as gps_tranforms
from nav_helpers import nav_functions
from qr_navigation.srv import *

class FollowGPS():
    def __init__(self):
        rospy.init_node("follow_gps")
        
        self.vel_msg = Twist()

        self.gps_target_file = PlatformConstants.GPS_TARGET_CSV_PATH
        self.initial_position_ll_2d = (None, None)
        # we dont save initial position in x y meters since inital position will always be 0
        self.current_position_xy_2d = (None, None)
        self.target_postition_xy_2d = (None, None)
        self.current_angle = None
        self.pitch = None
        self.roll = None
        self.started = False
        self.first_time = True
        self.angular_error_treshold = PlatformConstants.FOLLOW_GPS_ANGULAR_ERROR_TRESHOLD
        self.distance_error_treshold = PlatformConstants.FOLLOW_GPS_LINEAR_ERROR_TRESHOLD                    

        self.angular_kp = PlatformConstants.FOLLOW_GPS_ANGULAR_KP        
        self.linear_kp = PlatformConstants.FOLLOW_GPS_LINEAR_KP

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/combined_odom", Odometry, self.imu_and_gps_data_callback)        
        rospy.Subscriber("/control_node_in_turn", String, self.turn_checker_callback, queue_size=1)

        rospy.Service("Reset_follow_gps", reset_follow_gps, self.callback_reset_follow_gps)

        self.gps_arrived_pub = rospy.Publisher("/gps_arrived", Bool, queue_size = 1)
        self.vel_pub = rospy.Publisher("/follow_gps_cmd_vel", Twist, queue_size=1)    

        self.roll_limit = 30.0
        self.pitch_limit = 30.0
        self.time_limit = 10.0

        self.probable_stuck_point_xy = None
        self.stuck_counter = 0

    

    def callback_reset_follow_gps(self, req):
        self.reset_values()
        print("Follow GPS reseted successfully")
        return reset_follow_gpsResponse(True)

    def reset_values(self):        
        self.vel_msg = Twist()
        self.initial_position_ll_2d = (None, None)        
        self.current_position_xy_2d = (None, None)
        self.target_postition_xy_2d = (None, None)
        self.current_angle = None
        self.started = False
        self.first_time = True

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
        #print("target is: {}".format(self.target_postition_xy_2d))

    def turn_checker_callback(self, data):
        control_node_in_turn = data.data
        if control_node_in_turn == "follow_gps":
            self.started = True
        else:
            self.reset_values()       

    def imu_and_gps_data_callback(self, data):
        #print (data)
        if self.started:
            if self.first_time:
                self.initial_position_ll_2d = (data.pose.pose.position.x, data.pose.pose.position.y)                
                self.current_position_xy_2d = gps_tranforms.ll2xy( data.pose.pose.position.x,
                                                                   data.pose.pose.position.y, 
                                                                   self.initial_position_ll_2d[0],
                                                                   self.initial_position_ll_2d[1])
                self.current_angle = nav_functions.calculate_yaw_angle_deg( data.pose.pose.orientation )
                self.current_angle = (self.current_angle + PlatformConstants.FOLLOW_GPS_IMU_OFFSET)%360.0 # TODO remove this                                                
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
                #print ("current: ", self.current_position_xy_2d)
                self.current_angle = nav_functions.calculate_yaw_angle_deg( data.pose.pose.orientation )
                self.current_angle = (self.current_angle + PlatformConstants.FOLLOW_GPS_IMU_OFFSET)%360.0         

            self.pitch = nav_functions.calculate_pitch_angle_deg( data.pose.pose.orientation )
            self.roll = nav_functions.calculate_roll_angle_deg( data.pose.pose.orientation )
            #print("roll is: {r}, pitch is {p}".format(r = self.roll, p = self.pitch))


    def obstacle_avoidance(self, case):
        rospy.wait_for_service("avoid_obstacle")
        try:
            obstacle_avoidance_service = rospy.ServiceProxy("avoid_obstacle", ObstacleAvoidanceRoutineService)
            response = obstacle_avoidance_service(case)
            print("obstacle avoided")
            return response.success
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
                            
    def main(self):
        while not rospy.is_shutdown():            
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            if self.started and not self.first_time and self.roll != None:
                if self.probable_stuck_point_xy is None:
                    self.probable_stuck_point_xy = (self.current_position_xy_2d[0],self.current_position_xy_2d[1])

                if ( abs(self.roll) > self.roll_limit):
                    print("RISKY ROLL")
                    self.obstacle_avoidance(1)
                elif (self.pitch < -self.pitch_limit):
                    print("RISKY PITCH")
                    self.obstacle_avoidance(2)                
                else:
                    target_vector_minus_robot_vector = ( self.target_postition_xy_2d[0] - self.current_position_xy_2d[0],
                                                        self.target_postition_xy_2d[1] - self.current_position_xy_2d[1]  )
                    target_angle = nav_functions.rad2deg(math.atan2( target_vector_minus_robot_vector[1],target_vector_minus_robot_vector[0]))                
                    angle_error = nav_functions.calculate_angular_error_considering_upper_boundary_lag(target_angle, self.current_angle, "deg", (0.0,360.0))                
                    distance_error = nav_functions.euclidean_distance_single_point_2d( target_vector_minus_robot_vector )                          
                    if distance_error <= self.distance_error_treshold:                                        
                        self.gps_arrived_pub.publish(True)
                        self.started = False
                    elif abs(angle_error) > self.angular_error_treshold:                                    
                        self.vel_msg.angular.z = nav_functions.saturate_signal(self.angular_kp*angle_error, PlatformConstants.FOLLOW_GPS_ANGULAR_SATURATION_VAL)
                    elif distance_error > self.distance_error_treshold:                    
                        self.vel_msg.linear.x = nav_functions.saturate_signal(self.linear_kp*distance_error, PlatformConstants.FOLLOW_GPS_LINEAR_SATURATION_VAL)
                        self.gps_arrived_pub.publish(False)                                    
                    #self.vel_msg.linear.x = .5
                    #self.vel_msg.angular.z = 0.0
                    self.vel_pub.publish(self.vel_msg)
                    
                    if nav_functions.euclidean_distance_point_to_point_2d(self.probable_stuck_point_xy,  self.current_position_xy_2d) <= 2.0:
                        self.stuck_counter += 1
                        #print("current: ", self.current_position_xy_2d)
                        print ("counter: ", self.stuck_counter)
                        #print("eculidean: ", nav_functions.euclidean_distance_point_to_point_2d(self.probable_stuck_point_xy, self.current_position_xy_2d))
                    else:
                        print ("new counter")
                        self.stuck_counter = 0
                        self.probable_stuck_point_xy = self.current_position_xy_2d
                    
                    if self.stuck_counter >= 200 and distance_error >= 10.0 :
                        print("STUCKED , AVOIDING OBSTACLE")                                 
                        self.obstacle_avoidance(2)
                    
            self.rate.sleep()          

if __name__ == "__main__":
    follow_gps = FollowGPS()
    follow_gps.main()
