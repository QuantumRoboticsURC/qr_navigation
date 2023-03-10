#!/usr/bin/env python3

import rospy
import numpy as np
import time
from nav_helpers import nav_functions
from scripts.constants import PlatformConstants
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point, Twist


class CenterAndApproach():
    def __init__(self):
        # ___ ros atributes initialization ___
        rospy.init_node("center_and_approach")
        rospy.Subscriber("/closest_aruco_distance", Point, self.aruco_position_callback, queue_size=1)
        rospy.Subscriber("/control_node_in_turn", String, self.turn_checker_callback, queue_size=1)
        self.center_and_approach_ended_publisher = rospy.Publisher('/center_and_approach_ended', Bool, queue_size=1)
        self.command_velocity_publisher = rospy.Publisher('/center_and_approach_cmd_vel', Twist, queue_size=1)        
        # ___ gains initialization ___
        # self.kp_angle_error = -0.00125   # comes from .4[rad/s] = +-320 [max error pixels] * kpae 
        self.kp_angle_error = PlatformConstants.CENTER_AND_APPROACH_ANGULAR_KP
        self.kp_distance_error = PlatformConstants.CENTER_AND_APPROACH_LINEAR_KP    # comes from  1[rad/s] = +-5 [max error meters] * kpde
        # ___ error initialization ___
        self.angle_error = 0.0
        self.distance_error = PlatformConstants.CENTER_AND_APPROACH_LINEAR_SET_POINT + PlatformConstants.CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD + 0.01        
        # ___ vel command ___
        self.command_velocity = Twist()
        self.prev_angular_velocity = 0.0
        self.wheel_overshot_softener = 1.0
        self.overshoot_softener_value_changed_time = 0.0
        
        self.aruco_position = Point()
        self.started = False        

    def turn_checker_callback(self, data):
        control_node_in_turn = data.data
        if control_node_in_turn == "center_and_approach":
            self.started = True
        else:
            self.started = False

    def aruco_position_callback(self, data):        
        self.aruco_position = data 

    def calculate_error(self):
        self.angle_error = self.aruco_position.y  
        self.distance_error = self.aruco_position.x - PlatformConstants.CENTER_AND_APPROACH_LINEAR_SET_POINT                

    def main(self):
        while not rospy.is_shutdown():
            if self.started:
                self.prev_run_time = time.time()

                self.command_velocity.linear.x = 0.0
                self.command_velocity.angular.z = 0.0
                self.calculate_error()
                if abs(self.angle_error) > PlatformConstants.CENTER_AND_APPROACH_ANGULAR_ERROR_TRESHOLD:
                    candidate_angular_vel = self.angle_error * self.kp_angle_error
                    if ((time.time() - self.overshoot_softener_value_changed_time) >= 1.0) and (self.wheel_overshot_softener < 1.0):
                        self.wheel_overshot_softener += 0.1
                        self.overshoot_softener_value_changed_time = time.time()                
                    if np.sign(candidate_angular_vel) != np.sign(self.prev_angular_velocity):
                        self.wheel_overshot_softener = 0.1
                        self.overshoot_softener_value_changed_time = time.time()            
                    self.wheel_overshot_softener = nav_functions.saturate_signal(self.wheel_overshot_softener, 1.0)                    
                    self.command_velocity.angular.z = nav_functions.saturate_signal(candidate_angular_vel * self.wheel_overshot_softener, PlatformConstants.CENTER_AND_APPROACH_ANGULAR_SATURATION_VAL)
                    self.prev_angular_velocity = candidate_angular_vel                                        
                elif self.distance_error > PlatformConstants.CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD:
                    candidate_linear_vel = self.distance_error * self.kp_distance_error
                    self.command_velocity.linear.x = nav_functions.saturate_signal(candidate_linear_vel, PlatformConstants.CENTER_AND_APPROACH_LINEAR_SATURATION_VAL)                                        
                    self.center_and_approach_ended_publisher.publish(False)                
                elif self.distance_error <= PlatformConstants.CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD:                    
                    self.center_and_approach_ended_publisher.publish(True)                    
                self.command_velocity_publisher.publish(self.command_velocity)


                # here we predict the next aruco pose so that if the camera lost arucos sight control still works
                current_run_time = time.time()
                current_angle_between_robot_and_aruco = nav_functions.angle_to_only_possitive( np.arctan2(self.aruco_position.y, self.aruco_position.x) )
                next_angle_between_robot_and_aruco = current_angle_between_robot_and_aruco - self.command_velocity.angular.z*(current_run_time - self.prev_run_time)
                next_aruco_x_position = self.aruco_position.x - self.command_velocity.linear.x*(current_run_time - self.prev_run_time) 
                next_aruco_y_position = np.tan(next_angle_between_robot_and_aruco)*next_aruco_x_position             
                self.aruco_position.x = next_aruco_x_position
                self.aruco_position.y = next_aruco_y_position  
                
                

if __name__ == "__main__":
    center_and_approach = CenterAndApproach()
    center_and_approach.main()
