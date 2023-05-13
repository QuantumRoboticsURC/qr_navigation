#!/usr/bin/env python3

import rospy
import numpy as np
import time
from nav_helpers import nav_functions
from constants import PlatformConstants
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry


class CenterAndApproach():
    def __init__(self):
        # ___ ros atributes initialization ___
        rospy.init_node("center_and_approach")

        # ___ gains initialization ___        
        self.kp_angle_error = PlatformConstants.CENTER_AND_APPROACH_ANGULAR_KP # TODO check kp angle
        self.kp_distance_error = PlatformConstants.CENTER_AND_APPROACH_LINEAR_KP    # comes from  1[rad/s] = +-5 [max error meters] * kpde

        self.rate_float = 5.0
        self.rate = rospy.Rate(self.rate_float)
        # ___ error initialization ___
        self.angle_error = None
        self.distance_error = None        
        # ___ vel command ___
        self.command_velocity = Twist()
        self.prev_angular_velocity = 0.0
        self.wheel_overshot_softener = 1.0
        self.overshoot_softener_value_changed_time = 0.0
        
        self.aruco_position = None
        self.started = False        
        self.arrived_counter = 0                        

        self.last_aruco_position_message_time = None

        rospy.Subscriber("/closest_aruco_distance", Point, self.aruco_position_callback, queue_size=1)
        rospy.Subscriber("/control_node_in_turn", String, self.turn_checker_callback, queue_size=1)        
        self.center_and_approach_ended_publisher = rospy.Publisher('/center_and_approach_ended', Bool, queue_size=1)
        self.command_velocity_publisher = rospy.Publisher('/center_and_approach_cmd_vel', Twist, queue_size=1)
                                
    def turn_checker_callback(self, data):
        control_node_in_turn = data.data
        if control_node_in_turn == "center_and_approach":
            self.started = True
        else:            
            self.reset_values()

    def reset_values(self):
        self.angle_error = None
        self.distance_error = None
        self.command_velocity = Twist()
        self.prev_angular_velocity = 0.0
        self.wheel_overshot_softener = 1.0
        self.overshoot_softener_value_changed_time = 0.0        
        self.aruco_position = None
        self.started = False        
        self.arrived_counter = 0               
        self.last_aruco_position_message_time = None

    def aruco_position_callback(self, data):        
        print("NEW ARUCO POSITION")
        self.aruco_position = data
        self.last_aruco_position_message_time = rospy.get_time()
        if self.started:
            self.calculate_error() 

    def calculate_error(self):        
        self.angle_error = np.arctan(self.aruco_position.y / self.aruco_position.x)  
        self.distance_error = self.aruco_position.x - PlatformConstants.CENTER_AND_APPROACH_LINEAR_SET_POINT                 

    def main(self):
        while not rospy.is_shutdown():
            if self.started and self.angle_error is not None and self.distance_error is not None:
                print("angle_error: {ae}, distance_error: {de}".format(ae=self.angle_error,de = self.distance_error))                
                try:                    
                    self.command_velocity.linear.x = 0.0
                    self.command_velocity.angular.z = 0.0
                    #self.calculate_error()
                    if self.arrived_counter >= 5:
                        self.center_and_approach_ended_publisher.publish(True)

                    if abs(self.angle_error) > PlatformConstants.CENTER_AND_APPROACH_ANGULAR_ERROR_TRESHOLD:
                        """
                        candidate_angular_vel = self.angle_error * self.kp_angle_error
                        if ((time.time() - self.overshoot_softener_value_changed_time) >= 1.0) and (self.wheel_overshot_softener < 1.0):
                            self.wheel_overshot_softener += 0.1
                            self.overshoot_softener_value_changed_time = time.time()                
                        if np.sign(candidate_angular_vel) != np.sign(self.prev_angular_velocity):
                            self.wheel_overshot_softener = 0.1
                            self.overshoot_softener_value_changed_time = time.time()            
                        self.wheel_overshot_softener = nav_functions.saturate_signal(self.wheel_overshot_softener, 1.0)                    
                        """
                        self.command_velocity.angular.z = nav_functions.saturate_signal(self.angle_error*self.kp_angle_error, PlatformConstants.CENTER_AND_APPROACH_ANGULAR_SATURATION_VAL)
                        #self.prev_angular_velocity = candidate_angular_vel                                                                
                    elif self.distance_error > PlatformConstants.CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD:
                        candidate_linear_vel = self.distance_error * self.kp_distance_error
                        self.command_velocity.linear.x = nav_functions.saturate_signal(candidate_linear_vel, PlatformConstants.CENTER_AND_APPROACH_LINEAR_SATURATION_VAL)                                        
                        self.arrived_counter = 0
                        #self.center_and_approach_ended_publisher.publish(False)                
                    elif self.distance_error <= PlatformConstants.CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD:                    
                        #self.center_and_approach_ended_publisher.publish(True)                    
                        self.arrived_counter += 1
                    self.command_velocity_publisher.publish(self.command_velocity)


                    if self.distance_error > PlatformConstants.CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD:                        
                        print("applying tracker")
                        # here we predict the next aruco pose so that if the camera lost arucos sight control still works                    
                        delta_t = 1.0/self.rate_float
                        if abs(self.angle_error) > PlatformConstants.CENTER_AND_APPROACH_ANGULAR_ERROR_TRESHOLD:
                            print("APPLING TRACKER ANGULAR")
                            current_angle_between_robot_and_aruco = np.arctan(self.aruco_position.y / self.aruco_position.x)
                            #current_angular_vel = (self.current_angle - self.previous_angle)/(delta_t)
                            current_angular_vel = PlatformConstants.CENTER_AND_APPROACH_DINAMIC_MODEL_SCALING_ANGULAR_VEL*(self.command_velocity.angular.z)
                            current_distance_to_aruco = np.sqrt( (self.aruco_position.x)**2 + (self.aruco_position.y)**2 )
                            next_aruco_angle = current_angle_between_robot_and_aruco - current_angular_vel*delta_t
                            #print()                            
                            self.aruco_position.x = np.cos(next_aruco_angle)*current_distance_to_aruco
                            self.aruco_position.y = np.sin(next_aruco_angle)*current_distance_to_aruco
                            self.calculate_error()
                        elif self.distance_error > PlatformConstants.CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD:        
                            print("APPLING TRACKER LINEAR")
                            #current_angle_between_robot_and_aruco = np.arctan(self.aruco_position.y / self.aruco_position.x)                
                            #current_linear_vel = PlatformConstants.CENTER_AND_APPROACH_DINAMIC_MODEL_SCALING_LINEAR_VEL*(self.command_velocity.linear.x)
                            next_aruco_x_position = self.aruco_position.x - (self.command_velocity.linear.x*delta_t)
                            #next_aruco_x_position = self.aruco_position.x - (current_linear_vel*delta_t)*np.cos(current_angle_between_robot_and_aruco)
                            #next_aruco_y_position = self.aruco_position.y + (current_linear_vel*delta_t)*np.sin(current_angle_between_robot_and_aruco)                             
                            self.aruco_position.x = next_aruco_x_position
                            #self.aruco_position.y = next_aruco_y_position
                            self.calculate_error()                        

                except AttributeError:
                    pass                
            self.rate.sleep()    

if __name__ == "__main__":
    center_and_approach = CenterAndApproach()
    center_and_approach.main()
