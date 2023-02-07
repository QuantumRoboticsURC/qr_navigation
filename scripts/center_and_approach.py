#!/usr/bin/env python3

import rospy
import numpy as np
import time
from nav_helpers import nav_functions
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
        self.center_and_approach_ended_msg = Bool()
        # ___ gains initialization ___
        # self.kp_angle_error = -0.00125   # comes from .4[rad/s] = +-320 [max error pixels] * kpae 
        self.kp_angle_error = 0.1
        self.kp_distance_error = 0.1    # comes from  1[rad/s] = +-5 [max error meters] * kpde
        # ___ error initialization ___
        self.angle_error = 0.0
        self.distance_error = 1.11        
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
        self.distance_error = self.aruco_position.x - 1                

    def main(self):
        while not rospy.is_shutdown():
            if self.started:
                self.command_velocity.linear.x = 0.0
                self.command_velocity.angular.z = 0.0
                self.calculate_error()
                if abs(self.angle_error) > 0.15:
                    candidate_angular_vel = self.angle_error * self.kp_angle_error
                    if ((time.time() - self.overshoot_softener_value_changed_time) >= 1.0) and (self.wheel_overshot_softener < 1.0):
                        self.wheel_overshot_softener += 0.1
                        self.overshoot_softener_value_changed_time = time.time()                
                    if np.sign(candidate_angular_vel) != np.sign(self.prev_angular_velocity):
                        self.wheel_overshot_softener = 0.1
                        self.overshoot_softener_value_changed_time = time.time()            
                    self.wheel_overshot_softener = nav_functions.saturate_signal(self.wheel_overshot_softener, 1.0)                    
                    self.command_velocity.angular.z = nav_functions.saturate_signal(candidate_angular_vel * self.wheel_overshot_softener, 0.5)
                    self.prev_angular_velocity = candidate_angular_vel                                        
                elif self.distance_error > 0.1:
                    candidate_linear_vel = self.distance_error * self.kp_distance_error
                    self.command_velocity.linear.x = nav_functions.saturate_signal(candidate_linear_vel, 0.4)                                        
                    self.center_and_approach_ended_publisher.publish(False)                
                elif self.distance_error <= 0.1:                    
                    self.center_and_approach_ended_publisher.publish(True)                    
                self.command_velocity_publisher.publish(self.command_velocity)

if __name__ == "__main__":
    center_and_approach = CenterAndApproach()
    center_and_approach.main()
