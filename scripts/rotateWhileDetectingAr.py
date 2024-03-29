#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_helpers import nav_functions
from constants import PlatformConstants
import math
import copy
import numpy as np

class RotateWhileDetectingAr():
    def __init__(self):
        rospy.init_node("rotate_while_detecting_ar")        

        self.cmd_vel_msg = Twist()
        self.num_turns = 0.85
        self.new_ar_detected = False                
        self.curr_turns = 0.0     
        self.angle_displaced = 0.0           
        self.started = False
        self.first_time = True        
        self.current_angle = None
        self.previous_angle = None

        self.reference_frame_z_points_up = PlatformConstants.REFERENCE_FRAME_Z_AXIS_POINTS_UP

        self.rate = rospy.Rate(5.0)

        rospy.Subscriber("/combined_odom", Odometry, self.imu_pose_callback)
        rospy.Subscriber("/closest_aruco_distance", Point, self.ar_detected_callback, queue_size=1)        
        rospy.Subscriber("/control_node_in_turn", String, self.turn_checker_callback, queue_size=1) 

        self.pub_detected = rospy.Publisher("/ar_detected", Bool, queue_size = 1)
        self.pub_rotate_while_detecting_ar_ended = rospy.Publisher("/rotate_while_detecting_ar_ended", Bool, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher("/rotate_while_detecting_ar_cmd_vel", Twist, queue_size=1)                                        

    def turn_checker_callback(self, data):
        control_node_in_turn = data.data
        if control_node_in_turn == "rotate_while_detecting_ar":
            self.started = True
        else:
            self.reset_vaules()        

    def reset_vaules(self):
        self.cmd_vel_msg = Twist()
        self.new_ar_detected = False                
        self.curr_turns = 0.0     
        self.angle_displaced = 0.0           
        self.started = False
        self.first_time = True        
        self.current_angle = None
        self.previous_angle = None                    

    def imu_pose_callback(self, data):
        if self.started:
            if self.first_time:                
                self.previous_angle = self.calculate_angle(data.pose.pose.orientation)
                self.current_angle = self.calculate_angle(data.pose.pose.orientation)                
                self.first_time = False
            else:                
                self.current_angle = self.calculate_angle(data.pose.pose.orientation)
                
    def ar_detected_callback(self, data):        
        self.new_ar_detected = True

    def calculate_num_turns(self):
        if not self.reference_frame_z_points_up:
            if PlatformConstants.ROTATE_WHILE_DETECTING_AR_ANGULAR_VEL <= 0.0:
                # consider that for q-mars on 2023 season a positive angular vel 
                # produces a clockwise turn and not a counterclockwise turn 
                if self.current_angle > self.previous_angle:        
                    self.angle_displaced += abs(self.current_angle - self.previous_angle)
                elif (self.current_angle + math.pi) < self.previous_angle:
                    # angle restarted
                    self.angle_displaced += abs(self.current_angle - self.previous_angle + 2.0*math.pi)
            else:
                if self.current_angle < self.previous_angle:        
                    self.angle_displaced += abs(self.current_angle - self.previous_angle)
                elif (self.previous_angle + math.pi) < self.current_angle:
                    # angle restarted
                    self.angle_displaced += abs(self.current_angle - self.previous_angle - 2.0*math.pi)
        else:
            if PlatformConstants.ROTATE_WHILE_DETECTING_AR_ANGULAR_VEL >= 0.0:
                # consider that for q-mars on 2023 season a positive angular vel 
                # produces a clockwise turn and not a counterclockwise turn 
                if self.current_angle > self.previous_angle:        
                    self.angle_displaced += abs(self.current_angle - self.previous_angle)
                elif (self.current_angle + math.pi) < self.previous_angle:
                    # angle restarted
                    self.angle_displaced += abs(self.current_angle - self.previous_angle + 2.0*math.pi)
            else:
                if self.current_angle < self.previous_angle:        
                    self.angle_displaced += abs(self.current_angle - self.previous_angle)
                elif (self.previous_angle + math.pi) < self.current_angle:
                    # angle restarted
                    self.angle_displaced += abs(self.current_angle - self.previous_angle - 2.0*math.pi)

        print("angle displaced: {a}".format(a = self.angle_displaced))
        self.curr_turns = self.angle_displaced/(2.0*math.pi)

    def calculate_angle(self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = nav_functions.euler_from_quaternion(orientation_list)            
        yaw = nav_functions.angle_to_only_possitive(yaw)            
        return yaw        

    def main(self):        
        while not rospy.is_shutdown():            
            if self.started and (self.previous_angle is not None):
                self.calculate_num_turns()
                self.previous_angle = self.current_angle                                               
                if self.new_ar_detected:                
                    self.pub_detected.publish(True)
                    self.pub_rotate_while_detecting_ar_ended.publish(True)                    
                    self.cmd_vel_msg.angular.z = 0.0
                    self.started = False
                elif self.curr_turns >= self.num_turns:
                    self.pub_detected.publish(False)
                    self.pub_rotate_while_detecting_ar_ended.publish(True)                                                                     
                    self.cmd_vel_msg.angular.z = 0.0
                    self.started = False
                else:                
                    self.pub_detected.publish(False)
                    self.pub_rotate_while_detecting_ar_ended.publish(False)                    
                    self.cmd_vel_msg.angular.z = PlatformConstants.ROTATE_WHILE_DETECTING_AR_ANGULAR_VEL
                self.cmd_vel_pub.publish(self.cmd_vel_msg)            
            self.rate.sleep()


if __name__ == "__main__":
    rotate = RotateWhileDetectingAr()
    rotate.main()
