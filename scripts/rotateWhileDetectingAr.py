#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler #eeyyy
import math
import copy
import numpy as np

class RotateWhileDetectingAr():
    def __init__(self):
        rospy.init_node("rotate_while_detecting_ar")
        rospy.Subscriber("/combined_odom", Odometry, self.imu_pose_callback)
        rospy.Subscriber("/closest_aruco_distance", Point, self.ar_detected_callback, queue_size=1)
        #rospy.Subscriber("/rotate_while_detecting_ar_start", Bool, self.start, queue_size=1) 
        rospy.Subscriber("/control_node_in_turn", String, self.turn_checker_callback, queue_size=1) 

        self.pub_detected = rospy.Publisher("/ar_detected", Bool, queue_size = 1)
        self.pub_rotate_while_detecting_ar_ended = rospy.Publisher("/rotate_while_detecting_ar_ended", Bool, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher("/rotate_while_detecting_ar_cmd_vel", Twist, queue_size=1)
        
        self.cmd_vel_msg = Twist()
        self.num_turns = 1.0
        self.new_ar_detected = False                
        self.curr_turns = 0.0        
        self.error_treshold = 0.2 
        self.started = False
        self.first_time = True
        self.first_angle = None
        self.current_angle = None                

    def turn_checker_callback(self, data):
        control_node_in_turn = data.data
        if control_node_in_turn == "rotate_while_detecting_ar":
            self.started = True
        else:
            self.started = False                    

    def imu_pose_callback(self, data):
        if self.started:
            if self.first_time:
                self.first_angle = self.calculate_angle(data.pose.pose.orientation)
                self.current_angle = self.calculate_angle(data.pose.pose.orientation)
                self.first_time = False
            else:                
                self.current_angle = self.calculate_angle(data.pose.pose.orientation)                
                
    def ar_detected_callback(self, data):        
        self.new_ar_detected = True

    def calculate_angle(self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)            
        if np.sign(yaw) == -1.0:
            yaw = 2*math.pi + yaw            
        return yaw        

    def main(self):
        # Publish Twist                
        while not rospy.is_shutdown():            
            if (self.current_angle is not None):                                
                if self.new_ar_detected:                
                    self.pub_detected.publish(True)
                    self.pub_rotate_while_detecting_ar_ended.publish(True)                            
                    self.cmd_vel_msg.angular.z = 0.0
                elif abs(self.current_angle - self.first_angle) >= (2.0*math.pi - self.error_treshold):
                    self.pub_detected.publish(False)
                    self.pub_rotate_while_detecting_ar_ended.publish(True)                            
                    self.cmd_vel_msg.angular.z = 0.0
                else:                
                    self.pub_detected.publish(False)
                    self.pub_rotate_while_detecting_ar_ended.publish(False)
                    self.cmd_vel_msg.angular.z = 0.2
                self.cmd_vel_pub.publish(self.cmd_vel_msg)                  


if __name__ == "__main__":
    rotate = RotateWhileDetectingAr()
    rotate.main()
