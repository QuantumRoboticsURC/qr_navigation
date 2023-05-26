#!/usr/bin/env python

import csv
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool
from qr_navigation.srv import *

class ObstacleAvoidanceRoutine():    
    def __init__(self):
        
        rospy.init_node("obstacle_avoidance_routine")
        
        self.vel_msg = Twist()

        self.roll = None

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/combined_odom", Odometry, self.get_rotation)                

        rospy.Service("avoid_obstacle", ObstacleAvoidanceRoutineService, self.avoid_obstacle_response)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)       
        
        self.linear_vel_conversion_factor = 0.57
        self.angular_vel_conversion_factor = 0.4

        self.rate = rospy.Rate(10.0)

    def get_rotation (self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, _, _) = euler_from_quaternion (orientation_list)
        self.roll = math.degrees(roll)        

    def avoid_obstacle_response(self, req):
        if req.case_num == 1:
            roll_sign = self.roll/abs(self.roll)
            #self.roll = 40.0 # TODO DELETE THIS AFTER STANDALONE TESTS ARE DONE
            self.move_by_time_with_gain(10.0, 0.4) # rotate 10 seconds avoiding the incline plane
            self.move_by_time(5.0, roll_sign*(-0.31416), 0.0) # Turn right -90 deg
            self.move_by_time(10.0, 0.0, 0.5) # Go front 5 m
        elif req.case_num == 2 or req.case_num == 3:
            self.move_by_time(5.0, 0.0, -0.2) # Reverse 1 m
            self.move_by_time(5.0, 0.31416, 0.0) # Turn right -90 deg
            self.move_by_time(10.0, 0.0, 0.5) # Go front 5 m     

        return ObstacleAvoidanceRoutineServiceResponse(True)      
  
    def move_by_time(self, time, a_vel, l_vel):
        beginning = rospy.get_time()                
        while True:
            now = rospy.get_time()
            if (now - beginning) < time:
                self.vel_msg.linear.x = l_vel * self.linear_vel_conversion_factor
                self.vel_msg.angular.z = a_vel * self.angular_vel_conversion_factor
                self.vel_pub.publish(self.vel_msg)                
            else:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
                self.vel_pub.publish(self.vel_msg)
                break
            self.rate.sleep()


    def move_by_time_with_gain(self, time, l_vel, kpa = 0.02):
        beginning = rospy.get_time()
        now = rospy.get_time()
        while (now - beginning) < time:
            self.vel_msg.linear.x = l_vel * self.linear_vel_conversion_factor
            self.vel_msg.angular.z = kpa * abs(self.roll)*((self.roll)/abs(self.roll)) * self.angular_vel_conversion_factor
            self.vel_pub.publish(self.vel_msg)
            now = rospy.get_time()
            #self.roll -= 1.0
            print("roll is {r}".format(r = self.roll))
            self.rate.sleep()
        
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.vel_pub.publish(self.vel_msg)
                    

if __name__ == "__main__":
    follow_gps = ObstacleAvoidanceRoutine()    
    while not rospy.is_shutdown():
        rospy.spin()
