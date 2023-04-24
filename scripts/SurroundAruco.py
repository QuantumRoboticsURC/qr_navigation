#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist

class SurroundAruco():
    def __init__(self):
        rospy.init_node('surround_ar')    
        self.cmd_vel_publi = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        # self.cmd_vel_publi = rospy.Publisher('/surround_aruco_cmd_vel', Twist, queue_size=1)

        self.t0 = rospy.Time.now().to_sec()  
        self.first_time = True
        self.distance_traveled = 0.0
        self.current_time = None
        self.speed = 0.2
        self.cmd_vel_msg = Twist()  
        self.curr_rotations = 0.0  
        self.rotations = 1.0 

        self.angular_speed = 0.3 
        self.angle_traveled = 0.0

    def move(self):
        self.cmd_vel_msg.linear.x = self.speed
        self.cmd_vel_msg.linear.y = 0
        self.cmd_vel_msg.linear.z = 0
        self.cmd_vel_msg.angular.x = 0
        self.cmd_vel_msg.angular.y = 0
        self.cmd_vel_msg.angular.z = 0

        while self.distance_traveled < 2.0:
            self.cmd_vel_publi.publish(self.cmd_vel_msg)
            self.current_time = rospy.Time.now().to_sec() 
            self.distance_traveled += self.speed * (self.current_time - self.t0)
            self.t0 = self.current_time

        self.cmd_vel_msg.linear.x = 0
        self.distance_traveled = 0.0


    def rotate(self):
        self.t0 = rospy.Time.now().to_sec()
        if self.first_time:
            while (self.angle_traveled > -(math.pi/2.0)):
                self.cmd_vel_msg.angular.z = -self.angular_speed
                self.first_time = False
                self.cmd_vel_publi.publish(self.cmd_vel_msg)
                self.current_time = rospy.Time.now().to_sec()
                self.angle_traveled = -self.angular_speed * (self.current_time - self.t0) 

            self.cmd_vel_msg.angular.z = 0.0
            self.angle_traveled = 0.0
        else:
            while (self.angle_traveled <= math.pi/2.0):
                self.cmd_vel_msg.angular.z = self.angular_speed
                self.cmd_vel_publi.publish(self.cmd_vel_msg)
                self.current_time = rospy.Time.now().to_sec()
                self.angle_traveled = self.angular_speed * (self.current_time - self.t0) 

            self.cmd_vel_msg.angular.z = 0.0
            self.angle_traveled = 0.0
  
    def main(self):
        while not rospy.is_shutdown():   
           if self.curr_rotations < self.rotations:
                self.rotate()
                self.move()
                self.curr_rotations += 0.25

if __name__ == '__main__':
    try:
        surr = SurroundAruco() 
        surr.main()
    except rospy.ROSInterruptException:
        pass 