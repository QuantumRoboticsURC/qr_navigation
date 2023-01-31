#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler #eeyyy
import math

class RotateWhileDetectingAr():
    def __init__(self):
        rospy.init_node("rotate_while_detecting_ar")
        rospy.Subscriber("/combined_odom", Odometry, self.imu_pose_callback)
        rospy.Subscriber("/closest_aruco_distance", Point, self.ar_detected_callback, queue_size=1)
        rospy.Subscriber("/rotate_while_detecting_ar_start", Bool, self.start, queue_size=1) 
        self.pub_detected = rospy.Publisher("/ar_detected", Bool, queue_size = 1)
        self.pub_rotate_while_detecting_ar_ended = rospy.Publisher("/rotate_while_detecting_ar_ended", Bool, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher("/rotate_while_detecting_ar_cmd_vel", Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

        self.num_turns = 1.0
        self.new_ar_detected = False        
        self.prev_orientation = None
        self.curr_orientation = None
        self.curr_turns = 0.0
        self.angle_displaced = 0.0
        self.started = False
        self.first_time = True
        self.first_orientation = 0.0
        self.current_orientation = 0.0

    def start(self, data):
        self.started = data.data

    def imu_pose_callback(self, data):
        if self.started:
            if self.first_time:
                self.first_orientation = data.pose.pose.orientation
                self.current_orientation = data.pose.pose.orientation
                self.first_time = False
            else:                
                self.curr_orientation = data.pose.pose.orientation
                
                
                    
    def ar_detected_callback(self, data):        
        self.new_ar_detected = True    

    def calculate_num_turns(self):
        if self.prev_orientation is not None:
            actual_lecture = self.calculate_angle(self.curr_orientation)
            prev_angle = self.calculate_angle(self.prev_orientation)
            self.angle_displaced += abs(actual_lecture - prev_angle)
            self.curr_turns = self.angle_displaced/(2.0*math.pi)
    
    def calculate_angle(self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def main(self):
        # Publish Twist                
        while not rospy.is_shutdown():            
            if not self.first_time:
                print("{}".format(abs(self.calculate_angle(self.first_orientation) - self.calculate_angle(self.current_orientation))))
                print("{} : {}".format(self.calculate_angle(self.first_orientation), self.calculate_angle(self.current_orientation)))
                if self.new_ar_detected:                
                    self.pub_detected.publish(True)
                    self.pub_rotate_while_detecting_ar_ended.publish(True)                            
                    self.cmd_vel_msg.angular.z = 0.0
                elif abs(self.calculate_angle(self.first_orientation) - self.calculate_angle(self.current_orientation)) >= 2.0*math.pi:
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
