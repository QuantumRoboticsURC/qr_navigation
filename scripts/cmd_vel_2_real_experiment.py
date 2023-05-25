#!/usr/bin/env python
import rospy
import rospkg
import os
from geometry_msgs.msg import Twist, Pose2D, Pose
from nav_msgs.msg import Odometry
import numpy as np

class PuzzlebotOpenLoopExperiment():
    def __init__(self):        
        rospy.init_node('puzzlebot_experiment')
        #self.p2p_contr = Point2PointController()
        self.linear_vel, self.angular_vel, self.exec_time = (0.5*0.57, 0.0, 10.0)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_msg = Twist()                
        self.first_time = True
        self.inital_time = None                
        #self.nav_topic_d_type = None
        self.rate = rospy.Rate(20.0)

    def publish_vel(self, linear_vel, angular_vel):
        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(self.cmd_vel_msg)                            

    def main(self):
        while not rospy.is_shutdown():                            
            # TODO: implement this                        
            if self.first_time:
                self.inital_time = rospy.get_time()
                self.first_time = False
            else:
                if (rospy.get_time() - self.inital_time) < self.exec_time:
                    self.publish_vel(self.linear_vel, self.angular_vel)
                else:
                    self.publish_vel(0.0, 0.0)                        
            self.rate.sleep()        
        

if __name__ == '__main__':         
    puzz_ol_experiment = PuzzlebotOpenLoopExperiment()                
    puzz_ol_experiment.main()