#!/usr/bin/env python3

"""Made by:
	José Ángel del Ángel
    joseangeldelangel10@gmail.com
Code description:
TODO - add description
Notes:
"""
import rospy
#from std_msgs.msg import String, Int8, Header
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from gps_tranforms import alvinxy as gps_transforms

class SimulatedPosePublisher():
    def __init__(self):
        # ________ ros atributes initialization ______
        rospy.init_node("simulated_pose_publisher")
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.sim_link_states_callback, queue_size=1)                
        self.pose_msg_pub = rospy.Publisher('/combined_odom', Odometry, queue_size = 1)
        self.pose_msg = Odometry()
        self.robot_pose = Pose()

    def sim_link_states_callback(self, data):
        self.robot_pose = data.pose[1] # since base link will always be the second item in the list

    def main(self):
        while not rospy.is_shutdown():            
            self.pose_msg.header.stamp = rospy.get_rostime()                          
            latitude, longitude = gps_transforms.xy2ll(self.robot_pose.position.x,
                                                        self.robot_pose.position.y,
                                                        19.594558,
                                                        -99.228084)
            self.pose_msg.pose.pose.position.x = latitude
            self.pose_msg.pose.pose.position.y = longitude
            self.pose_msg.pose.pose.position.z = self.robot_pose.position.z
            self.pose_msg.pose.pose.orientation = self.robot_pose.orientation            
            self.pose_msg_pub.publish(self.pose_msg) 

if __name__ == "__main__":
    simulated_pose_publisher = SimulatedPosePublisher()
    simulated_pose_publisher.main()