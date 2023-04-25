#!/usr/bin/env python3

"""Made by:
	José Ángel del Ángel
    joseangeldelangel10@gmail.com
Code description:
TODO - add description
Notes:
"""
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from gps_tranforms import alvinxy as gps_transforms

class SimulatedPosePublisher():
    def __init__(self):
        # ________ ros atributes initialization ______
        rospy.init_node("simulated_pose_publisher")
        rospy.Subscriber("/tf", TFMessage, self.sim_tf_callback, queue_size=1)                
        self.pose_msg_pub = rospy.Publisher('/combined_odom', Odometry, queue_size = 1)
        self.pose_msg = Odometry()
        self.robot_pose = Pose()

    def sim_tf_callback(self, data):
        self.robot_pose.position.x = data.transforms[0].transform.translation.x
        self.robot_pose.position.y = data.transforms[0].transform.translation.y
        self.robot_pose.position.z = data.transforms[0].transform.translation.z
        self.robot_pose.orientation = data.transforms[0].transform.rotation

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