#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu 
from geometry_msgs.msg import PoseStamped


latitude = 0
longitude = 0
altitude = 0
imu_x = 0
imu_y = 0
imu_z = 0
imu_w = 0


def gps_callback(data):
    global latitude, longitude, altitude, imu_x, imu_y, imu_z, imu_w
    latitude  = data.latitude            #  // x measurement GPS.
    longitude = data.longitude           #  // y measurement GPS.
    altitude  = data.altitude 
    
def imu_callback(data): 
    global latitude, longitude, altitude, imu_x, imu_y, imu_z, imu_w
    imu_x = data.orientation.x              #// identity quaternion
    imu_y = data.orientation.y             #// identity quaternion
    imu_z = data.orientation.z             #// identity quaternion
    imu_w = data.orientation.w 
    odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size = 1)
    odom_msg = Odometry()
    odom_msg.header.seq = data.header.seq
    odom_msg.header.stamp = rospy.get_rostime()                #  // time of gps measurement
    odom_msg.header.frame_id = data.header.frame_id          #// the tracked robot frame
    odom_msg.pose.pose.position.x = latitude            #  // x measurement GPS.
    odom_msg.pose.pose.position.y = longitude           #  // y measurement GPS.
    odom_msg.pose.pose.position.z = altitude            #  // z measurement GPS.
    odom_msg.pose.pose.orientation.x = imu_x               #// identity quaternion
    odom_msg.pose.pose.orientation.y = imu_y               #// identity quaternion
    odom_msg.pose.pose.orientation.z = imu_z               #// identity quaternion
    odom_msg.pose.pose.orientation.w = imu_w    
    odom_pub.publish(odom_msg)

def odom_publisher():
    global latitude, longitude, altitude, imu_x, imu_y, imu_z, imu_w
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odom_publisher', anonymous=True)

    rospy.Subscriber("ublox/fix", NavSatFix, gps_callback)
    rospy.Subscriber("imu/data", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    odom_publisher()
