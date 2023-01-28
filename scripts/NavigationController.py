"""
Made by:Jose Angel del Angel Dominguez	
	joseangeldelangel10@gmail.com

Modified (DD/MM/YY): 

Code description:
1. 

Notes:
- 
"""

import rospy
from geometry.msg import Point, Twist
from std_msgs.msg import Bool

class NavigationController():
    def __init__(self):
        # ___ ros atributes initialization ___
        rospy.init_node("matrix_signal_reciever")
        rospy.Subscriber("/gps_arrived", Bool, self.arrived_to_point_signal_callback, queue_size=1)
        rospy.Subscriber("/follow_gps_cmd_vel", Bool, self.arrived_to_point_signal_callback, queue_size=1)
        self.command_velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)        

    def arrived_to_point_signal_callback(self, data):
        self.angle_error = data.y + 320
        self.distance_error = data.x - 1

    def main(self):
        while not rospy.is_shutdown():
            if abs(self.angle_error) > 15:  
                self.command_velocity.linear.x = 0.0
                self.command_velocity.angular.z = self.angle_error * self.kp_angle_error
            
            else:
                self.command_velocity.linear.x = self.distance_error * self.kp_distance_error
                self.command_velocity.angular.z = 0.0

            self.command_velocity_publisher.publish(command)

if _name_ == "_main_":
    center_and_approach = CenterAndApproach()
    center_and_approach.main()