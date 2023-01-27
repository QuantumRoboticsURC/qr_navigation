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


class NavigationController():
    def _init_(self):
        # ___ ros atributes initialization ___
        rospy.init_node("matrix_signal_reciever")
        rospy.Subscriber("/closest_aruco_distance", Point, self.closest_aruco_callback, queue_size=1)
        self.command_velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        
        # ___ gains initialization ___
        self.kp_angle_error = 0.00125   # comes from .4[rad/s] = +-320 [max error pixels] * kpae 
        self.kp_distance_error = 0.2    # comes from  1[rad/s] = +-5 [max error meters] * kpde

        # ___ error initialization ___
        self.angle_error = 0
        self.distance_error = 0

        # ___ vel command ___
        self.command_velocity = Twist()

    def closest_aruco_callback(self, data):
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