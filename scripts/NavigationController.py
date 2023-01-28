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
import sys
from geometry.msg import Point, Twist
from std_msgs.msg import Bool

class NavigationController(target_point_type = "gps_only"):
    def __init__(self):
        # ___ ros atributes initialization ___
        rospy.init_node("navigation_controller")
        rospy.Subscriber("/gps_arrived", Bool, self.arrived_to_point_signal_callback, queue_size=1)
        rospy.Subscriber("/center_and_approach_ended", Bool, self.center_and_approach_ended_callback, queue_size=1)
        rospy.Subscriber("/ar_detected", Bool, self.ar_detected_callback, queue_size=1)                
        rospy.Subscriber("/follow_gps_cmd_vel", Twist, self.gps_cmd_vel_calback , queue_size=1)        
        rospy.Subscriber("/center_and_approach_cmd_vel", Twist, self.center_and_approach_cmd_vel_callback, queue_size=1)
        rospy.Subscriber("/rotate_while_detecting_ar_cmd_vel", Twist, self.rotate_while_detecting_ar_cmd_vel_callback, queue_size)                
        self.command_velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)            

        self.gps_arrived = False
        self.ar_detected = False
        self.center_and_approach_ended = False
        self.follow_gps_vel = Twist()
        self.center_and_approach_vel = Twist()
        self.rotate_while_detecting_ar_vel = Twist()    

        self.target_point_type = target_point_type    

    def arrived_to_point_signal_callback(self, data):
        self.gps_arrived = data.data

    def ar_detected_callback(self, data):
        self.ar_detected = data.data

    def center_and_approach_ended_callback(self, data):
        self.center_and_approach_ended = data.data

    def center_and_approach_cmd_vel_callback(self, data):
        self.center_and_approach_vel = data

    def gps_cmd_vel_calback(self, data):
        self.follow_gps_vel = data
    
    def rotate_while_detecting_ar_cmd_vel_callback(self, data):
        self.rotate_while_detecting_ar_vel = data

    def main(self):
        while not rospy.is_shutdown():
            if self.gps_arrived = F

            self.command_velocity_publisher.publish(command)

if __name__ == "__main__":
    if len(sys.argv) > 0:
        target_point_type = sys.argv[1]
        print("target_point_type is {t}".format(t = target_point_type))
    navigation_controller = NavigationController(target_point_type)
    navigation_controller.main()    