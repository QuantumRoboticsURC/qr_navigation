#!/usr/bin/env python3

"""	José Ángel del Ángel
    joseangeldelangel10@gmail.com

Modified (15/12/2022): 

Code description:

Notes:

"""

import rospy
import sys
import pandas as pd
import sched, time
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Int8, String
from qr_navigation.srv import set_target,set_targetRequest,set_targetResponse
from gps_tranforms import alvinxy as gps_transforms
from constants import PlatformConstants

class NavigationController():
    def __init__(self):
        # ___ ros atributes initialization ___
        rospy.init_node("navigation_controller")
        rospy.Subscriber("/gps_arrived", Bool, self.arrived_to_point_signal_callback, queue_size=1)
        rospy.Subscriber("/rotate_while_detecting_ar_ended", Bool, self.rotate_while_detecting_ar_ended_callback, queue_size=1)
        rospy.Subscriber("/center_and_approach_ended", Bool, self.center_and_approach_ended_callback, queue_size=1)
        rospy.Subscriber("/ar_detected", Bool, self.ar_detected_callback, queue_size=1)                
        rospy.Subscriber("/follow_gps_cmd_vel", Twist, self.gps_cmd_vel_calback , queue_size=1)        
        rospy.Subscriber("/rotate_while_detecting_ar_cmd_vel", Twist, self.rotate_while_detecting_ar_cmd_vel_callback, queue_size = 1)                
        rospy.Subscriber("/center_and_approach_cmd_vel", Twist, self.center_and_approach_cmd_vel_callback, queue_size=1)        
        self.control_node_in_turn_pub = rospy.Publisher('/control_node_in_turn', String, queue_size=1)
        self.command_velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.matrix_signal_publisher = rospy.Publisher('/matrix_signal', Int8, queue_size=1)

        self.gps_target_file = PlatformConstants.GPS_TARGET_CSV_PATH
        self.gps_arrived = False
        self.ar_detected = False
        self.rotate_while_detecting_ar_ended = False        
        self.center_and_approach_ended = False
        self.block_new_follow_gps_data = False
        self.block_new_rotate_while_detecting_ar_data = False
        self.block_new_ar_detected_data = False
        self.follow_gps_vel = Twist()
        self.center_and_approach_vel = Twist()
        self.rotate_while_detecting_ar_vel = Twist()            
        self.stop_vel = Twist() 
        self.stop_vel.linear.x = 0.0
        self.stop_vel.angular.z = 0.0
        self.matrix_signal_msg = Int8()
        self.target_latitude = None
        self.target_longitude = None
        self.snail_trayectory_gps_points = []
        self.snail_trayectory_index = -1
        self.autonomous_navigation_ended = False
        
        #TODO - enable the next user input functions to work with arrow keys and control copy 
        self.get_gps_target()
        self.get_target_point_type()
        self.set_gps_target(self.target_latitude, self.target_longitude)        
        self.generate_snail_trayectory_points()
        
        print("target is {t}".format(t = (self.target_latitude, self.target_longitude)))
        print("target type is {t}".format(t = self.target_point_type))

    def rotate_while_detecting_ar_ended_callback(self, data):
        if not self.block_new_rotate_while_detecting_ar_data:
            self.rotate_while_detecting_ar_ended = data.data
            if self.rotate_while_detecting_ar_ended:
                print("rotate_while_detectin_ar_ended!!!")
                self.block_new_rotate_while_detecting_ar_data = True 

    def arrived_to_point_signal_callback(self, data):
        if not self.block_new_follow_gps_data:
            self.gps_arrived = data.data
            if self.gps_arrived:
                print("gps_arrived!!!")
                self.block_new_follow_gps_data = True

    def ar_detected_callback(self, data):
        if not self.block_new_ar_detected_data:        
            self.ar_detected = data.data
            if self.ar_detected:
                print("aruco_detected!!!")                           
                self.block_new_ar_detected_data = True     
        
    def center_and_approach_ended_callback(self, data):
        self.center_and_approach_ended = data.data

    def center_and_approach_cmd_vel_callback(self, data):
        self.center_and_approach_vel = data

    def gps_cmd_vel_calback(self, data):
        self.follow_gps_vel = data
    
    def rotate_while_detecting_ar_cmd_vel_callback(self, data):
        self.rotate_while_detecting_ar_vel = data

    def xy2ll_simplyfied_for_snail_generation(self, cord):
        return gps_transforms.xy2ll(cord[0], cord[1], self.target_latitude, self.target_longitude)
    
    def generate_snail_trayectory_points(self, num_turns = 2):        
        if self.target_point_type == "gps_and_post" or self.target_point_type == "gps_and_post":
            origin = (0,0)
            cords_list = [origin]
            current_cord = origin
            current_turn = 1
            disatnce_between_each_sanil_point = 5
            while current_cord != (origin[0] + num_turns*disatnce_between_each_sanil_point, origin[1] -num_turns*disatnce_between_each_sanil_point):
                
                if current_cord == (origin[0] + current_turn*disatnce_between_each_sanil_point, origin[1] -current_turn*disatnce_between_each_sanil_point):
                    current_turn += 1
                
                if  (current_cord[1] - disatnce_between_each_sanil_point >= origin[1] -current_turn*disatnce_between_each_sanil_point and
                (current_cord[0], current_cord[1] - disatnce_between_each_sanil_point) not in cords_list ):
                    current_cord = (current_cord[0], current_cord[1] - disatnce_between_each_sanil_point)
                    cords_list.append(current_cord)
                elif (current_cord[0] - disatnce_between_each_sanil_point >= origin[0] -current_turn*disatnce_between_each_sanil_point and
                (current_cord[0] - disatnce_between_each_sanil_point, current_cord[1]) not in cords_list ):
                    current_cord = (current_cord[0] - disatnce_between_each_sanil_point, current_cord[1])
                    cords_list.append(current_cord)
                elif (current_cord[1] + disatnce_between_each_sanil_point <= origin[1] + current_turn*disatnce_between_each_sanil_point and
                (current_cord[0], current_cord[1] + disatnce_between_each_sanil_point) not in cords_list ):
                    current_cord = (current_cord[0], current_cord[1] + disatnce_between_each_sanil_point)
                    cords_list.append(current_cord)
                elif (current_cord[0] + disatnce_between_each_sanil_point <= origin[0] + disatnce_between_each_sanil_point*current_turn and
                (current_cord[0] + disatnce_between_each_sanil_point, current_cord[1]) not in cords_list ):
                    current_cord = (current_cord[0] + disatnce_between_each_sanil_point, current_cord[1])
                    cords_list.append(current_cord)
            
            cords_list.pop(0)
            self.snail_trayectory_gps_points = list(map(self.xy2ll_simplyfied_for_snail_generation, cords_list))

    def set_gps_target(self, lat, lon):
        df  = pd.DataFrame.from_dict({"latitude":[lat], "longitude":[lon]})
        df.to_csv(self.gps_target_file, index=False)        

    def get_target_point_type(self):
        get_target_function = rospy.ServiceProxy('get_target', set_target)
        result = get_target_function
        self.target_point_type=result.mode2

    def get_gps_target(self):
        get_target_function = rospy.ServiceProxy('get_target', set_target)
        result = get_target_function
        self.target_latitude = result.latitud2
        self.target_latitude = result.longitud2

    def unblock_new_follow_gps_and_rotate_while_detecting_ar_data(self):
        self.block_new_follow_gps_data = False
        self.block_new_rotate_while_detecting_ar_data = False 

    def main(self):
        while not rospy.is_shutdown():
            if self.autonomous_navigation_ended:
                print("SUCCESS: Navigation completed")
                break
            else:
                if self.gps_arrived == False:
                    self.control_node_in_turn_pub.publish("follow_gps")
                    self.command_velocity_publisher.publish(self.follow_gps_vel)
                    self.matrix_signal_msg.data = 2 # change matrix to red 
                    self.matrix_signal_publisher.publish(self.matrix_signal_msg)                
                else:
                    if self.target_point_type == "gps_only":
                        self.command_velocity_publisher.publish(self.stop_vel)
                        self.matrix_signal_msg.data = 3 # change matrix to green
                        self.matrix_signal_publisher.publish(self.matrix_signal_msg)
                        self.autonomous_navigation_ended = True
                        # THIS IS THE END OF ROUTINE FOR GPS_ONLY
                    elif self.target_point_type == "gps_and_post" or self.target_point_type == "gps_and_gate":                                        
                        if not self.rotate_while_detecting_ar_ended:
                            self.control_node_in_turn_pub.publish("rotate_while_detecting_ar")
                            self.command_velocity_publisher.publish(self.rotate_while_detecting_ar_vel)
                            self.matrix_signal_msg.data = 2
                            self.matrix_signal_publisher.publish(self.matrix_signal_msg)                        
                        else:                                                
                            if self.ar_detected:
                                if not self.center_and_approach_ended:
                                    self.control_node_in_turn_pub.publish("center_and_approach")
                                    self.command_velocity_publisher.publish(self.center_and_approach_vel)
                                    self.matrix_signal_msg.data = 2
                                    self.matrix_signal_publisher.publish(self.matrix_signal_msg)        
                                else:                                
                                    self.command_velocity_publisher.publish(self.stop_vel)
                                    self.matrix_signal_msg.data = 3
                                    self.matrix_signal_publisher.publish(self.matrix_signal_msg)                
                                    self.autonomous_navigation_ended = True
                                    # THIS IS THE END OF ROUTINE FOR GPS_AND_POST                                    
                            else:
                                self.snail_trayectory_index += 1
                                print("snail trayectory index is {}".format(self.snail_trayectory_index))
                                self.set_gps_target(self.snail_trayectory_gps_points[self.snail_trayectory_index][0],
                                                        self.snail_trayectory_gps_points[self.snail_trayectory_index][1])                                                        
                                self.gps_arrived = False
                                self.rotate_while_detecting_ar_ended = False
                                task = sched.scheduler(rospy.get_time, rospy.sleep)
                                task.enter(1.0, 1, self.unblock_new_follow_gps_and_rotate_while_detecting_ar_data)
                                task.run()                            
                                self.control_node_in_turn_pub.publish("follow_gps")

if __name__ == "__main__":
    navigation_controller = NavigationController()
    navigation_controller.main()    
