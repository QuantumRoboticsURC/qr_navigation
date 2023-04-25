#!/usr/bin/env python3

"""Made by:
	José Ángel del Ángel
    joseangeldelangel10@gmail.com
Code description:
TODO - add description
Notes:
"""
import rospy
import pandas as pd
from gps_tranforms import alvinxy as gps_transforms
from visualization_msgs.msg import Marker


class MarkerSpawner():
    def __init__(self):
        # ________ ros atributes initialization ______
        rospy.init_node("marker_spawner")
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.marker_msg = Marker()
        self.gps_target_file = "/home/jose/Documents/quantum/quantum_ws/src/qr_navigation/scripts/csv_files/joses_tests.csv"
        self.rate = rospy.Rate(0.5)

        self.snail_trayectory_xy_points = []
        self.target_x = None
        self.target_y = None
        self.read_target()
        self.generate_snail_trayectory_points()

    def read_target(self):
        df = pd.read_csv(self.gps_target_file, index_col=False)        
        latitude = float( df["latitude"][0] )
        longitude = float( df["longitude"][0] )                
        self.target_x, self.target_y = gps_transforms.ll2xy( latitude,
                                                            longitude,
                                                            19.594558,
                                                            -99.228084) 
                                
    def generate_snail_trayectory_points(self, num_turns = 2):                        
        origin = (self.target_x, self.target_y)
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
        
        self.snail_trayectory_xy_points = cords_list        

    def build_marker(self, x, y, color = (0.0, 0.0, 1.0)):
        marker_msg = Marker()
        shape = Marker.CUBE  
        marker_msg.header.frame_id = "/map"
        marker_msg.header.stamp = rospy.Time()
        marker_msg.ns = "basic_shapes_{}_{}".format(x,y)
        marker_msg.id = 0
        marker_msg.type = shape
        marker_msg.action = Marker.ADD
        
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.x = 0.3
        marker_msg.scale.y = 0.3
        marker_msg.scale.z = 0.3

        r, g, b = color
        marker_msg.color.r = r
        marker_msg.color.g = g
        marker_msg.color.b = b
        marker_msg.color.a = 1.0

        marker_msg.lifetime = rospy.Duration()
        self.marker_pub.publish(marker_msg)

    def main(self):
        while not rospy.is_shutdown():
            for point in self.snail_trayectory_xy_points:
                if point == (self.target_x, self.target_y):
                    self.build_marker(point[0], point[1], (0.0, 1.0, 0.0))
                else:    
                    self.build_marker(point[0], point[1])
            self.rate.sleep()

if __name__ == "__main__":
    matrix_signal_reciever = MarkerSpawner()
    matrix_signal_reciever.main()