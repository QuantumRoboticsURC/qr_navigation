#!/usr/bin/env python

"""Made by:
	Leonardo Javier Nava
    navaleonardo40@gmail.com
Code description:
TO-DO - Cleaning code from follow_gps2
Notes:
"""

import math
import csv
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix

#Camara ZED Disminuye sentido horario. NORTE = 0 grados 
#Camara ZED Aumenta sentido anti horario ESTE = -90 grados
#Norte 19.634669, -99.074926,  4.81 grados
#Sur 19.62164,  -99.073953, -177 grados
#Este 19.624768, -99.067749, -96 grados
#Oeste 19.626214, -99.077013, 75 grados  
#Se tuvo que poner menos al angulo que se calcula para que quede con el de la ZED
#Se inicializa viendo hacia el norte 


class FollowGPS():
    def __init__():

        rospy.init_node('follow_gps', anonymous=True)
        rospy.Subscriber("combined_odom", Odometry, callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal_position_publisher = rospy.Publisher('ublox/gps_goal', NavSatFix, queue_size = 1)
        self.goal_pub = NavSatFix()
        self.r = rospy.Rate(10)
        self.command = Twist()

        #Se inicializa variable row
        self.row = 0 #A partir de que waypoint va a iniciar
        self.roll = 0.0
        self.pitch = 0.0 
        self.yaw = 0.0
        self.target_angle = 0.0
        self.zed_angle = 0.0
        self.kp = -0.3
        self.distance = 0.0
        self.error = 0
        self.sat = 0.4




    def callback(data):
        
        file = open("/home/quantum_main/catkin_ws/src/qr_navigation/scripts/csv_files/frida5.csv")
        csvreader = csv.reader(file)
        #header = next(csvreader)
        #print(heade
        csvreader = csv.reader(file)
        rows = list(csvreader)
        if (self.row <= len(rows)):
            latitude = float(rows[self.row][1])
            longitude = float(rows[self.row][2])
            delta_lat = latitude-data.pose.pose.position.x
            delta_long = longitude-data.pose.pose.position.y
            orientation_q = data.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
            zed_angle = self.yaw*180/math.pi
            self.target_angle = -math.atan2(delta_long, delta_lat)*180/math.pi
            self.error = self.target_angle - zed_angle         
            distance = ((delta_lat**2 + delta_long**2)**0.5)*108000
            
            if (distance < 1): #Se comento
                row += 1          #Se comento
                self.command = Twist()
                self.command.linear.x = 0
                self.command.angular.z = 0
                pub.publish(self.command)
                print("Siguiente checkpoint")
            file.close() 
        else:
            print ("Termine")
            rospy.is_shutdown()
        goal_pub.latitude = latitude
        goal_pub.longitude = longitude
        goal_position_publisher.publish(goal_pub)

    def main():

        while not rospy.is_shutdown():
            error = target_angle-zed_angle
            print (error)
            if (error > -5 and error < 5):
                if (distance < 6 and distance > 2 ):
                    command.linear.x = 0.25
                    command.angular.z = 0.0
                    print("Avanzando recto velocidad minima, distancia restante: ", distance)
                         

                else:
                    command.linear.x = 0.25
                    command.angular.z = 0.0
                    print("Avanzando recto velocidad maxima, distancia restante: ", distance)
            else:
                if abs(error) > 180:
                    error = error -(error/abs(error))*360
                if abs(error) < 5:
                    error = 0 
                command.linear.x = 0
                command.angular.z = max(-self.sat, min(self.kp * (error),self.sat))

            print(command)    
            #pub.publish(command)

            r.sleep()

if __name__ == "__main__":
    follow_gps = FollowGPS()
    follow_gps.main()