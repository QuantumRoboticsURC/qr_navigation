#!/usr/bin/env python
import math
import csv
import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import NavSatFix

#Camara ZED Disminuye sentido horario. NORTE = 0 grados 
#Camara ZED Aumenta sentido anti horario ESTE = -90 grados
#Norte 19.634669, -99.074926,  4.81 grados
#Sur 19.62164,  -99.073953, -177 grados
#Este 19.624768, -99.067749, -96 grados
#Oeste 19.626214, -99.077013, 75 grados  
#Se tuvo que poner menos al angulo que se calcula para que quede con el de la ZED
#Se inicializa viendo hacia el norte 

#Se inicializa variable row
row = 0 #A partir de que waypoint va a iniciar
roll = pitch = yaw = 0.0
target_angle = 0.0
zed_angle = 0.0
kp = -0.1
distance = 0.0
error = 0
sat = 0.1

def callback(data):
    global row, roll, pitch, yaw, target_angle, zed_angle, distance, row # Se agrego ROW
    file = open("/home/quantum/catkin_ws/src/qr_navigation/scripts/csv_files/travelodge2.csv")
    csvreader = csv.reader(file)
    #header = next(csvreader)
    #print(header)
    csvreader = csv.reader(file)
    rows = list(csvreader)
    
    if (row <= len(rows)):
        latitude = float(rows[row][1])
        longitude = float(rows[row][2])
        delta_lat = latitude-data.pose.pose.position.x
        delta_long = longitude-data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        zed_angle = yaw*180/math.pi
        target_angle = -math.atan2(delta_long, delta_lat)*180/math.pi
        error = target_angle - zed_angle         
        distance = ((delta_lat**2 + delta_long**2)**0.5)*108000
        #print (distance)
        if (distance < 2): #Se comento

            row += 1          #Se comento
            command = Twist()
            command.linear.x = 0
            command.angular.z = 0
            pub.publish(command)
            print("Siguiente checkpoint")
            #time.sleep(10)          

            
#        print ("Posicion actual        : Lat ", data.pose.pose.position.x, ", Long: ", data.pose.pose.position.y)
#        print ("Posicion objetivo      : Lat ", rows[row][1], ", Long: ", rows[row][2])
#        print ("Angulo de actual (ZED) : ", zed_angle)
#        print ("Angulo objetivo        : ", target_angle)
#        print ("Distancia              : ", distance)
#        print ()
        file.close()
    else:
        print ("Termine")
	posicion_final 
        rospy.is_shutdown()
    goal_pub.latitude = latitude
    goal_pub.longitude = longitude
    goal_position_publisher.publish(goal_pub)

rospy.init_node('follow_gps', anonymous=True)
rospy.Subscriber("combined_odom", Odometry, callback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
goal_position_publisher = rospy.Publisher('ublox/gps_goal', NavSatFix, queue_size = 1)
goal_pub = NavSatFix()
r = rospy.Rate(10)
command = Twist()

while not rospy.is_shutdown():
    error = target_angle-zed_angle
    print (error)
    if (error > -5 and error < 5):
        if (distance < 6 and distance > 2 ):
            command.linear.x = 0.08
            command.angular.z = 0.0
            print("Avanzando recto velocidad minima, distancia restante: ", distance)
       # elif (distance <= 2):
        #    print("Siguiente checkpoint")
         #   time.sleep(5)

                        

        else:
            command.linear.x = 0.1
            command.angular.z = 0.0
            print("Avanzando recto velocidad maxima, distancia restante: ", distance)
    else:
        if abs(error) > 180:
            error = error -(error/abs(error))*360
        if abs(error) < 5:
            error = 0 
        command.linear.x = 0
        command.angular.z = max(-sat, min(kp * (error),sat))

    print(command)    
    pub.publish(command)

    r.sleep()

