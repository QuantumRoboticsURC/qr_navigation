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
row = 0
roll = pitch = yaw = 0.0
target_angle = 0.0
zed_angle = 0.0
kp = -0.1
flag = 0
latitude = 0
longitude = 0
error = 0
sat = 0.1
def callback(data):
    global row, roll, pitch, yaw, target_angle, zed_angle, distance, goal_position, flag, latitude, longitude, error

    file = open("/home/quantum/catkin_ws/src/qr_navigation/scripts/csv_files/cedetec1.csv")
    csvreader = csv.reader(file)
    #header = next(csvreader)
    #print(header)
    csvreader = csv.reader(file)
    rows = list(csvreader)
  
    if (row <= len(rows)):
	latitude = float(rows[row][1])
	longitude = float(rows[row][2])
        #print ("latitude read: ", latitude, ", longitude read: ", longitude)
        delta_lat = float(rows[row][1])-data.pose.pose.position.x
        delta_long = float(rows[row][2])-data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        zed_angle = yaw*180/math.pi
        target_angle = -math.atan2(delta_long, delta_lat)*180/math.pi
        error = target_angle - zed_angle


        if abs(error) < 2.2:
            print ("Goal: ", row, " achieved... waiting 10 seconds for next goal")
            row += 1
            command = Twist()
            pub.publish(command)
            time.sleep(3) 
        print ("Checkpoint             : ", row)
        print ("Posicion actual        : Lat ", data.pose.pose.position.x, ", Long: ", data.pose.pose.position.y)
        print ("Posicion objetivo      : Lat ", rows[row][1], ", Long: ", rows[row][2])
        print ("Angulo de actual (IMU) : ", zed_angle)
        print ("Angulo objetivo        : ", target_angle)
        
        print ()
        file.close()
    else:
        print ("Termine")
        rospy.is_shutdown()
	flag = 1




rospy.init_node('follow_gps_rotate', anonymous=True)
rospy.Subscriber("combined_odom", Odometry, callback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
goal_position_publisher = rospy.Publisher('ublox/gps_goal', NavSatFix, queue_size = 1)
r = rospy.Rate(10)
command = Twist()
goal_position = NavSatFix()

while not rospy.is_shutdown():
        error = (target_angle-zed_angle)*math.pi/180

	if abs(error) > math.pi:
		error = error -(error/abs(error))*2*math.pi
        if abs(error) < 0.04:
	        error = 0 
        command.angular.z =max(-sat, min(kp * (error),sat))
        print(command)    
        pub.publish(command)
        #print("target: ", target, ", current angle: ", yaw/math.pi*180, ", angular_z output: ", command.angular.z)
        r.sleep()

 
#    vel_angular_z = -kp * (target_angle-zed_angle)        
 #   vel_mapeada = (vel_angular_z - -180) * (1 - -1) / (180 - -180) + -1;

  #  speed = 0.05
  #  if (vel_mapeada > speed):
   #     command.angular.z = speed
    #elif (vel_mapeada < -speed):
     #   command.angular.z = -speed
    #else:
     #   command.linear.x = 0
      #  command.angular.z = vel_mapeada
 #   pub.publish(command)

        if flag:
                command.angular.z = 0
                pub.publish(command)
		break

        #r.sleep()

