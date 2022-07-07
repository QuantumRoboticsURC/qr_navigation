#! /usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import NavSatFix
import os.path
from os import path


def callback (data):
	global latitude
	global longitude
	latitude = data.latitude
	longitude = data.longitude

latitude = 0	
longitude = 0

rospy.init_node("csv_writer")
rospy.Subscriber("/ublox/fix", NavSatFix, callback)
rate = rospy.Rate(10)


if __name__ == "__main__":
	
	user_file = input("file: (use quotation marks) ")
	user_file = "/home/quantum/catkin_ws/src/qr_navigation/scripts/csv_files/" + user_file + ".csv"

	if path.exists(user_file) == 1:
		print ("Archivo existente")
		tipo = "a+"

		with open(user_file, "r") as file:
			first_line = file.readline()
			for last_line in file:
				pass
		i = int(last_line[0]) + 1
		

	else:
		print ("Archivo inexistente")
		i = 0
		tipo = "w"

	with open(user_file, tipo) as file_out:
			writer = csv.writer(file_out, delimiter=',')

			while True:
				user_input = input("save? (1: yes, 0: exit) ")
				if user_input == 1:
					print ("i: " + str(i)+ "; long: " + str(longitude) +  "; lat: "+str(latitude)	)
					writer.writerow([i,latitude,longitude])
					i += 1
				elif user_input == 0:
					file_out.close()
					break
				rate.sleep()

