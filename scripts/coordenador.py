#! /usr/bin/env python

"""
Made by:Raul Lopez Musito
	A01378976@tec.mx
	raulmusito@gmail.com

Modified (DD/MM/YY): 
	Raul Musito 28/06/2022 Correct a bugg at the number of line write at 
				the csv file.
	Raul Musito 03/06/2022 Created the program

Code description:
1. Ask the user for the name of a file.
2. If the file exist at a predefined directory it opens it and read it.
   If not, it creates a file with the name.
3. Ask the user the action to execute (write lat & long | write the square).
4. Close the file. 

Notes:
- Validate the user input
- Add an exit option
* Despite the code adds 0.00001 theorically, it's not exact.
 
"""

import csv
import os.path
from os import path

if __name__ == "__main__":
	
	user_file = input("file: (use quotation marks) ")
	user_file = "/home/quantum/catkin_ws/src/qr_navigation/scripts/csv_files/" + user_file + ".csv"

	#If the file does exist
	if path.exists(user_file) == 1:
		print ("Archivo existente")
		tipo = "a+" #Append type
		
		#Iterate over the file
		last_line = 0 
		with open(user_file, "r") as file:
			#first_line = file.readline()
			for last_line in file:
				print (last_line)
				pass
		#Get the line index
		if last_line == 0:
			i = 0
			latitude = 0	
			longitude = 0
		else:
			i = int(last_line[0]) + 1
			latitude = int(last_line[1])
			longitude = int(last_line[2])
		
	#If the file doesn't exist
	else:
	
		print ("Archivo inexistente")
		i = 0
		tipo = "w+" #write type

	print (latitude)
	#Open the file with the correct type
	with open(user_file, tipo) as file_out:
		writer = csv.writer(file_out, delimiter=',')

		user_input = input("save? (2: write, 1: calculate, 0: exit) ")#Take the user input
		plos = 0.00001  #Approx 1 meter
		reps = 4 #Number of squares to do

		while True: 
			
			#Exit
			if user_input == 0:
				break

			#Add a square
			elif user_input == 1:
				lat1 = latitude
				lon1 = longitude

				lat2 = latitude
				lon2 = longitude

				lat3 = latitude
				lon3 = longitude

				lat4 = latitude
				lon4 = longitude

				for k in range (0,reps):
					lat1 += plos
					lon1 -= plos

					lat2 -= plos
					lon2 -= plos

					lat3 -= plos
					lon3 += plos

					lat4 += plos
					lon4 += plos

					writer.writerow([i,lat1,lon1])
					i += 1

					writer.writerow([i,lat2,lon2])
					i += 1

					writer.writerow([i,lat3,lon3])
					i += 1

					writer.writerow([i,lat4,lon4])
					i += 1

			#Write lat & long			
			elif user_input == 2:
				latitude = float(input("latitude: "))
				longitude = float(input("longitude: "))
				print ("i: " + str(i)+ "; long: " + str(longitude) +  "; lat: "+str(latitude))
				i += 1
				writer.writerow([i,latitude,longitude]) #Write into the file


			#Take the user input
			user_input = int(input("save? (2: write, 1: calculate, 0: exit) "))
				
		print("Adios popo")
		file_out.close()


