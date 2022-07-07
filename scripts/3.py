#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
import time

yaPublicoVerde = False
yaPublicoRojo = False
markers = []
flagred = 1
flaggreen = 1
flag = 1

def callback (data):
	global markers, flagred, flagblue, yaPublicoRojo, yaPublicoVerde, flag
	markers = data.markers
	pub = rospy.Publisher('/status_led', Int32, queue_size=1)     
	if flag == 1:
		pub.publish(2)
		print ("publique rojo")
		time.sleep(1)
	if flag == 1:
		pub.publish(2)
		print ("publique rojo")
		flag = 0
	if markers != [] and not yaPublicoVerde:		
		pub.publish(3)
		print("publicando verde")
		yaPublicoVerde = True	
	
	

def tres():
	rospy.init_node("ar_tag_detector")
#	pub = rospy.Publisher('/status_led',Int32, queue_size=1)	
	sub = rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers, callback)
	rospy.spin()

if __name__ == '__main__':
	tres()
#rate = rospy.Rate(.1)

#1 azul
#2 rojo
#3 verde

#while not rospy.is_shutdown():
 #   if markers != []:
  #      pub.publish(3)
   # else:
    #    pub.publish(2)

 #   rate.sleep()

