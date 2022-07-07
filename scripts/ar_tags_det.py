#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
import time

yaPublicoVerde = False
yaPublicoRojo = False
markers = []

def callback (data):
	global markers
	markers = data.markers


rospy.init_node("ar_tag_detector")
pub = rospy.Publisher('/status_led',Int32, queue_size=1)
sub = rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers, callback)
rate = rospy.Rate(10)

#1 azul
#2 rojo
#3 verde
while not rospy.is_shutdown():
    if markers != []:
        if yaPublicoVerde == 0:
            pub.publish(2)
	    yaPublicoVerde = 1
	    yaPublicoRojo = 0
    else:
	if yaPublicoRojo == 0:
            pub.publish(2)
            yaPublicoVerde = 0
	    yaPublicoRojo = 1
    rate.sleep()
