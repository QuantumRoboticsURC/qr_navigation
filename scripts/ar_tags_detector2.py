#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import time
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import math

yaPublico = False

+markers = []

def callback (data):
    global yaPublico, isDoing, markers
    markers = data.markers

pub = rospy.Publisher('/status_led',Int32, queue_size=1)
rospy.init_node("ar_tag_detector")
sub = rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers, callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if data.markers != []:
        if(not yaPublico ):
            yaPublico = True
            for i in range(3):
                pub.publish(3)
                rospy.sleep(1)
                pub.publish(0)
                rospy.sleep(1)
                pub.publish(2)
        else:
            if(yaPublico):
                fpub.publish(2)
                yaPublico = False

    rospy.spin()
