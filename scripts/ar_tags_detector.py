#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import time
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
import time
#geometry_msgs/PoseStamped
from geometry_msgs.msg import PoseStamped
import math
yaPublico = False
isDoing = False
def callback (data):
	global yaPublico, isDoing
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
			pub.publish(2)
		yaPublico = False

pub = rospy.Publisher('/status_led',Int32, queue_size=1)
rospy.init_node("ar_tag_detector")
sub = rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers, callback)	
#rate = rospy.Rate(10)
rospy.spin()
