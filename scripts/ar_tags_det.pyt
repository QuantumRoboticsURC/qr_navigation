 #!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
import time


yaPublico = False
markers = []

def callbak (data):
	global markers
	markers = data.markers


rospy.init_node("ar_tag_detector")
pub = rospy.Publisher('/status_led',Int32, queue_size=1)
sub = rospy.Subscriber('/zed/ar_pose_marker',AlvarMarkers, callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if markers != []:
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

    rospy.spin()
