#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String

def talker():
    pub = rospy.Publisher('detected', Bool, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    z = Bool()
    z.data = True
    while not rospy.is_shutdown():
        pub.publish(z)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass