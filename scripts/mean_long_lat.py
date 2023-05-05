#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix

class MeanLongLat():


    def __init__(self):

        rospy.init_node('mean_long_lat', anonymous=True)

        rospy.Subscriber("ublox/fix", NavSatFix, self.gps_callback)

        self.stack_lat = []
        self.stack_long = []

        self.latitude = None
        self.longitude = None
        self.altitude = None

        self.i = 0

        self.rate = rospy.Rate(20)  

    def gps_callback(self, data):
        self.latitude  = data.latitude            #  // x measurement GPS.
        self.longitude = data.longitude           #  // y measurement GPS.


    def main(self):
        while not rospy.is_shutdown():
            if self.i < 100:
                self.stack_lat.append(self.latitude)
                self.stack_long.append(self.longitude)
                self.i += 1
            else:
                print(np.mean(self.stack_lat),np.mean(self.stack_long))
                self.stack_lat = []
                self.stack_long = []
                self.i = 0

if __name__ == "__main__":
    mean_ubication = MeanLongLat()
    mean_ubication.main()

