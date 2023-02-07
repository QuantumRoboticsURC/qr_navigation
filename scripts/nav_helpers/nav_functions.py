#!/usr/bin/env python3

import math
import numpy as np

def calculate_yaw_angle(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)            
    yaw = angle_to_only_possitive(yaw)
    return yaw  

def angle_to_only_possitive(angle):
    theta = angle
    if np.sign(theta) == -1.0:
        theta = 2*math.pi + theta
    return theta

def euclidean_distance_point_to_point_2d(p1, p2):
    return math.sqrt( (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 )

def euclidean_distance_single_point_2d(p1):
    return math.sqrt( (p1[0])**2 + (p1[1])**2 )

def euler_from_quaternion(orientation_list):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = orientation_list
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def saturate_signal(signal, saturation_value):
        if signal > abs(saturation_value):
            result = abs(saturation_value)
        elif signal < -abs(saturation_value):
            result = -abs(saturation_value)
        else:
            result = signal
        return result