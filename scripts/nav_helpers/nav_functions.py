#!/usr/bin/env python3

import math
import numpy as np
from scipy.spatial.transform import Rotation

def calculate_yaw_angle(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)            
    yaw = angle_to_only_possitive(yaw)
    return yaw  

def calculate_yaw_angle_deg(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    yaw = yaw*(180.0/math.pi)            
    yaw = angle_to_only_possitive_deg(yaw)
    return yaw  

def calculate_pitch_angle_deg(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, pitch, _) = euler_from_quaternion(orientation_list)
    pitch = pitch*(180.0/math.pi)                
    return pitch  

def calculate_roll_angle_deg(orientation_q):
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, _, _) = euler_from_quaternion(orientation_list)
    roll = roll*(180.0/math.pi)                
    return roll  


def angle_to_only_possitive_deg(angle):
    theta = angle
    if np.sign(theta) == -1.0:
        theta = 360.0 + theta
    return theta

def angle_to_only_possitive(angle):
    theta = angle
    if np.sign(theta) == -1.0:
        theta = 2.0*math.pi + theta
    return theta

def rad2deg(theta):
    res = theta*(180.0/math.pi)
    res = angle_to_only_possitive_deg(res)
    return res

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
    """
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
    """

    r = Rotation.from_quat([x, y, z, w])
    roll, pitch, yaw = r.as_euler('xyz')
    return roll, pitch, yaw

def saturate_signal(signal, saturation_value):
        if signal > abs(saturation_value):
            result = abs(saturation_value)
        elif signal < -abs(saturation_value):
            result = -abs(saturation_value)
        else:
            result = signal
        return result

def calculate_angular_error_considering_upper_boundary_lag(target_angle, current_angle, deg_or_rad, angle_reference_range):
    """
    ## Calculate angular error with upper boundary lag
    this function calculates angular error considering analysis found 
    at [dagrams/angle_error_exception.jpeg](/diagrams/angle_error_exception.jpeg)    
    """ 

    

    if (( deg_or_rad == "deg" and (angle_reference_range == (-180.0, 180.0) or angle_reference_range == (0.0, 360.0)) ) or
    ( deg_or_rad == "rad" and (angle_reference_range == (-math.pi, math.pi) or angle_reference_range == (0.0, 2*math.pi)) )):

        err = target_angle - current_angle        

        if angle_reference_range == (0.0, 360.0) or angle_reference_range == (0.0, 2*math.pi):

            #correct case 2A and 2B            
            if target_angle < (angle_reference_range[1]/2.0) and current_angle > (angle_reference_range[1]/2.0) and (current_angle >= (target_angle + angle_reference_range[1]/2.0)): 
                #case 2A                
                err += angle_reference_range[1]
            elif current_angle < (angle_reference_range[1]/2.0) and target_angle > (angle_reference_range[1]/2.0) and (target_angle >= (current_angle + angle_reference_range[1]/2.0)):
                #case 2B                
                err -= angle_reference_range[1]

        elif angle_reference_range == (-180.0, 180.0) or angle_reference_range == (-math.pi, math.pi):

            if (target_angle > angle_reference_range[0]) and (target_angle < angle_reference_range[0]/2.0) and (current_angle < angle_reference_range[1]) and (current_angle > angle_reference_range[1]/2.0):

                err = abs((angle_reference_range[0]-target_angle)-(angle_reference_range[1]-current_angle))

            elif (current_angle > angle_reference_range[0]) and (current_angle < angle_reference_range[0]/2.0) and (target_angle < angle_reference_range[1]) and (target_angle > angle_reference_range[1]/2.0):
                
                err = -abs((angle_reference_range[1]-target_angle)-(angle_reference_range[0]-current_angle))

        else:

            pass

        return err
    else:
        Exception("calculate_angular_error_considering_upper_boundary_lag: discrepancy in angle units or reference range")