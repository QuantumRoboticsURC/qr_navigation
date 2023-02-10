"""
This script contains constants that vary according
to the platform where the code runs, thus this
file should contain diferent constants depending on 
the platform where it runs.

The info of this platform is the following:
- Platform: Jose's personal laptop
- Purpose: robot simulation
"""

# _____________ FOLLOW GPS CONSTANTS ________
GPS_TARGET_CSV_PATH = "/home/quantum_main/catkin_ws/src/qr_navigation/scripts/csv_files/goal_cords.csv"
FOLLOW_GPS_ANGULAR_ERROR_TRESHOLD = 0.1 # rads
FOLLOW_GPS_LINEAR_ERROR_TRESHOLD = 2.5 # meters
FOLLOW_GPS_ANGULAR_KP = -0.3
FOLLOW_GPS_LINEAR_KP = 0.5
FOLLOW_GPS_ANGULAR_SATURATION_VAL = 1.0*3.1416 
FOLLOW_GPS_LINEAR_SATURATION_VAL = 0.6

# _____________ ROTATE WHILE DETECTING AR CONSTANTS ________
ROTATE_WHILE_DETECTING_AR_ANGULAR_VEL = 0.5

# _____________ CENTER AND APPROACH CONSTANTS ________
CENTER_AND_APPROACH_ANGULAR_KP = -0.15
CENTER_AND_APPROACH_LINEAR_KP = 0.2
CENTER_AND_APPROACH_LINEAR_SET_POINT = 1.0
CENTER_AND_APPROACH_ANGULAR_ERROR_TRESHOLD = 0.15 # rads
CENTER_AND_APPROACH_LINEAR_ERROR_TRESHOLD = 0.1 # meters
CENTER_AND_APPROACH_ANGULAR_SATURATION_VAL = 1*3.1416
CENTER_AND_APPROACH_LINEAR_SATURATION_VAL = 0.4
