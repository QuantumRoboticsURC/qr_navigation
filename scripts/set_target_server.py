

import rospy
from qr_navigation.srv import set_target,set_targetRequest,set_targetResponse

def set_target_function(req):
     return set_targetResponse(req.latitud,req.longitud,req.mode)

def get_target_function(req):
     return set_targetResponse(req.latitud,req.longitud.req.mode)
    
def coordinates_server():
      rospy.init_node('set_target_server')
      s = rospy.Service('set_target', set_target, set_target_function)
      s1 = rospy.Service('get_target', set_target, get_target_function)
      rospy.spin()

coordinates_server()