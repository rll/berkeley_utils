#!/usr/bin/env python
import roslib
roslib.load_manifest("rll_utils")
import rospy

##  Calls a service by creating a proxy, catching the necessary exceptions
#   Usage ex: resp = call_service("process_mono",ProcessMono,"wide_stereo/left"
#   Usage ex: resp = call_service("process_mono",ProcessMono,camera="wide_stereo/left")
#   (can still use keyword arguments)
def call_service(service_name, service_type, *non_keyword_args, **keyword_args):
    rospy.logdebug("Waiting for service: %s"%service_name)
    rospy.wait_for_service(service_name)
    try:
        srv = rospy.ServiceProxy(service_name, service_type)
        resp = srv(*non_keyword_args, **keyword_args)
        return resp
    except rospy.ServiceException,e:
        rospy.loginfo("Service Call Failed: %s"%str(e))
        return None
