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

def get_next_message(topic,message_type,timeout=None):
    responses = []
    sub = rospy.Subscriber(topic,message_type,lambda msg: responses.append(msg))
    start = rospy.Time.now()
    if timeout:
        finish = rospy.Time.now() + rospy.Duration(timeout)
    while len(responses) == 0:
        rospy.sleep(0.1)
        if timeout:
            if rospy.Time.now() >= finish:
                responses.append(None)     
    sub.unregister()
    output = responses[0]
    for val in list(responses):
        responses.remove(val)
    return output
    
    
def call_and_response(output_topic,output_type,output_message,input_topic,input_type,timeout=None):
    pub = rospy.Publisher(output_topic,output_type)
    pub.publish(output_message)
    return get_next_message(input_topic,input_type,timeout)
