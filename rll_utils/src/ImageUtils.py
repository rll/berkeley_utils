#!/usr/bin/env python
import roslib
roslib.load_manifest("rll_utils")
from cv_bridge import CvBridge, CvBridgeError

_bridge = CvBridge()

def imgmsg_to_cv(image_message,code="bgr8"):
    try:
        cv_image = _bridge.imgmsg_to_cv(image_message, code)
        return cv_image
    except CvBridgeError, e:
        print "CVERROR converting from ImageMessage to cv IplImage"
        return None
    
def cv_to_imgmsg(cv_image,code="bgr8"):
    try:
        image_message = _bridge.cv_to_imgmsg(cv_image, code)
        return image_message
    except CvBridgeError, e:
        print "CVERROR converting from cv IplImage to ImageMessage"
        return None

