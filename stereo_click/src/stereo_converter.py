#!/usr/bin/env python

##    @package stereo_click
#    This module uses basic stereo rectification to take two pixel values from a pair of stereo cameras, and output the proper 3D point.

import roslib
roslib.load_manifest("stereo_click")
import rospy
import sys
import tf
import cv
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptySrv
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
import thread
from stereo_click.msg import *
from stereo_click.srv import *
import image_geometry

##    StereoConverter documentation
#    
#    A node which takes two input ClickPoint streams, and (once it has received a new point from both feeds) outputs the
#    corresponding PointStamped.
class StereoConverter:

    ##    The constructor
    #    @param cptopic1 A topic which publishes ClickPoints, corresponding to one camera in a stereo pair
    #    @param cptopic2 A second topic which publishes ClickPoints, corresponding to a second camera
    #    @param outputname The name of the topic which 3D points will be output to
    #    @param outputframe The frame in which to output the 3D point
    def __init__(self,cptopic1,cptopic2,outputname,outputframe):
        self.name = rospy.get_name()
        self.cptopic1 = cptopic1
        self.cptopic2 = cptopic2
        self.outputname = outputname
        self.listener = tf.TransformListener()
        self.output_frame = outputframe
        self.cp1 = False
        self.cp2 = False
        self.cp1_sub = rospy.Subscriber(cptopic1, ClickPoint, self.handle_cp1)
        self.cp2_sub = rospy.Subscriber(cptopic2, ClickPoint, self.handle_cp2)
        self.service = rospy.Service("%s/convert"%self.name,ConvertPoints, self.serve)
        self.pub = rospy.Publisher(self.outputname,PointStamped)
        
        
    def handle_cp1(self,cp):
        self.cp1 = cp
        self.handle_cps()
    
    def handle_cp2(self,cp):
        self.cp2 = cp
        self.handle_cps()
        
    def serve(self,req):
        cp1 = req.click_point_left
        cp2 = req.click_point_right
        pt3d = self.convert_stereo(cp1,cp2)
        pt3d.header.stamp = rospy.Time.now()
        return ConvertPointsResponse(pt3d)
    
    ##    Waits till new ClickPoints have been received from both topics, calculates the corresponding
    #    3D point, publishes it, and resets both ClickPoints.
    def handle_cps(self):
        if self.cp1 != False and self.cp2 != False:
            pt3d = self.convert_stereo(self.cp1,self.cp2)
            self.publish(pt3d)
            self.cp1 = False
            self.cp2 = False
            self.signal_received(self.cptopic1)
            self.signal_received(self.cptopic2)
    
    ##    Signals that it has received a clickpoint from the given cptopic
    def signal_received(self,cptopic):
        try:
            signal = rospy.ServiceProxy("%s/received"%cptopic,EmptySrv)
            signal()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

                
    def publish(self,pt3d):
        self.pub.publish(pt3d)
    
    ##    Uses the image_geometry module to convert the two ClickPoints to one 3D point
    #    Note that the two ClickPoints come from a stereo pair of cameras. i.e., if the
    #    first is not the left camera, the second must be.
    #    @param cp1 The first ClickPoint
    #    @param cp2 The second ClickPoint
    def convert_stereo(self,cp1,cp2):
        if cp1.camera_info.P[3] == 0:
            left = cp1
            right = cp2
        else:
            left = cp2
            right = cp1
        u = left.x
        v = left.y
        disparity = left.x - right.x
        
        stereo_model = image_geometry.StereoCameraModel()
        stereo_model.fromCameraInfo(left.camera_info,right.camera_info)
        (x,y,z) = stereo_model.projectPixelTo3d((u,v),disparity)
        camera_pt = PointStamped()
        camera_pt.header.frame_id = left.camera_info.header.frame_id
        camera_pt.header.stamp = rospy.Time.now()
        camera_pt.point.x = x
        camera_pt.point.y = y
        camera_pt.point.z = z
        self.listener.waitForTransform(self.output_frame,camera_pt.header.frame_id,rospy.Time.now(),rospy.Duration(4.0))
        output_point = self.listener.transformPoint(self.output_frame,camera_pt)

        return output_point
        
## Creates a stereo_converter node    
def main(args):
    
    rospy.init_node("stereo_converter")
    cptopic1 = rospy.get_param('~input1',"input1_default")
    cptopic2 = rospy.get_param('~input2',"input2_default")
    outputname = rospy.get_param('~output',"output_defalut")
    outputframe = rospy.get_param('~output_frame',"output_frame_default")
    converter = StereoConverter(cptopic1=cptopic1,cptopic2=cptopic2,outputname=outputname,outputframe=outputframe)
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
