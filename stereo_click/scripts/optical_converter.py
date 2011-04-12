#!/usr/bin/env python

##    @package stereo_click
#    This module uses basic stereo rectification to take corresponding pixel values from two views, and output the proper 3D point.

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
import numpy as np

##    OpticalConverter documentation
#    
#    A node which takes one ClickPoint stream, and (once it has received two points) outputs the
#    corresponding PointStamped.
class OpticalConverter:

    ##    The constructor
    #    @param cptopic A topic which publishes ClickPoints, corresponding to one camera
    #    @param outputname The name of the topic which 3D points will be output to
    #    @param outputframe The frame in which to output the 3D point
    def __init__(self,cptopic,outputname,outputframe):
        self.name = rospy.get_name()
        self.cptopic = cptopic
        self.outputname = outputname
        self.listener = tf.TransformListener()
        self.output_frame = outputframe
        self.cp1 = False
        self.cp2 = False
        self.A1 = None
        self.A2 = None
        self.B1 = None
        self.B2 = None
        self.cp_sub = rospy.Subscriber(cptopic, ClickPoint, self.handle_cp)
        #self.service = rospy.Service("%s/convert"%self.name,ConvertPoints, self.serve)
        self.pub = rospy.Publisher(self.outputname,PointStamped)
        
        
    def handle_cp(self,cp):
        (pt_1, pt_2) = self.compute_pts(cp)
        if not self.cp1:
            self.cp1 = True
            self.A1 = pt_1
            self.A2 = pt_2
        else:
            self.cp2 = True
            self.B1 = pt_1
            self.B2 = pt_2
            self.handle_cps()
        self.signal_received(self.cptopic)
        
    def compute_pts(self, cp):
        cam_frame = cp.camera_info.header.frame_id
        now = rospy.Time.now()
        self.listener.waitForTransform(self.output_frame,cam_frame,now,rospy.Duration(4.0))
        cam_origin = PointStamped()
        cam_origin.header.frame_id = cam_frame
        cam_origin.header.stamp = now
        pt_1 = self.listener.transformPoint(self.output_frame,cam_origin)
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(cp.camera_info)
        (x2,y2,z2) = cam_model.projectPixelTo3dRay((cp.x,cp.y))
        cam_ray = PointStamped()
        cam_ray.header.frame_id = cam_frame
        cam_ray.header.stamp = now
        cam_ray.point.x=x2
        cam_ray.point.y=y2
        cam_ray.point.z=z2
        pt_2 = self.listener.transformPoint(self.output_frame,cam_ray)
        return(pt_1,pt_2)
    
    ##    Waits till new ClickPoints have been received from both topics, calculates the corresponding
    #    3D point, publishes it, and resets both ClickPoints.
    def handle_cps(self):
        pt3d = self.convert_optical(self.A1, self.A2,self.B1, self.B2)
        self.publish(pt3d)
        self.cp1 = False
        self.cp2 = False
        self.A1 = None
        self.A2 = None
        self.B1 = None
        self.B2 = None
       
    
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
    #    Solves LS for Ax = y, where x is the array [x y z s t], A is [I v_1 0; I 0 v_2], and
    #   y is [x_0;x_1]
    def convert_optical(self,A1,A2,B1,B2):
        x_0 = np.array([A1.point.x,A1.point.y,A1.point.z])
        v_0 = np.array([A2.point.x,A2.point.y,A2.point.z]) - x_0
        x_1 = np.array([B1.point.x,B1.point.y,B1.point.z])
        v_1 = np.array([B2.point.x,B2.point.y,B2.point.z]) - x_1
        A = np.zeros((6,5))
        A[0,0] = 1
        A[1,1] = 1
        A[2,2] = 1
        A[3,0] = 1
        A[4,1] = 1
        A[5,2] = 1
        A[0:3,3] = v_0
        A[3:6,4] = v_1
        y = np.concatenate((x_0,x_1))
        x = np.dot(np.linalg.pinv(A), y.transpose())
        output_pt = PointStamped()
        output_pt.header.frame_id = A1.header.frame_id
        output_pt.header.stamp = rospy.Time.now()
        output_pt.point.x = x[0]
        output_pt.point.y = x[1]
        output_pt.point.z = x[2]
        return output_pt
        
## Creates an optical_converter node    
def main(args):
    
    rospy.init_node("optical_converter")
    cptopic = rospy.get_param('~input',"input_default")
    outputname = rospy.get_param('~output',"output_defalut")
    outputframe = rospy.get_param('~output_frame',"output_frame_default")
    converter = OpticalConverter(cptopic=cptopic,outputname=outputname,outputframe=outputframe)
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
