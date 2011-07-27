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
from geometry_msgs.msg import PointStamped,PoseStamped
import thread
from stereo_click.msg import *
from stereo_click.srv import *
import image_geometry
import numpy as np

##    MonoConverter documentation
#    
#    A node which takes one input ClickPoint stream, and (once it has received a new point from both feeds) outputs the
#    corresponding PointStamped.
class MonoConverter:

    ##    The constructor
    #    @param cptopic A topic which publishes ClickPoints, corresponding to one camera
    #   @param table_height The height of the table (assuming it is flat wrt base footprint)
    #    @param outputname The name of the topic which 3D points will be output to
    #    @param outputframe The frame in which to output the 3D point
    def __init__(self,cptopic,table_height,outputname,outputframe):
        self.name = rospy.get_name()
        self.cptopic = cptopic
        self.table_height = table_height
        self.outputname = outputname
        self.listener = tf.TransformListener()
        self.output_frame = outputframe
        self.cp_sub = rospy.Subscriber(self.cptopic, ClickPoint, self.handle_cp)
        self.convert_server = rospy.Service("%s/convert"%self.name, ConvertPoint, self.convert_serve)
        self.pub = rospy.Publisher(self.outputname,PointStamped)
        self.pose_pub = rospy.Publisher("test_poses",PoseStamped)
        
    def handle_cp(self,cp):
        pt3d = self.convert_mono(cp)
        self.publish(pt3d)
        self.signal_received(self.cptopic)
        
    def convert_serve(self,req):
        cp = req.click_point
        pt3d = self.convert_mono(cp)
        pt3d.header.stamp = rospy.Time.now()
        return ConvertPointResponse(pt3d)
    
    # Use this when opencv gets properly updated. projectPixelTo3dRay is broken because of fx(), etc.
    def convert_mono_better(self,cp):

        # Get the 3d ray in the camera frame
        cam_frame = cp.camera_info.header.frame_id
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(cp.camera_info)
        (r_x,r_y,r_z) = cam_model.projectPixelTo3dRay((cp.x,cp.y))

        # Get x_0 and x_1 in the base frame
        now = rospy.Time.now()
        origin_cam_frame = PointStamped()
        origin_cam_frame.header.frame_id =cam_frame
        origin_cam_frame.header.stamp = now
        ray_cam_frame = PointStamped()
        ray_cam_frame.header.frame_id = cam_frame
        ray_cam_frame.header.stamp = now
        ray_cam_frame.point.x = r_x
        ray_cam_frame.point.y = r_y
        ray_cam_frame.point.z = r_z
        self.listener.waitForTransform('base_footprint',cam_frame, now, rospy.Duration(10.0))
        origin_base_frame = self.listener.transformPoint('base_footprint',origin_cam_frame)
        ray_base_frame = self.listener.transformPoint('base_footprint',ray_cam_frame)
        #Solve x = x_0 + vt for x[2] = table_height
        x_0 = np.array([origin_base_frame.point.x,origin_base_frame.point.y,origin_base_frame.point.z])
        v = np.array([ray_base_frame.point.x,ray_base_frame.point.y,ray_base_frame.point.z]) - x_0
        t = (self.table_height - x_0[2]) / float(v[2])
        x = x_0 + v*t
        #Convert to output frame and return
        output_pt_base_frame = PointStamped()
        output_pt_base_frame.header.frame_id = 'base_footprint'
        now = rospy.Time.now()
        output_pt_base_frame.header.stamp = now
        output_pt_base_frame.point.x = x[0]
        output_pt_base_frame.point.y = x[1]
        output_pt_base_frame.point.z = x[2]
        self.listener.waitForTransform(self.output_frame,'base_footprint', now, rospy.Duration(10.0))
        output_pt = self.listener.transformPoint(self.output_frame,output_pt_base_frame)

        return output_pt

    def convert_mono(self,cp):
        u = cp.x
        v = cp.y
        info = cp.camera_info
        camera_frame = info.header.frame_id
        # Initialize matrices
        pt_homog = cv.CreateMat(3,1,cv.CV_32FC1)
        pt_homog[0,0] = u
        pt_homog[1,0] = v
        pt_homog[2,0] = 1
        Proj = cv.CreateMat(3,3,cv.CV_32FC1)
        for i,val in enumerate(info.P):
            if i%4 != 3 and i/4 < 3:
                Proj[i/4,i%4] = val
        Proj_inv = cv.CreateMat(3,3,cv.CV_32FC1)
        cv.Invert(Proj,Proj_inv)
        #Get translation and rotation
        now = rospy.Time.now()
        self.listener.waitForTransform("/base_footprint", camera_frame, now, rospy.Duration(10.0))
        (trans, rot) = self.listener.lookupTransform("/base_footprint", camera_frame, now)
        rot_array = tf.transformations.quaternion_matrix(rot)
        Rot = cv.CreateMat(3,3,cv.CV_32FC1)
        for i in range(3):
            for j in range(3):
                Rot[i,j] = rot_array[i,j]
        u_unrot = cv.CreateMat(3,1,cv.CV_32FC1)
        self.cvMultiply(Proj_inv,pt_homog,u_unrot)
        u_rot = cv.CreateMat(3,1,cv.CV_32FC1)
        self.cvMultiply(Rot,u_unrot,u_rot)
        s = (self.table_height- trans[2]) / float(u_rot[2,0])
        x = s*u_rot[0,0] + trans[0]
        y = s*u_rot[1,0] + trans[1]
        #z = s*u_rot[2,0] + trans[2]
        z = self.table_height
        #assert abs(z-z2) < 0.005
        base_pt = PointStamped()
        base_pt.header.stamp = rospy.Time.now()
        base_pt.header.frame_id = "/base_footprint"
        base_pt.point.x = x
        base_pt.point.y = y
        base_pt.point.z = z
        now = rospy.Time.now()
        self.listener.waitForTransform(self.output_frame,"/base_footprint", now, rospy.Duration(10.0))
        output_pt = self.listener.transformPoint(self.output_frame,base_pt)
        return output_pt

        
    def cvMultiply(self,mat1,mat2,output):
        cv.GEMM(mat1,mat2,1,None,1,output)

    ##    Signals that it has received a clickpoint from the given cptopic
    def signal_received(self,cptopic):
        return
        try:
            signal = rospy.ServiceProxy("%s/received"%cptopic,EmptySrv)
            signal()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

                
    def publish(self,pt3d):
        self.pub.publish(pt3d)
        pose = PoseStamped()
        pose.header.frame_id = pt3d.header.frame_id
        pose.header.stamp = pt3d.header.stamp
        pose.pose.position.x = pt3d.point.x
        pose.pose.position.y = pt3d.point.y
        pose.pose.position.z = pt3d.point.z
        pose.pose.orientation.w = 1
        self.pose_pub.publish(pose)
    
        
## Creates a stereo_converter node    
def main(args):
    
    rospy.init_node("stereo_converter")
    cptopic = rospy.get_param('~input',"input_default")
    table_height = rospy.get_param('table_height',0.895)
    outputname = rospy.get_param('~output',"output_defalut")
    outputframe = rospy.get_param('~output_frame',"output_frame_default")
    converter = MonoConverter(cptopic=cptopic,table_height=table_height,outputname=outputname,outputframe=outputframe)
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
