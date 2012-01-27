#!/usr/bin/env python
import roslib
roslib.load_manifest("rll_utils")
import rospy
from tf import TransformListener, Exception as TFException, transformations
from geometry_msgs.msg import PointStamped,Point,PoseStamped,Pose,QuaternionStamped,Quaternion
from numpy import array as nparray
import numpy as np

##  Decorator which, when added, tries the execute a function max_tries times, before
#   throwing an error.
def catchTFErrors(max_tries):
    def wrap(f):
        def new_f(*args,**kwds):
            latest_error = None
            for i in range(max_tries):
                try:
                    output = f(*args,**kwds)
                    return output
                except TFException, e:
                    rospy.logdebug("Error transforming: %s. On try %d of %d"%(str(e),i,max_tries))
                    latest_error = e
            #If it has reached the end and not finished, throw the latest error
            raise latest_error
            return None
        return new_f
    return wrap

##  Behaves exactly as a TransformListener, but automatically
#   sets the stamp=now, waits for the transform, and catches exceptions.
#   Strongly recommended that you use this instead of a TransformListener, every time
class SimpleTransformListener(TransformListener):

    @catchTFErrors(5)
    def transformPoint(self,target_frame,ps):
        now = rospy.Time.now()
        self.waitForTransform(target_frame,ps.header.frame_id,rospy.Time.now(),rospy.Duration(5.0))
        ps.header.stamp = now
        return TransformListener.transformPoint(self,target_frame,ps)

    @catchTFErrors(5)
    def transformPose(self,target_frame,ps):
        now = rospy.Time.now()
        self.waitForTransform(target_frame,ps.header.frame_id,rospy.Time.now(),rospy.Duration(5.0))
        ps.header.stamp = now
        return TransformListener.transformPose(self,target_frame,ps)

    @catchTFErrors(5)
    def transformQuaternion(self,target_frame,quat):
        now = rospy.Time.now()
        self.waitForTransform(target_frame,quat.header.frame_id,rospy.Time.now(),rospy.Duration(5.0))
        quat.header.stamp = now
        return TransformListener.transformQuaternion(self,target_frame,quat)

    @catchTFErrors(5)
    def transformFrames(self, source_frame, target_frame):
        now = rospy.Time.now()
        self.waitForTransform(source_frame, target_frame, now, rospy.Duration(5.0))
        return self.lookupTransform(source_frame, target_frame, now)

"""
Utility functions for using points/poses
"""

def array_to_point(nparr):
    pt = Point()
    pt.x = nparr[0]
    pt.y = nparr[1]
    pt.z = nparr[2]
    return pt

def point_to_array(pt):
    return nparray([pt.x,pt.y,pt.z])

def array_to_quaternion(nparr):
    quat = Quaternion()
    quat.x = nparr[0]
    quat.y = nparr[1]
    quat.z = nparr[2]
    quat.w = nparr[3]
    return quat

def quaternion_to_array(quat):
    return nparray([quat.x,quat.y,quat.z,quat.w])

def rpy_to_quaternion(roll,pitch,yaw):
    quatarray = transformations.quaternion_from_euler(roll,pitch,yaw)
    return array_to_quaternion(quatarray)

def quaternion_to_rpy(quat):
    quatarray = quaternion_to_array(quat)
    rpyarray = transformations.euler_from_quaternion(quatarray)
    return (rpyarray[0],rpyarray[1],rpyarray[2])

def xyzrpy_to_pose(x,y,z,roll,pitch,yaw):
    pose = Pose()
    pose.position = array_to_point([x,y,z])
    pose.orientation = rpy_to_quaternion(roll,pitch,yaw)
    return pose

def pose_to_xyzrpy(pose):
    [x,y,z] =  point_to_array(pose.position)
    [roll,pitch,yaw] = quaternion_to_rpy(pose.orientation)
    return (x,y,z,roll,pitch,yaw)

def xyz_to_point(x,y,z):
    pt = Point()
    pt.x = x
    pt.y = y
    pt.z = z
    return pt

def point_to_xyz(pt):
    return [pt.x,pt.y,pt.z]

def stamp(el):
    el.header.stamp = rospy.Time.now()

def set_frame(el,frame):
    el.header.frame_id = frame

def quaternion_multiply(q1, q2):
    return transformations.quaternion_multiply(np.array([q1[3], q1[0], q1[1], q1[2]]),
            np.array([q2[3], q2[0], q2[1], q2[2]]))

def rotate_by_quaternion(v3, q):
    # Rotate 3D vector by quaternion
    # Takes numpy arrays as arguments
    v = np.mat(np.hstack((v3, 1.0)))
    v = v.T
    q = np.mat(transformations.quaternion_matrix(q))
    v = q*v
    v = v[0:3]
    v = v.T
    v = np.array(v.tolist()[0])
    return v

    
