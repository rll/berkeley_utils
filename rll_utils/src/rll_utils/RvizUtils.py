#!/usr/bin/env python
import roslib
roslib.load_manifest("rll_utils")
import rospy
from tf.transformations import quaternion_multiply, quaternion_about_axis
from visualization_msgs.msg import Marker
from geometry_msgs.msg import *
from rll_utils.TFUtils import rpy_to_quaternion, quaternion_to_array,\
        array_to_quaternion
from math import pi

def place_marker(point, pub, _id=0, _type=Marker.CUBE, ns='basic_shapes',\
        r=0, g=1, b=0, xscale=.03, yscale=.03, zscale=.03,\
        orientation=Quaternion(0,0,0,1), text=''):
    """
    Publishes a visualization marker for Rviz. The publisher needs to be supplied just because I've noticed
    that sometimes when you try to make the publisher in this method, it doesn't work for whatever reason.
    """
    marker = Marker(type=_type, action=Marker.ADD)
    marker.ns = ns
    marker.header.frame_id = point.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.pose.position = point.point
    marker.pose.orientation = orientation
    marker.scale.x = xscale
    marker.scale.y = yscale
    marker.scale.z = zscale
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1
    marker.id = _id
    marker.text = text
    marker.lifetime = rospy.Duration()
    pub.publish(marker)
#    rospy.loginfo("Placed a marker")

def place_arrow(point, pub, _id, ns='basic_shapes', rgb=(0, 1, 0), scale=(0.1, 0.1, 0.1), orientation=Quaternion(0,0,0,1),text=''):
    place_marker(point, pub, _id, Marker.ARROW, ns=ns, r=rgb[0], g=rgb[1], b=rgb[2],\
            xscale=scale[0], yscale=scale[1], zscale=scale[2], orientation=orientation)
    if text != '':
        place_marker(point, pub, _id, Marker.TEXT_VIEW_FACING, ns='labels',\
                r=0, g=0, b=0, xscale=0, yscale=0, zscale=0.1,\
                orientation=orientation, text=text)

def draw_trajectory():
    """
    #TODO: Draw arm and base trajectories to visualize action and confirm safety
    before execution.
    """
    pass

def draw_axes(pub, id, ns, pose_stamped, scale=(0.05, 0.1, 0.1), text=""):
    ps = PointStamped()
    ps.header = pose_stamped.header
    ps.point = pose_stamped.pose.position
    q_x = quaternion_to_array(pose_stamped.pose.orientation)
    q_y = quaternion_multiply(q_x, quaternion_about_axis(pi/2, (0, 1, 0)))
    q_z = quaternion_multiply(q_x, quaternion_about_axis(pi/2, (0, 0, 1)))
    if id > 999:
        rospy.logerr('Currently can\'t visualize markers with id > 999.')
        return
    place_arrow(ps, pub, id, ns, (1, 0, 0),\
            scale, array_to_quaternion(q_x))
    place_arrow(ps, pub, id + 1000, ns, (0, 1, 0),\
            scale, array_to_quaternion(q_y))
    place_arrow(ps, pub, id + 2000, ns, (0, 0, 1),\
            scale, array_to_quaternion(q_z), text=text)

def place_lines(points, pub, _id=0, r = 0, g = 1, b = 0, xscale = .02, yscale=.02, zscale = .02):
    """
    Publishes a set of lines connecting the given points to Rviz.
    """
    marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
    marker.header.frame_id = points[0].header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.pose.orientation.w = 1
    marker.scale.x = xscale
    marker.scale.y = yscale
    marker.scale.z = zscale
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1
    marker.id = _id
    marker.points = map(lambda x: x.point, points)

    marker.lifetime = rospy.Duration()
    pub.publish(marker)
    print "Placed a marker"

