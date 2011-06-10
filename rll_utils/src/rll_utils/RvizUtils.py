#!/usr/bin/env python
import roslib
roslib.load_manifest("rll_utils")
import rospy
from visualization_msgs.msg import Marker

def place_marker(point, pub, _id=0, _type = Marker.CUBE, r = 0, g = 1, b = 0, xscale = .03, yscale=.03, zscale = .03):
    """
    Publishes a visualization marker for Rviz. The publisher needs to be supplied just because I've noticed
    that sometimes when you try to make the publisher in this method, it doesn't work for whatever reason.
    """
    marker = Marker(type=_type, action=Marker.ADD)
    marker.header.frame_id = point.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.pose.position = point.point
    marker.pose.orientation.w = 1
    marker.scale.x = xscale
    marker.scale.y = yscale
    marker.scale.z = zscale
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1
    marker.id = _id
    marker.lifetime = rospy.Duration()
    pub.publish(marker)
    print "Placed a marker"

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

