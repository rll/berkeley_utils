#!/usr/bin/env python
import roslib; roslib.load_manifest('stereo_click')
from geometry_msgs.msg import PointStamped
import rospy
import sys
import os

if len(sys.argv) < 3:
    print 'Usage: rosrun stereo_click replay_points_3d.py <points_3d_file> <frame_id>'
    print '- <points_3d_file> should contain one point per line with x,y,z separated by spaces'
    print '- frame_id is e.g. "base_footprint"'

rospy.init_node('replay_points_3d')

points_3d = open(sys.argv[1], 'r').read()
pub = rospy.Publisher('/stereo_points_3d', PointStamped)
frame_id = sys.argv[2]
rospy.sleep(1.0) # Need publisher to initialize
for pt in points_3d.split('\n'):
    pt = pt.split()
    if len(pt) != 3:
        continue
    ps = PointStamped()
    ps.header.frame_id = frame_id
    ps.point.x = float(pt[0])
    ps.point.y = float(pt[1])
    ps.point.z = float(pt[2])
    pub.publish(ps)

