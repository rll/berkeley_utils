#!/usr/bin/env python

import roslib
roslib.load_manifest('rll_utils')
import rospy
from rll_utils.RosUtils import get_next_message
from sensor_msgs.msg import JointState

def get_joint_state(joint_name):
    states = get_next_message('joint_states', JointState)
    if joint_name in states.name:
        index = states.name.index(joint_name)
        position = states.position[index]
        velocity = states.velocity[index]
        effort = states.effort[index]
        return (position, velocity, effort)
    else:
        return (_,_,_)

def get_joints_states(joint_names):
    return map(get_joint_state, joint_names)

if __name__ == "__main__":
    rospy.init_node('joint_utils_test')
    print get_joint_state('torso_lift_joint')