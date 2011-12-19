#!/usr/bin/env python

import roslib
roslib.load_manifest('rll_utils')
import rospy
import actionlib
from arm_navigation_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal

from math import pi
import TFUtils

def arm_error_code_map():
    """Ugly one liner to get a reverse mapping for arm navigation error codes"""
    return dict((getattr(ArmNavigationErrorCodes, key), key) for key in filter(lambda x: x==x.upper(), dir(ArmNavigationErrorCodes)))

def create_move_arm_goal(x, y, z, roll, pitch, yaw, frame='base_footprint', arm='r', target_link='%s_wrist_roll_link'):
    """
    Populates a MoveArmGoal in a convenient manner.
    """
    mg = MoveArmGoal()
    mg.motion_plan_request.group_name = ('right_arm' if arm == 'r' else 'left_arm')
    mg.motion_plan_request.num_planning_attempts = 5
    mg.planner_service_name = '/ompl_planning/plan_kinematic_path'
    mg.motion_plan_request.allowed_planning_time = rospy.Duration(100.0)
    mg.accept_partial_plans = True
    #mg.motion_plan_request.planner_id = 'kinematic::RRT'
    link_name = target_link
    try:
        link_name = target_link % arm
    except TypeError:
        pass
    pc = PositionConstraint()
    pc.header.stamp = rospy.Time.now()
    pc.header.frame_id = frame
    pc.link_name = link_name
    pc.position.x = x
    pc.position.y = y
    pc.position.z = z
    
    ps = PointStamped()
    ps.header.frame_id = pc.header.frame_id
    ps.point = pc.position
    pc.constraint_region_shape.type = Shape.BOX
    pc.constraint_region_shape.dimensions = [0.02, 0.02, 0.02]
    pc.constraint_region_orientation.w = 1.0

    mg.motion_plan_request.goal_constraints.position_constraints.append(pc)
    
    oc = OrientationConstraint()
    oc.header.stamp = rospy.Time.now()
    oc.header.frame_id = frame
    oc.link_name = link_name
    oc.orientation = TFUtils.rpy_to_quaternion(roll, pitch, yaw)
    oc.absolute_roll_tolerance = .04
    oc.absolute_pitch_tolerance =.04
    oc.absolute_yaw_tolerance = .04
    oc.weight = 1.0

    mg.motion_plan_request.goal_constraints.orientation_constraints.append(oc)
    return mg

def send_move_arm_goal(mg):
    err_map = arm_error_code_map();
    arm = mg.motion_plan_request.group_name
    print 'Sending goal to', '/move_%s'%arm
    move_arm = actionlib.SimpleActionClient('/move_%s'%arm, MoveArmAction)
    move_arm.wait_for_server()
    move_arm.send_goal(mg)
    counter = 0
    while move_arm.simple_state != 2:
        rospy.sleep(1.0)
        counter = counter + 1
        if counter % 20 == 0:
            print counter, 'seconds have passed. State is', move_arm.get_state()
    print 'Move arm server finished'
    result = move_arm.get_result()
    print result
    if result.error_code.val != ArmNavigationErrorCodes.SUCCESS:
        print 'Motion planning failed with error', err_map[result.error_code.val]
    else:
        print 'Motion planning request succeeded'
    return result
    
def move_arm_simple(x, y, z, roll, pitch, yaw, frame, arm):
    mg = create_move_arm_goal(x, y, z, roll, pitch, yaw, frame, arm);
    return send_move_arm_goal(mg)

def move_arm_pt(pt, roll, pitch, yaw, frame, arm):
    mg = create_move_arm_goal(pt.x, pt.y, pt.z, roll, pitch, yaw, frame, arm);
    return send_move_arm_goal(mg)

def visualize_contacts(result):
    """
    This allows you to see where the planner reported contact with the collision
    environment.
    """
    for contact in result.contacts:
        print contact

def test_move_arm():
    rospy.init_node('test_move_arm_py')
    return move_arm_simple(0.75, 0.0, -0.0, 0, pi/2, 0, 'torso_lift_link', 'r')

if __name__ == '__main__':
    test_move_arm()

