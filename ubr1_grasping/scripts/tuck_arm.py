#!/usr/bin/env python

"""
Tuck UBR-1 arm using MoveIt!
"""

import argparse
import rospy
from moveit_python import *

# definitions of pose
joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
joints_tucked  = [0.0, -1.4486, 1.3439, -2.8327, -1.8119, 0.0, -1.6571, 0.0]
joints_untucked  = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--untuck", "-u", help="Untuck the arm", action="store_true")
    parser.add_argument("--plan", help="Only do planning, no execution", action="store_true")
    args = parser.parse_args()

    if args.untuck:
        print("Not yet supported")
    else:
        rospy.init_node("tuck_arm_moveit")
        m = MoveGroupInterface("arm_with_torso", "base_link", plan_only=args.plan)
        m.moveToJointPosition(joint_names, joints_tucked)
        
