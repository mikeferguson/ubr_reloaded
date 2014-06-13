#!/usr/bin/env python

import copy
import math
import sys

import rospy
import actionlib
from moveit_python import *
from moveit_python.geometry import *

from geometry_msgs.msg import *
from grasping_msgs.msg import *
from moveit_msgs.msg import *
from trajectory_msgs.msg import *

def create_cube(interface):
    interface.addCube("cube", 0.06, 0.25, -0.25, 0.03)
    return interface._objects["cube"]

def make_gripper_posture(pose):
    t = JointTrajectory()
    t.joint_names = ["left_gripper_joint", "left_gripper_joint"]
    tp = JointTrajectoryPoint()
    tp.positions = [pose/2.0 for j in t.joint_names]
    tp.effort = [28.0 for j in t.joint_names]
    t.points.append(tp)
    return t

def make_gripper_translation(min_dist, desired, axis=1.0):
    g = GripperTranslation()
    g.direction.vector.x = axis
    g.direction.header.frame_id = "wrist_roll_link"
    g.min_distance = min_dist
    g.desired_distance = desired
    return g

def make_grasps(pose_stamped):
    grasps = list()
    g = Grasp()
    g.pre_grasp_posture = make_gripper_posture(0.09)
    g.grasp_posture = make_gripper_posture(0.0)
    g.pre_grasp_approach = make_gripper_translation(0.1, 0.15)
    g.post_grasp_retreat = make_gripper_translation(0.1, 0.15, -1.0)
    g.grasp_pose = pose_stamped
    g.grasp_pose.pose.position.z += .110 + .01

    q = quaternion_from_euler(0, 1.57, 0.0)
    g.grasp_pose.pose.orientation.x = q[0]
    g.grasp_pose.pose.orientation.y = q[1]
    g.grasp_pose.pose.orientation.z = q[2]
    g.grasp_pose.pose.orientation.w = q[3]
    g.id = "overhead"
    g.grasp_quality = 1.0
    grasps.append(g)
    return grasps

if __name__=="__main__":
    rospy.init_node("generate_grasp_test")

    scene = PlanningSceneInterface("base_link")
    pickplace = PickPlaceInterface("arm", "gripper", verbose = True)

    cube = create_cube(scene)
    pose_stamped = PoseStamped()
    pose_stamped.pose = cube.primitive_poses[0]
    pose_stamped.header.frame_id = "base_link"
    grasps = make_grasps(pose_stamped)

    rospy.loginfo("Beginning to pick.")
    success, pick_result = pickplace.pick_with_retry("cube", grasps, scene = scene)

