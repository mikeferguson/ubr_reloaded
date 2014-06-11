#!/usr/bin/env python

# Copyright 2013-2014, Unbounded Robotics, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Unbounded Robotics, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Michael Ferguson

import argparse
import copy
import math
import sys

import rospy
import actionlib
from moveit_python import *
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from grasping_msgs.msg import *
from moveit_msgs.msg import MoveItErrorCodes, PlaceLocation

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
    "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
ready_pose = [-1.393150, -0.103543, -1.573480, -1.608378, -0.458660, 0.0, -2.611218]

def move_to_ready(interface):
   result = interface.moveToJointPosition(joint_names, ready_pose)
   if result.error_code.val != 1:
       rospy.sleep(1.0)
       rospy.logerr("Move arm to ready position failed, trying again...")
       result = interface.moveToJointPosition(joint_names, ready_pose, 0.02)

if __name__=="__main__":
    parser = argparse.ArgumentParser(description="Simple demo of pick and place")
    parser.add_argument("--objects", help="Just do object perception", action="store_true")
    parser.add_argument("--all", help="Just do object perception, but insert all objects", action="store_true")
    parser.add_argument("--once", help="Run once.", action="store_true")
    parser.add_argument("--ready", help="Move the arm to the ready position.", action="store_true")
    parser.add_argument("--plan", help="Only do planning, no execution", action="store_true")
    args, unknown = parser.parse_known_args()

    rospy.init_node("pick_and_place_demo")
    move_group = MoveGroupInterface("arm", "base_link", plan_only = args.plan)

    # if all we want to do is prepare the arm, do it now
    if args.ready:
        move_to_ready(move_group)
        exit(0)

    scene = PlanningSceneInterface("base_link")
    pickplace = PickPlaceInterface("arm", "gripper", plan_only = args.plan, verbose = True)

    rospy.loginfo("Connecting to basic_grasping_perception/find_objects...")
    find_objects = actionlib.SimpleActionClient("basic_grasping_perception/find_objects", FindGraspableObjectsAction)
    find_objects.wait_for_server()
    rospy.loginfo("...connected")

    while not rospy.is_shutdown():
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        find_objects.send_goal(goal)
        find_objects.wait_for_result(rospy.Duration(5.0))
        find_result = find_objects.get_result()

        rospy.loginfo("Found %d objects" % len(find_result.objects))

        # remove all previous objects
        for name in scene.getKnownCollisionObjects():
            scene.removeCollisionObject(name, False)
        for name in scene.getKnownAttachedObjects():
            scene.removeAttachedObject(name, False)
        scene.waitForSync()
        # clear colors
        scene._colors = dict()

        # insert objects, find the one to grasp
        the_object = None
        the_object_dist = 0.85
        count = -1
        for obj in find_result.objects:
            count += 1
            scene.addSolidPrimitive("object%d"%count,
                                    obj.object.primitives[0],
                                    obj.object.primitive_poses[0],
                                    wait = False)

            # has to be at right hieght
            if obj.object.primitive_poses[0].position.z > 1.5 or \
               obj.object.primitive_poses[0].position.z < 0.75:
                continue
            # has to be a cube
            if obj.object.primitives[0].type != obj.object.primitives[0].BOX:
                continue
            # has to be right size to be a cube
            if obj.object.primitives[0].dimensions[0] > 0.067 or obj.object.primitives[0].dimensions[0] < 0.055 or \
               obj.object.primitives[0].dimensions[1] > 0.067 or obj.object.primitives[0].dimensions[1] < 0.055 or \
               obj.object.primitives[0].dimensions[2] > 0.067 or obj.object.primitives[0].dimensions[2] < 0.043:
                continue
            dx = obj.object.primitive_poses[0].position.x
            dy = obj.object.primitive_poses[0].position.y
            d = math.sqrt((dx * dx) + (dy * dy))
            if d < the_object_dist:
                the_object_dist = d
                the_object = count

        if the_object == None:
            rospy.logerr("Nothing to grasp! try again...")
            continue

        # insert table
        for obj in find_result.support_surfaces:
            # extend surface to floor
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            2.0, # make table wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            scene.addSolidPrimitive(obj.name,
                                    obj.primitives[0],
                                    obj.primitive_poses[0],
                                    wait = False)

        obj_name = "object%d"%the_object

        # sync
        scene.waitForSync()

        # set color of object we are grabbing
        scene.setColor(obj_name, 223.0/256.0, 90.0/256.0, 12.0/256.0)  # orange
        scene.setColor(find_result.objects[the_object].object.support_surface, 0, 0, 0)  # black
        scene.sendColors()

        # exit now if we are just doing object update
        if args.objects:
            if args.once:
                exit(0)
            else:
                continue

        # check grasps
        grasps = find_result.objects[the_object].grasps
        if len(grasps) < 1:
            rospy.logerr("No grasps planned")
            continue
        support_surface = find_result.objects[the_object].object.support_surface

        # call move_group to pick the object
        rospy.loginfo("Beginning to pick.")
        while True:
            pick_result = pickplace.pickup(obj_name, grasps, support_name=support_surface)
            if pick_result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Pick succeeded")
                break
            elif pick_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                rospy.logerr("Pick failed in the planning stage, try again...")
                rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                continue
            elif pick_result.error_code.val == MoveItErrorCodes.CONTROL_FAILED or \
                 pick_result.error_code.val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE or \
                 pick_result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                rospy.logerr("Pick failed during execution, attempting to cleanup.")
                if obj_name in scene.getKnownAttachedObjects():
                    rospy.loginfo("Pick managed to grab object, retreat must have failed, continuing anyways")
                    break
                else:
                    rospy.loginfo("Pick did not grab object, try again...")
                    continue
            else:
                rospy.logerr("Pick failed with error code: %d. Will retry..." % pick_result.error_code.val)
                continue

        # create a set of place locations for the cube
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = find_result.objects[the_object].object.primitive_poses[0]
        l.place_pose.header.frame_id = find_result.objects[the_object].object.header.frame_id
        # invert the y of the pose
        l.place_pose.pose.position.y = -l.place_pose.pose.position.y
        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 90 degrees in yaw direction
        l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 1.57)
        places.append(copy.deepcopy(l))
        l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 1.57)
        places.append(copy.deepcopy(l))
        l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 1.57)
        places.append(copy.deepcopy(l))

        # drop it like it's hot
        rospy.loginfo("Beginning to place.")
        while True:
            place_result = pickplace.place(obj_name, places, support_name=support_surface)
            if place_result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place succeeded")
                break
            elif place_result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                rospy.logerr("Place failed in the planning stage, try again...")
                rospy.sleep(0.5)  # short sleep to try and let state settle a bit?
                continue
            elif place_result.error_code.val == MoveItErrorCodes.CONTROL_FAILED or \
                 place_result.error_code.val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE or \
                 place_result.error_code.val == MoveItErrorCodes.TIMED_OUT:
                rospy.logerr("Place failed during execution, attempting to cleanup.")
                if obj_name in scene.getKnownAttachedObjects():
                    rospy.loginfo("Place did not place object, approach must have failed, will retry...")
                    continue
                else:
                    rospy.loginfo("Object no longer in gripper, must be placed, continuing...")
                    break
            else:
                rospy.logerr("Place failed with error code: %d. Will retry..." % place_result.error_code.val)
                continue

        # place arm back at side
        move_to_ready(move_group)
        rospy.loginfo("Ready...")

        # rinse and repeat
        if args.once:
            exit(0)

