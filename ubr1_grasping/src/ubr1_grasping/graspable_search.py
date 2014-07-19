#!/usr/bin/env python

# Copyright 2014, Unbounded Robotics, Inc.
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

## @package ubr1_grasping.graspable_search Python wrapper around ROS-based grasp
##          planning. Used to search available objects and find a graspable one.

import math
import actionlib
import rospy
from grasping_msgs.msg import GraspPlanningAction, GraspPlanningGoal

## @brief A class for searching available objects and finding grasps for the
##        most desirable one that is graspable
class GraspableSearch:

    ## @brief Create a ROS action connection to our perception node.
    ## @param verbose If True, extensive logging data will be generated.
    def __init__(self, verbose = False):
        self.verbose = verbose

        # Init grasp planner ROS connection
        if self.verbose:
            rospy.loginfo("Waiting for shape_grasp_planner/plan_grasp to be available")
        self.client = actionlib.SimpleActionClient("shape_grasp_planner/plan_grasp", GraspPlanningAction)
        self.client.wait_for_server()
        if self.verbose:
            rospy.loginfo("...done")
        self.distance_object = None
        self.obj_index = None
            
    ## @brief Search through objects to find one that is nearest the point
    ##        of interest and also graspable
    ## @returns Tuple consisting of the object to grasp, and the grasps
    def search(self, objects, x, y, z):
        # reset the search
        self.distance_object = list()
        self.obj_index = None

        # determine distance to each object from interest point
        for obj in objects:
            # TODO prescreen ungraspable objects?
            dx = obj.primitive_poses[0].position.x - x
            dy = obj.primitive_poses[0].position.y - y
            dz = obj.primitive_poses[0].position.z - z
            d = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
            self.distance_object.append((d, obj))

        # sort by distance
        self.distance_object = sorted(self.distance_object)
        self.obj_index = 0

        return self.re_search()

    ## @brief Continue planning from before
    def re_search(self):
        if self.obj_index == None:
            # no previous search in progress
            return [None, None]

        # find best result
        for i in range(len(self.distance_object)):
            # already evaluated?
            if i < self.obj_index:
                continue

            # plan grasps
            if self.verbose:
                rospy.loginfo("Planning grasps...")
            goal = GraspPlanningGoal()
            goal.object = self.distance_object[i][1]
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(10.0))
            result = self.client.get_result()
            if self.verbose:
                rospy.loginfo("...done")

            # return result
            if len(result.grasps) > 0:
                self.obj_index = i + 1
                return [goal.object, result.grasps]  # found grasps!

            # next candidate
            if self.verbose:
                rospy.logwarn("No grasps found!")

        # Nothing is graspable!!!
        rospy.logerr("No grasps found and all objects searched!")
        self.obj_index = None
        return [None, None]

