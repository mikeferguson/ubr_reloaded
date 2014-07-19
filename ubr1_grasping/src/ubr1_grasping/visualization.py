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

## @package ubr1_grasping.visualization ROS Visualization of grasping data.

import rospy
from visualization_msgs.msg import Marker, MarkerArray

## @brief ROS visualization of grasps
class GraspVisualizer:

    ## @brief Create the visualizer, rospy.init_node must have been called before this.
    def __init__(self):
        self.pub = rospy.Publisher('grasps', MarkerArray, queue_size=10, latch=True)
        self.max_marker_id = 0

    ## @brief Publish a visualization of a set of grasps
    def publish(self, grasps, obj = None):
        msg = MarkerArray()

        self.marker_id = 0  # reset marker counter
        if obj and len(obj.primitives) > 0 and len(obj.primitive_poses) > 0:
            m = Marker()
            m.header = obj.header
            m.ns = "object"
            m.id = self.marker_id
            self.marker_id += 1
            m.type = m.CUBE
            m.action = m.ADD
            m.pose = obj.primitive_poses[0]
            m.scale.x = obj.primitives[0].dimensions[0]
            m.scale.y = obj.primitives[0].dimensions[1]
            m.scale.z = obj.primitives[0].dimensions[2]
            m.color.r = 0
            m.color.g = 0
            m.color.b = 1
            m.color.a = 0.8
            msg.markers.append(m)

        for grasp in grasps:
            msg.markers.append(self.get_gripper_marker(grasp.grasp_pose, grasp.grasp_quality))

        self.pub.publish(msg)            

    def get_gripper_marker(self, pose_stamped, quality):
        m = Marker()
        m.header = pose_stamped.header
        m.ns = "gripper"
        m.id = self.marker_id
        self.marker_id += 1
        m.type = m.MESH_RESOURCE
        m.action = m.ADD
        m.pose = pose_stamped.pose
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        if quality < 0.5:
            m.color.r = quality * 2
            m.color.g = 0
        else:
            m.color.r = 0
            m.color.g = (quality - 0.5) * 2
        m.color.b = 0
        m.color.a = 0.8
        m.mesh_resource = "package://ubr1_description/meshes/gripper_link.STL"
        return m

