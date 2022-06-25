#!/usr/bin/env python3

# Copyright (c) 2020-2022, Michael Ferguson
# Copyright (c) 2013-2014 Unbounded Robotics Inc.
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Unbounded Robotics Inc. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNBOUNDED ROBOTICS INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Tilt head for navigation obstacle avoidance.
#

from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

import numpy as np

from transforms3d.quaternions import quat2mat

from control_msgs.action import PointHead
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

class NavHeadController(Node):

    def __init__(self):
        super().__init__("tilt_head_node")

        # tilt head goal to send
        self.goal = PointHead.Goal()
        self.goal.target.header.frame_id = 'base_link'
        self.goal.min_duration = Duration(seconds=1).to_msg()
        self.goal.target.point.x = 1.0
        self.goal.target.point.y = 0.0
        self.goal.target.point.z = 0.0

        # last time our goal was updated (because a plan was recieved)
        self.goal_update_time = None

        # future (which indicates a goal is active)
        self.goal_future = None

        # lock covering goal*
        self.goal_mutex = Lock()

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.get_logger().info("Waiting for point_head action...")
        self.client = ActionClient(self, PointHead, '/head_controller/point_head')
        self.client.wait_for_server()
        self.get_logger().info("...OK")

        self.plan_sub = self.create_subscription(Path, 'GracefulController/local_plan', self.plan_callback, 10)

    def plan_callback(self, msg):
        # get the goal
        try:
            pose_stamped = msg.poses[-1]
        except IndexError:
            self.get_logger().warn("Unable to parse empty plan")
            return
        pose = pose_stamped.pose

        # look ahead one meter
        R = quat2mat([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        point = np.array([1, 0, 0]).reshape((3, 1))
        M = np.dot(R, point)

        p = PointStamped()
        p.header.frame_id = pose_stamped.header.frame_id
        p.point.x += pose_stamped.pose.position.x + M[0, 0]
        p.point.y += pose_stamped.pose.position.y + M[1, 0]
        p.point.z += pose_stamped.pose.position.z + M[2, 0]

        # transform to base_link
        p = self.buffer.transform(p, self.goal.target.header.frame_id)

        # update
        with self.goal_mutex:
            if p.point.x < 0.65:
                self.goal.target.point.x = 0.65
            else:
                self.goal.target.point.x = p.point.x
            if p.point.y > 0.5:
                self.goal.target.point.y = 0.5
            elif p.point.y < -0.5:
                self.goal.target.point.y = -0.5
            else:
                self.goal.target.point.y = p.point.y
            self.goal_update_time = self.get_clock().now()

        # Start moving head (if not already)
        self.update()

    def goal_callback(self, future):
        # This gets called when the goal is accepted or rejected
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Failed to send goal")
            with self.goal_mutex:
                self.goal_future = None
                return

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug("Goal complete")
        else:
            self.get_logger().debug('Goal failed with status: {0}'.format(status))

        with self.goal_mutex:
            self.goal_future = None
            self.result_future = None

        self.update()

    def update(self):
        if self.goal_future:
            self.get_logger().debug("Active tilt head goal - not updating")
            return

        if self.goal_update_time and self.get_clock().now() <= self.goal_update_time + Duration(seconds=1):
            if self.goal.target.point.z == 0.0:
                self.get_logger().debug("Tilting up")
                self.goal.target.point.z = 0.75
            else:
                self.get_logger().debug("Tilting down")
                self.goal.target.point.z = 0.0

            with self.goal_mutex:
                self.goal.target.header.stamp = self.get_clock().now().to_msg()
                self.goal_future = self.client.send_goal_async(self.goal)
                self.goal_future.add_done_callback(self.goal_callback)
        else:
            self.get_logger().debug("No updates")


if __name__=='__main__':
    rclpy.init()

    h = NavHeadController()
    rclpy.spin(h)
    h.destroy_node()
    rclpy.shutdown()
