#!/usr/bin/env python3

# Copryight (c) 2020 Michael Ferguson
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

"""
Usage: set_torso_pose.py <pose>
  <pose> is the height of the torso (0.0 to 0.35m)
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FollowJointTrajectoryClient(Node):

    def __init__(self):
        super().__init__("set_torso_pose")
        self._action_client = ActionClient(self, FollowJointTrajectory, 'torso_controller/follow_joint_trajectory')

    def run(self, trajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(seconds=0).to_msg()
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal)


if __name__=='__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        exit(-1)

    trajectory = JointTrajectory()
    trajectory.joint_names = ["torso_lift_joint"]
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [float(sys.argv[1])]
    trajectory.points[0].velocities = [0.0]
    trajectory.points[0].accelerations = [0.0]
    trajectory.points[0].time_from_start = Duration(seconds=7).to_msg()  # we can get anywhere by 7s

    rclpy.init()
    action_client = FollowJointTrajectoryClient()
    action_client.run(trajectory)
