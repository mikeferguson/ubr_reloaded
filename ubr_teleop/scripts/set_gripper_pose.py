#! /usr/bin/env python3

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
Usage: set_gripper_pose.py <pose> <effort=28.0>
  <pose> is the opening of the gripper, or 'open' or 'closed' for predefined positions
  <effort> is the max effort to use.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand


class GripperClient(Node):

    def __init__(self):
        super().__init__("set_gripper_pose")
        self._action_client = ActionClient(self, GripperCommand, 'gripper_controller/command')

    def run(self, position, max_effort):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(__doc__)
        exit(-1)
    else:
        position = 0.0
        max_effort = 28.0
        if len(sys.argv) == 3:
            max_effort = float(sys.argv[2])
        if sys.argv[1] == "open":
            position = 0.09
        elif sys.argv[1] == "closed":
            position = 0.0
        else:
            position = float(sys.argv[1])

        rclpy.init()
        action_client = GripperClient()
        action_client.run(position, max_effort)
