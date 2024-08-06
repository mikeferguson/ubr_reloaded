#!/usr/bin/env python3

# Copyright (c) 2020-2024, Michael Ferguson
# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of the copyright holder nor the names of its
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

from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from robot_controllers_msgs.msg import ControllerState
from robot_controllers_msgs.srv import QueryControllerStates
from sensor_msgs.msg import Joy


class ControllerResetTeleop(Node):

    def __init__(self):
        super().__init__('controller_reset')

        # ROS Service connection to controller state query
        self.client = self.create_client(QueryControllerStates, 'query_controller_states')
        while not self.client.wait_for_service(timeout_sec=1.0):
            print('query_controller_states not available, waiting again...')

        # Connection gripper controller
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_controller/command')
        while not self.gripper_client.wait_for_server(timeout_sec=1.0):
            print('gripper_controller/command not available, waiting again...')

        self.start = []
        self.start.append('arm_controller.gravity_compensation')

        self.stop = []
        self.stop.append('arm_controller.follow_joint_trajectory')
        self.stop.append('arm_with_torso_controller.follow_joint_trajectory')
        self.stop.append('torso_controller.follow_joint_trajectory')
        self.stop.append('head_controller.follow_joint_trajectory')
        self.stop.append('head_controller.point_head')

        button_param = self.declare_parameter('reset_axis', 7)
        self.reset_button = button_param.get_parameter_value().integer_value
        value_param = self.declare_parameter('reset_value', 1.0)
        self.reset_value = value_param.get_parameter_value().double_value

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        try:
            if msg.axes[self.reset_button] == self.reset_value:
                if not self.pressed:
                    self.pressed_last = self.get_clock().now()
                    self.pressed = True
                elif (self.pressed_last and
                      self.get_clock().now() > self.pressed_last + Duration(seconds=1)):
                    self.reset()
                    self.pressed_last = None
            else:
                self.pressed = False
        except KeyError:
            self.get_logger().warn('reset_button is out of range')

    def reset(self):
        # Reset controllers
        req = QueryControllerStates.Request()

        for controller in self.start:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            req.updates.append(state)

        for controller in self.stop:
            state = ControllerState()
            state.name = controller
            state.state = state.STOPPED
            req.updates.append(state)

        self.future = self.client.call_async(req)

        # Disable gripper torque
        goal = GripperCommand.Goal()
        goal.command.max_effort = -1.0
        self.gripper_client.send_goal_async(goal)


if __name__ == '__main__':
    rclpy.init()

    try:
        c = ControllerResetTeleop()
        rclpy.spin(c)
    except KeyboardInterrupt:
        pass
