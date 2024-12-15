#!/usr/bin/env python3

# Copyright 2024, Michael Ferguson
# Copyright 2013-2014, Unbounded Robotics, Inc.
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

"""
This script is used to determine joint accelerations on the UBR-1.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pylab import figure, legend, title, text, show


def float_from_msg(msg):
    return msg.sec + (msg.nanosec / 1e9)


# Results from running this on UBR-1 on 12/15/2024:
# (NOTE: no payload in gripper)
#
#  shoulder_pan_joint:   14.027
#  shoulder_lift_joint:  9.807
#  upperarm_roll_joint:  12.173
#  elbow_flex_joint:     10.094
#  forearm_roll_joint:   11.907
#  wrist_flex_joint:     10.645
#  wrist_roll_joint:     11.647
class AccelerationTest(Node):

    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint"
    ]

    # Test shoulder pan acceleration by moving quickly from one side to the other
    ready_shoulder_pan = [1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    test_shoulder_pan = [-1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Test shoulder lift by quickly lifting
    ready_shoulder_lift = [0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
    test_shoulder_lift = [0.0, -0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Test elbow flex by quickly contracting it against gravity
    ready_elbow_flex = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    test_elbow_flex = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
    # Test upperarm roll by moving from horizontal to vertical with elbow flexed
    ready_upperarm_roll = [0.0, 0.0, -1.57, -1.57, 0.0, 0.0, 0.0]
    test_upperarm_roll = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
    # Test wrist flex by quickly contracting it against gravity
    ready_wrist_flex = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    test_wrist_flex = [0.0, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0]
    # Test forearm roll by moving from horizontal to vertical with wrist flexed
    ready_forearm_roll = [0.0, 0.0, 0.0, 0.0, -1.57, -1.57, 0.0]
    test_forearm_roll = [0.0, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0]
    # Test wrist roll by rotating
    ready_wrist_roll = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    test_wrist_roll = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.14]

    # Mapping which movement feedback index is used for which joint
    joint_movement_idx = [1, 3, 7, 5, 11, 9, 13]

    def __init__(self):
        super().__init__("test_accelerations")

        self.movements = [
            ("Preparing shoulder_pan_joint", self.ready_shoulder_pan, 5e9),
            ("Testing shoulder_pan_joint", self.test_shoulder_pan, 2e8),
            ("Preparing shoulder_lift_joint", self.ready_shoulder_lift, 5e9),
            ("Testing shoulder_lift_joint", self.test_shoulder_lift, 2e8),
            ("Preparing elbow_flex_joint", self.ready_elbow_flex, 5e9),
            ("Testing elbow_flex_joint", self.test_elbow_flex, 2e8),
            ("Preparing upperarm_roll_joint", self.ready_upperarm_roll, 5e9),
            ("Testing upperarm_roll_joint", self.test_upperarm_roll, 2e8),
            ("Preparing wrist_flex_joint", self.ready_wrist_flex, 5e9),
            ("Testing wrist_flex_joint", self.test_wrist_flex, 2e8),
            ("Preparing forearm_roll_joint", self.ready_forearm_roll, 5e9),
            ("Testing forearm_roll_joint", self.test_forearm_roll, 2e8),
            ("Preparing wrist_roll_joint", self.ready_wrist_roll, 5e9),
            ("Testing wrist_roll_joint", self.test_wrist_roll, 2e8),
        ]
        self.movement_feedback = []

        self.feedback = []
        self.client = SimpleActionClient(self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory")

        self.sendNext()

    def sendNext(self):
        if len(self.movements) == 0:
            self.analyze()
            return
        msg, pose, nanoseconds = self.movements[0]
        self.movements = self.movements[1:]
        self.get_logger().info(msg)
        self.send_trajectory(pose, nanoseconds)

    def send_trajectory(self, pose, nanoseconds):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = pose
        trajectory.points[0].velocities = [0.0 for i in trajectory.joint_names]
        trajectory.points[0].accelerations = [0.0 for i in trajectory.joint_names]
        trajectory.points[0].time_from_start = Duration(nanoseconds=nanoseconds).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(seconds=2).to_msg()

        try:
            result = self.client.send_goal(goal,
                                           result_callback=self.resultCb,
                                           feedback_callback=self.feedbackCb)
        except:
            self.get_logger().error('Go to has failed!')

    def feedbackCb(self, msg):
        self.feedback.append(msg)

    def resultCb(self, result_code, result):
        self.movement_feedback.append(self.feedback)
        self.feedback = []
        self.sendNext()

    def analyze(self):
        col = 1
        row = 1
        fig = figure()
        for i in range(len(self.joint_names)):
            name = self.joint_names[i]
            movement_idx = self.joint_movement_idx[i]
            self.get_logger().info('Analyzing %s' % name)

            pos = list()
            vel = list()
            effort = list()

            feedback = self.movement_feedback[movement_idx]

            # Determine acceleration from dead start
            start_time = float_from_msg(feedback[0].actual.time_from_start)
            max_accel = 0.0

            for f in feedback:
                pos.append(f.actual.positions[i])
                vel.append(f.actual.velocities[i])
                effort.append(f.actual.effort[i])

                # Determine max acceleration
                dv = f.actual.velocities[i]
                t = float_from_msg(f.actual.time_from_start) - start_time
                try:
                    acc = abs(dv / t)
                    if acc > max_accel:
                        max_accel = acc
                except ZeroDivisionError:
                    pass

            ax = fig.add_subplot(3, 3, i + 1)
            ax.plot(pos)
            ax.plot(vel)
            legend(('Position', 'Velocity'))
            ax2 = ax.twinx()
            ax2.plot(effort, 'r')
            title(name)
            text(0.05, 0.05, 'Accel %f' % max_accel, transform=ax.transAxes)

            col += 1
            if col > 3:
                col = 1
                row += 1
        show()
        pass

if __name__ == "__main__":
    rclpy.init()
    b = AccelerationTest()
    rclpy.spin(b)
