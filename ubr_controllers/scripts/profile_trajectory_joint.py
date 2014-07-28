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

"""
This script is used to profile a particular joint that is
controlled by a joint trajectory action server.
"""

import argparse
import rospy
import actionlib
from copy import deepcopy
from control_msgs.msg import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import *

# visualization
from pylab import *

class JointTest:
    arm_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    torso_joints = ["torso_lift_joint"]
    head_joints = ["head_pan_joint", "head_tilt_joint"]

    controller_joints = dict()
    controller_joints["arm_controller"] = arm_joints
    controller_joints["torso_controller"] = torso_joints
    controller_joints["head_controller"] = head_joints

    ## @brief Construct a JointTest
    ## @param name Name of the joint to control
    ## @param velocity Constant velocity of the joint to control
    def __init__(self, name, velocity):
        self.name = name
        self.velocity = velocity

        for controller in self.controller_joints.keys():
            if name in self.controller_joints[controller]:
                # Set joint names
                self.joints = self.controller_joints[controller]

                # Connect to controller for playback
                rospy.loginfo("Waiting for %s..." % controller)
                self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % controller, FollowJointTrajectoryAction)
                self.client.wait_for_server()
                rospy.loginfo("...connected.")

                # Subscribe to feedback from arm_controller
                rospy.Subscriber("%s/follow_joint_trajectory/feedback" % controller, FollowJointTrajectoryFeedback, self.feedbackCb)
                self.feedback = list()

        # Subscribe to joint_state
        self.joint_positions = dict()
        rospy.Subscriber("joint_states", JointState, self.stateCb)

    def stateCb(self, msg):
        for i in range(len(msg.name)):
            self.joint_positions[msg.name[i]] = msg.position[i]

    ## @brief Get feedback from controller
    def feedbackCb(self, msg):
        self.feedback.append(msg.feedback)

    ## @brief Actually run the test
    ## @param constant_duration Time to run at constant velocity
    def run(self, constant_duration):
        # Setup trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        idx = self.joints.index(self.name)

        trajectory.points.append(JointTrajectoryPoint())
        try:
            trajectory.points[0].positions = [self.joint_positions[name] for name in trajectory.joint_names]
        except:
            rospy.logerr("Missing joint information")
            return False
        trajectory.points[0].velocities = [0.0 for i in trajectory.joint_names]
        trajectory.points[0].velocities[idx] = self.velocity
        trajectory.points[0].accelerations = [0.0 for i in trajectory.joint_names]
        trajectory.points[0].time_from_start = rospy.Duration(0.0)
        # Create second point with joint movement
        trajectory.points.append(deepcopy(trajectory.points[0]))
        trajectory.points[1].time_from_start = rospy.Duration(constant_duration)
        trajectory.points[1].positions[idx] += self.velocity * constant_duration
        # Setup action goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)
        try:
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(2*constant_duration))
        except:
            rospy.logerr('Failed to send trajectory action goal!')
            return False

    def plot(self):
        idx = self.feedback[0].joint_names.index(self.name)

        v = list()
        e = list()
        for f in self.feedback:
            v.append(f.actual.velocities[idx])
            e.append(f.actual.effort[idx])

        fig = figure()
        ax = fig.add_subplot(1,1,1)
        time = [i/50.0 for i in range(len(v))]
        ax.plot(time, label = 'Velocity')
        ax.legend(('Velocity'))
        ax2 = ax.twinx()
        ax2.plot(time, e, 'r')
        ax2.legend(('Effort'), loc=0)
        show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', help='The name of the joint to profile')
    parser.add_argument("-v", help="Angular velocity, in rad/s", type=float)
    parser.add_argument("-t", help="Duration to run at constant velocity after ramping, in seconds", type=float, default=2.0)
    args, unknown = parser.parse_known_args()

    rospy.init_node("profile_trajectory_joint")

    j = JointTest(args.name, args.v)
    rospy.sleep(1)
    j.run(args.t)
    j.plot()
