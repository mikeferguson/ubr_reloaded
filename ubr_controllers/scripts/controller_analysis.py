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
Run a standard test and analyse the controller response.
"""

import argparse
import math
import pickle
from pylab import *

import rospy
import actionlib

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from control_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import *


class ControllerAnalysis:
    ## Names of joints
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

    ## Four corners of a box
    a = [-1.1251749992370605, 0.8425388336181641, -1.1305437088012695, -1.1773302555084229, 0.6787863969802856, -0.21207284927368164, 0.0809173583984375]
    b = [0.6481070518493652, 0.9207720756530762, 0.79268479347229, -1.0856750011444092, -0.26384496688842773, -0.07037115097045898, -0.09069681167602539]
    c = [-0.9859662055969238, -0.4751507043838501, -1.8277382850646973, -0.8686165809631348, -0.9456992149353027, -0.015148162841796875, -3.1045851707458496]
    d = [0.46671366691589355, -0.6730340719223022, -3.9465489387512207, -0.8747525215148926, 0.6285485029220581, 0.005944252014160156, -3.10075044631958]

    ## @brief Initialize connections 
    def __init__(self, name):
        self.name = name

        # Connect to arm controller for playback
        rospy.loginfo("Waiting for %s..." % self.name)
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % self.name, FollowJointTrajectoryAction)
        self.client.wait_for_server()
        rospy.loginfo("...connected.")

        # Subscribe to feedback from arm_controller
        rospy.Subscriber("%s/follow_joint_trajectory/feedback" % self.name, FollowJointTrajectoryFeedback, self.feedbackCallback)
        self.feedback = list()
        
    ## @brief Get feedback from controller
    def feedbackCallback(self, msg):
        self.feedback.append(msg.feedback)

    ## @brief Send arm to a new pose
    def goto(self, pose, time = 5.0):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = [0.0 for i in trajectory.joint_names]
        trajectory.points[0].velocities = [0.0 for i in trajectory.joint_names]
        trajectory.points[0].accelerations = [0.0 for i in trajectory.joint_names]
        trajectory.points[0].positions = pose
        trajectory.points[0].time_from_start = rospy.Duration(time)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        try:
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(10.0))
        except:
            rospy.logerr('Go to has failed!')

    ## @brief Do analysis of controller feedback
    def analyze(self):
        row = 1
        col = 1
        fig = figure()
        for i in range(len(self.feedback[0].joint_names)):
            name = self.feedback[0].joint_names[i]
            print('analyzing %s' % name)

            p = list()
            v = list()
            e = list()
            for f in self.feedback:
                p.append(f.error.positions[i])
                v.append(f.error.velocities[i])
                e.append(f.actual.effort[i])

            ax = fig.add_subplot(3, 3, i+1)
            ax.plot(p)
            ax.plot(v)
            legend(('Position', 'Velocity'))
            ax2 = ax.twinx()
            ax2.plot(e, 'r')
            title(name)
            text(0.05, 0.05, 'Final Error %f' % self.feedback[-1].error.positions[i], transform=ax.transAxes)

            col += 1
            if col > 3:
                col = 1
                row += 1
        show()

        # analyzed, reset this
        self.feedback = list()

if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--box', help='Do box test, show graph after each move.', action="store_true")
    parser.add_argument('--controller', help='Name of controller', default='arm_controller')
    args = parser.parse_args()

    rospy.init_node('controller_analysis')

    c = ControllerAnalysis(args.controller)

    if args.box:
        c.goto(c.a)
        c.analyze()
        c.goto(c.b)
        c.analyze()
        c.goto(c.c)
        c.analyze()
        c.goto(c.d)
    else:
        rospy.loginfo('Monitoring %s feedback, ctrl-c to stop and show graph' % args.controller)
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.05)
            except:
                pass
    c.analyze()
