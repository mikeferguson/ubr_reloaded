#!/usr/bin/env python

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
Usage: set_head_pose.py <pan> <tilt>
  <pan> is the head pan location (-1.57 to 1.57 radians)
  <tilt> is the head tilt location (-1.57 to 0.785 radians)
"""

import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__=='__main__':
    if len(sys.argv) < 3:
        print(__doc__)
        exit(-1)

    rospy.init_node('set_head_pose')

    rospy.loginfo('Waiting for head_controller...')
    client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo('...connected.')

    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [float(sys.argv[1]), float(sys.argv[2])]
    trajectory.points[0].velocities = [0.0, 0.0]
    trajectory.points[0].time_from_start = rospy.Duration(1.0)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(20.0))
    rospy.loginfo('...done')
    
