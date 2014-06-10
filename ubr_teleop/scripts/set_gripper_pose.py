#! /usr/bin/env python

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
Usage: set_gripper_pose.py <pose>
  <pose> is the opening of the gripper, or 'open' or 'closed' for predefined positions
"""

import sys
import rospy
import actionlib
from control_msgs.msg import *

if __name__ == "__main__":
    goal = GripperCommandGoal()

    if len(sys.argv) < 2:
        print(__doc__)
        exit(-1)
    else:
        if sys.argv[1] == "open":
            goal.command.position = 0.09
        elif sys.argv[1] == "closed":
            goal.command.position = 0.0
        else:
            goal.command.position = float(sys.argv[1])
    
    rospy.init_node("open_gripper")
    rospy.loginfo("Waiting for gripper_controller...")
    client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    client.wait_for_server()
    rospy.loginfo("...connected")
    rospy.loginfo("Setting gripper pose to %f" % goal.command.position)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    rospy.loginfo("Results:")
    print(client.get_result())

