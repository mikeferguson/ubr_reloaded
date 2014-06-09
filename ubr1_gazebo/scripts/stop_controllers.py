#!/usr/bin/env python

from __future__ import print_function

import rospy
import actionlib

from ubr_msgs.srv import *

if __name__ == '__main__':
    rospy.init_node('stop_controllers')
    rospy.wait_for_service('gazebo/update_controllers')
    service = rospy.ServiceProxy('gazebo/update_controllers', UpdateControllers)
    try:
        start = list()
        start.append('arm_controller/gravity_compensation')
        stop = list()
        stop.append('arm_controller/follow_joint_trajectory')
        stop.append('arm_with_torso_controller/follow_joint_trajectory')
        stop.append('torso_controller/follow_joint_trajectory')
        resp = service(start, stop)
    except rospy.ServiceException as e:
        print('Failed to stop controllers')
