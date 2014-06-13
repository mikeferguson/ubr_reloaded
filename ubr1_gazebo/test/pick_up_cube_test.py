#!/usr/bin/env python

import rospy
import rosbag
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

class PickUpCubeTest:

    def __init__(self):
        rospy.loginfo("Waiting for arm_controller...")
        self.arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected")

    def run(self):
        bag = rosbag.Bag("pick_up_cube_trajectories.bag")
        msg_count = 0
        for topic, msg, t in bag.read_messages():
            # send each trajectory
            self.arm_client.send_goal(msg.goal)
            self.arm_client.wait_for_result(rospy.Duration(15.0))
            msg_count += 1

            # maybe close the gripper?
            if msg_count == 2:
                goal = GripperCommandGoal()
                goal.command.max_effort = 28.0
                goal.command.position = 0.0
                self.gripper_client.send_goal(goal)
                self.gripper_client.wait_for_result(rospy.Duration(15.0))

if __name__ == "__main__":
    rospy.init_node("pick_up_cube_test")
    test = PickUpCubeTest()
    test.run()

