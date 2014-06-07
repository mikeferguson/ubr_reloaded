#!/usr/bin/env python

"""
Prepares the robot in simulation by tucking the arm
and setting a goal for the head controller.
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 
              'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
tucked  = [-1.3901, 1.3439, -2.8327, -1.8119, 0.0, -1.6571, 0.0]

if __name__=="__main__":
    rospy.init_node("reset_robot")

    rospy.loginfo('Waiting for arm_controller...')
    arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo('...connected.')

    rospy.loginfo("Waiting for head_controller...")
    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    rospy.loginfo("...connected.")

    # Point Head Forward
    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0, 0.0] # look forward and level
    trajectory.points[0].velocities = [0.0 for i in trajectory.joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(1.0)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)
    head_client.send_goal(goal)

    trajectory = JointTrajectory()
    trajectory.joint_names = arm_joints
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = tucked
    trajectory.points[0].velocities = [0.0 for i in trajectory.joint_names]
    trajectory.points[0].accelerations = [0.0 for i in trajectory.joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(3.0)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)
    arm_client.send_goal(goal)

    head_client.wait_for_result(rospy.Duration(5.0))
    arm_client.wait_for_result(rospy.Duration(5.0))
    rospy.loginfo('...done')

