#!/usr/bin/env python

"""
Tucks the robot arm
"""

import rospy

# The head is controlled via ROS 'action', thus we need actionlib
import actionlib

# Messages/Actions needed to control the head
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# The names of the joints, and the "tucked" position of the joints
joint_names = ['head_pan_joint', 'head_tilt_joint']

if __name__=='__main__':
    rospy.init_node('move_my_head')

    # The head is controlled by a joint trajectory action
    rospy.loginfo('Waiting for head_controller...')
    client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo('...connected.')

    # Create a trajectory, with some head movement
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0, 0.0] # look forward and level
    trajectory.points[0].velocities = [0.0 for i in joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(0.0)

    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = [1.57, -0.45] # look left and up
    trajectory.points[1].velocities = [0.0 for i in joint_names]
    trajectory.points[1].time_from_start = rospy.Duration(4.0)

    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[2].positions = [-1.57, 1.57] # look right and down
    trajectory.points[2].velocities = [0.0 for i in joint_names]
    trajectory.points[2].time_from_start = rospy.Duration(8.0)

    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[3].positions = [0.0, 0.0] # look forward and level
    trajectory.points[3].velocities = [0.0 for i in joint_names]
    trajectory.points[3].time_from_start = rospy.Duration(10.0)

    # Put this trajectory in an action goal
    rospy.loginfo('Moving head...')
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(20.0))
    rospy.loginfo('...done')
    
