#!/usr/bin/env python

"""
Tucks the robot arm
"""

import rospy

# The torso is controlled via ROS 'action', thus we need actionlib
import actionlib

# Messages/Actions needed to control the torso
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__=='__main__':
    rospy.init_node('move_my_torso')

    # The arm is controlled by a joint trajectory action
    rospy.loginfo('Waiting for torso_controller...')
    client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo('...connected.')

    # Create a trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = ["torso_lift_joint"]
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = [0.0]
    trajectory.points[0].velocities = [0.0 for i in joint_names]
    trajectory.points[0].accelerations = [0.0 for i in joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(3.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = [0.35]
    trajectory.points[1].velocities = [0.0 for i in joint_names]
    trajectory.points[1].accelerations = [0.0 for i in joint_names]
    trajectory.points[1].time_from_start = rospy.Duration(6.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[2].positions = [0.0]
    trajectory.points[2].velocities = [0.0 for i in joint_names]
    trajectory.points[2].accelerations = [0.0 for i in joint_names]
    trajectory.points[2].time_from_start = rospy.Duration(9.0)

    # Put this trajectory in an action goal
    rospy.loginfo('Moving torso...')
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(5.0))
    rospy.loginfo('...done')

