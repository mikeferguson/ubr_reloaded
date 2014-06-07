#!/usr/bin/env python

"""
Tucks the robot arm
"""

import rospy

# The arm controlled via ROS 'action', thus we need actionlib
import actionlib

# Messages/Actions needed to control the arm
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# The names of the joints, and the "tucked" position of the joints
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 
               'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
tucked  = [-1.3901, 1.3439, -2.8327, -1.8119, 0.0, -1.6571, 0.0]

if __name__=='__main__':
    rospy.init_node('tuck_my_arm')

    # The arm is controlled by a joint trajectory action
    rospy.loginfo('Waiting for arm_controller...')
    client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo('...connected.')

    # Create a trajectory, with a single end point of the arm being tucked
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = tucked
    trajectory.points[0].velocities = [0.0 for i in joint_names]
    trajectory.points[0].accelerations = [0.0 for i in joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(3.0)

    # Put this trajectory in an action goal
    rospy.loginfo('Tucking arm...')
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(5.0))
    rospy.loginfo('...done')
    
