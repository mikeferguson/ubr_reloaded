/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2014, Unbounded Robotics Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael Ferguson */

#ifndef UBR_TELEOP_INCLUDE_UBR_TELEOP_ROBOT_CONTROLLER_H_
#define UBR_TELEOP_INCLUDE_UBR_TELEOP_ROBOT_CONTROLLER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <topic_tools/MuxSelect.h>

class RobotController
{
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;

public:
  RobotController() : active_(false)
  {
    rvx_ = rvw_ = vx_ = vw_ = 0.0;
    rpan_ = rtilt_ = pan_ = tilt_ = 0.0;
    rtorso_ = torso_ = 0.0;
    last_command_ = last_publish_ = last_arm_command_ = 0.0;
    arm_active_ = false;
    gripper_open_ = gripper_close_ = false;
    arm_lx_ = arm_ly_ = arm_lz_ = 0.0;
    arm_ax_ = arm_ay_ = arm_az_ = 0.0;
  }

  /**
   *  \brief Initialize things.
   *  \param n The node handle to use for finding parameters. Should be properly namespaced.
   */
  void init(ros::NodeHandle n)
  {
    // Get a node handle in the global namespace for publishers and action clients
    ros::NodeHandle global;

    n.param("base_max_vel_x", base_max_vel_x_, 0.75);
    n.param("base_max_vel_w", base_max_vel_w_, 2.00);
    n.param("base_max_acc_x", base_max_acc_x_, 0.75);
    n.param("base_max_acc_w", base_max_acc_w_, 3.0);

    n.param("head_pan_min_pos", head_pan_min_pos_, -1.57079633);
    n.param("head_pan_max_pos", head_pan_max_pos_, 1.57079633);
    n.param("head_pan_max_vel", head_pan_max_vel_, 0.5);

    n.param("head_tilt_min_pos", head_tilt_min_pos_, -0.785398163);
    n.param("head_tilt_max_pos", head_tilt_max_pos_, 1.57079633);
    n.param("head_tilt_max_vel", head_tilt_max_vel_, 0.5);

    n.param("torso_min_pos", torso_min_pos_, 0.0);
    n.param("torso_max_pos", torso_max_pos_, 0.35);
    n.param("torso_max_vel", torso_max_vel_, 0.05);

    n.param("gripper_open_pos", gripper_open_pos_, 0.09);
    n.param("gripper_close_pos", gripper_close_pos_, 0.0);
    n.param("gripper_max_effort", gripper_max_effort_, 28.0);

    // Subscribe to joint_states to update positions when inactive
    state_sub_ = global.subscribe("joint_states", 10, &RobotController::stateCb, this);

    // Base and base mux
    cmd_vel_pub_ = global.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 1);
    n.param("use_mux", use_mux_, true);
    if(use_mux_)
    {
      mux_client_ = global.serviceClient<topic_tools::MuxSelect>("cmd_vel_mux/select");
    }

    // Head action setup
    head_client_.reset( new TrajClient("head_controller/follow_joint_trajectory", true) );
    if (!head_client_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("head_controller/follow_joint_trajectory may not be connected. Head may not move.");
    }

    // Torso action setup
    torso_client_.reset( new TrajClient("torso_controller/follow_joint_trajectory", true) );
    if (!torso_client_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("torso_controller/follow_joint_trajectory may not be connected. Torso may not move.");
    }

    // Gripper action setup
    gripper_client_.reset( new GripperClient("gripper_controller/gripper_action", true) );
    if (!gripper_client_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("gripper_controller/gripper_action may not be connected. Gripper may not move.");
    }

    // Publisher for arm movements
    arm_twist_pub_ = global.advertise<geometry_msgs::Twist>("arm_controller/cartesian_twist/command", 1);

    // Initialized
    last_publish_ = ros::Time::now().toSec();
  }

  /**
   *  \brief Gain control of robot base.
   */
  void start()
  {
    if (active_)
      return;

    // Set mux
    if(use_mux_)
    {
      topic_tools::MuxSelect mux_req;
      mux_req.request.topic = cmd_vel_pub_.getTopic();
      if(mux_client_.call(mux_req))
      {
        prev_topic_ = mux_req.response.prev_topic;
      }
      else
      {
        ROS_ERROR("Failed to switch mux");
      }
    }

    // Go active regardless of mux state
    active_ = true;
  }

  /**
   *  \brief Release control of the robot, called when deadman is released.
   */
  void stop()
  {
    if (!active_)
      return;

    // Stop the base
    rvx_ = rvw_ = vx_ = vw_ = 0.0;

    // Have to publish an empty Twist message
    geometry_msgs::Twist cmd_msg;
    cmd_vel_pub_.publish(cmd_msg);

    // If arm control is currently enabled, send message to have it stop 
    if (arm_active_)
    {
      arm_twist_pub_.publish(cmd_msg); //re-use zeroed twist message
      arm_active_ = false;
    }

    // Set mux
    if(use_mux_)
    {
      topic_tools::MuxSelect mux_req;
      mux_req.request.topic = prev_topic_;
      if(!mux_client_.call(mux_req)){
        ROS_ERROR("Failed to to switch mux");
        return;
      }
    }

    // Only go inactive when mux succeeded
    active_ = false;
  }

  /**
   *  \brief Send the latest commands. This should be called a steady periodic rate as internal
   *         calculations assume constant time between calls.
   */
  void sendCommands()
  {
    // Update time
    double now = ros::Time::now().toSec();
    double elapsed = now - last_publish_;

    if (now - last_command_ > 0.5)
    {
      rvx_ = rvw_ = 0.0;
    }

    last_publish_ = now;

    if (active_)
    {
      // Publish cmd_vel (limit acceleration)
      vx_ = integrate(rvx_, vx_, base_max_acc_x_, elapsed);
      vw_ = integrate(rvw_, vw_, base_max_acc_w_, elapsed);
      geometry_msgs::Twist cmd_msg;
      cmd_msg.linear.x = vx_;
      cmd_msg.angular.z = vw_;
      cmd_vel_pub_.publish(cmd_msg);

      // Head action
      double pan_vel = integrate(rpan_-pan_, 0, head_pan_max_vel_, 1.0);
      double tilt_vel = integrate(rtilt_-tilt_, 0, head_tilt_max_vel_, 1.0);
      pan_ = integrate(rpan_, pan_, fabs(pan_vel), 4*elapsed);
      tilt_ = integrate(rtilt_, tilt_, fabs(tilt_vel), 4*elapsed);

      if (fabs(pan_vel) > 0.01 || fabs(tilt_vel) > 0.01)
      {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("head_pan_joint");
        goal.trajectory.joint_names.push_back("head_tilt_joint");
        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.push_back(pan_);
        p.positions.push_back(tilt_);
        p.velocities.push_back(pan_vel);
        p.velocities.push_back(tilt_vel);
        p.time_from_start = ros::Duration(4*elapsed);
        goal.trajectory.points.push_back(p);
        goal.goal_time_tolerance = ros::Duration(0.0);
        head_client_->sendGoal(goal);
      }

      // Torso action
      double torso_vel = integrate(rtorso_-torso_, 0, torso_max_vel_, 1.0);
      torso_ = integrate(rtorso_, torso_, fabs(torso_vel), 4*elapsed);
      if (fabs(torso_vel) > 0.01)
      {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("torso_lift_joint");
        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.push_back(torso_);
        p.velocities.push_back(torso_vel);
        p.time_from_start = ros::Duration(4*elapsed);
        goal.trajectory.points.push_back(p);
        goal.goal_time_tolerance = ros::Duration(0.0);
        torso_client_->sendGoal(goal);
      }

      // Gripper action
      if (gripper_open_)
      {
        control_msgs::GripperCommandGoal goal;
        goal.command.position = gripper_open_pos_;
        goal.command.max_effort = gripper_max_effort_;
        gripper_client_->sendGoal(goal);
        gripper_open_ = false;
      }
      else if(gripper_close_)
      {
        control_msgs::GripperCommandGoal goal;
        goal.command.position = gripper_close_pos_;
        goal.command.max_effort = gripper_max_effort_;
        gripper_client_->sendGoal(goal);
        gripper_close_ = false;
      }

      // Arm twist
      if (arm_active_)
      {
        geometry_msgs::Twist arm_cmd_msg;       
        if (now - last_arm_command_ > 0.5)
        {
          // disable arm and send final zero command
          arm_active_ = false;
        }
        else
        {
          arm_cmd_msg.linear.x = arm_lx_;
          arm_cmd_msg.linear.y = arm_ly_;
          arm_cmd_msg.linear.z = arm_lz_;
          arm_cmd_msg.angular.x = arm_ax_;
          arm_cmd_msg.angular.y = arm_ay_;
          arm_cmd_msg.angular.z = arm_az_;
        }
        arm_twist_pub_.publish(arm_cmd_msg);
      }
    }
  }

  /**
   *  \brief Set the desired velocity of the robot base.
   *  \param vx Forward velocity, in m/s.
   *  \param vw Rotational velocity, in m/s.
   */
  void setBaseVelocity(double vx, double vw)
  {
    // Limit top speed
    rvx_ = fmax(fmin(vx, base_max_vel_x_), -base_max_vel_x_);
    rvw_ = fmax(fmin(vw, base_max_vel_w_), -base_max_vel_w_);
    last_command_ = ros::Time::now().toSec();
  }

  double getBaseVelX() { return vx_; }
  double getBaseVelW() { return vw_; }

  /**
   *  \brief Set the desired position of the head.
   *  \param pan The pan position, in radians.
   *  \param tilt The tilt position, in radians.
   */
  void setHeadPosition(double pan, double tilt)
  {
    // Limit position
    rpan_ = fmax(fmin(pan, head_pan_max_pos_), head_pan_min_pos_);
    rtilt_ = fmax(fmin(tilt, head_tilt_max_pos_), head_tilt_min_pos_);
    last_command_ = ros::Time::now().toSec();
  }

  double getHeadPan() { return pan_; }
  double getHeadTilt() { return tilt_; }

  /** \brief Set the desired torso position, in meters. */
  void setTorsoPosition(double torso)
  {
    // Limit position
    rtorso_ = fmax(fmin(torso, torso_max_pos_), torso_min_pos_);
    last_command_ = ros::Time::now().toSec();
  }

  double getTorsoPosition() { return torso_; }

  void openGripper()
  {
    gripper_open_ = true;
  }

  void closeGripper()
  {
    gripper_close_ = true;
  }


  /** \brief Send twist command to arm. */
  void setArmTwist(double linear_x, double linear_y, double linear_z,
                   double angular_x, double angular_y, double angular_z)
  {
    arm_active_ = true;
    arm_lx_ = linear_x;
    arm_ly_ = linear_y;
    arm_lz_ = linear_z;
    arm_ax_ = angular_x;
    arm_ay_ = angular_y;
    arm_az_ = angular_z;
    last_arm_command_ = ros::Time::now().toSec();
  }


  /** \brief Disable arm, if arm was being controlled, send command to stop moving arm */
  void disableArm()
  {
    // If arm control is currently enabled, send message to have it stop  
    if (arm_active_)
    {
      arm_active_ = false;
      geometry_msgs::Twist cmd_msg;
      arm_twist_pub_.publish(cmd_msg);
    }
  }

private:

  /**
   *  \brief Integrate a new value based on a desired value, the actual value,
   *         max rate of change and time elapsed since last update.
   */
  double integrate(double desired, double actual, double rate, double elapsed)
  {
    if (desired > actual)
      return fmin(desired, actual + rate*elapsed);
    else
      return fmax(desired, actual - rate*elapsed);
  }

  /**
   *  \brief Callback for joint_states, updates current requested position
   *         to actual position when we are not commanded.
   */
  void stateCb(const sensor_msgs::JointState::ConstPtr& msg)
  {
    if (!active_)
    {
      // command has timed out, update the requested positions to current positions
      for (size_t j = 0; j < msg->name.size(); ++j)
      {
        if (msg->name[j] == "torso_lift_joint")
        {
          rtorso_ = torso_ = msg->position[j];
        }
        else if (msg->name[j] == "head_pan_joint")
        {
          rpan_ = pan_ = msg->position[j];
        }
        else if (msg->name[j] == "head_tilt_joint")
        {
          rtilt_ = tilt_ = msg->position[j];
        }
      }
    }
  }

  // Requested Position
  double rvx_, rvw_;
  double rpan_, rtilt_;
  double rtorso_;

  // Values being Published
  double vx_, vw_;
  double pan_, tilt_;
  double torso_;

  // Arm related values
  double arm_lx_, arm_ly_, arm_lz_;
  double arm_ax_, arm_ay_, arm_az_;
  double arm_active_;
  double last_arm_command_;

  // Base Limts
  double base_max_vel_x_;
  double base_max_vel_w_;
  double base_max_acc_x_;
  double base_max_acc_w_;

  // Head Limits
  double head_pan_min_pos_, head_pan_max_pos_;
  double head_pan_max_vel_;
  double head_tilt_min_pos_, head_tilt_max_pos_;
  double head_tilt_max_vel_;

  // Torso Limits
  double torso_min_pos_, torso_max_pos_;
  double torso_max_vel_;

  // Gripper Limits
  double gripper_open_pos_, gripper_close_pos_, gripper_max_effort_;
  bool gripper_open_, gripper_close_;

  // ROS Stuff
  ros::Subscriber state_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher arm_twist_pub_;
  boost::shared_ptr< TrajClient > head_client_;
  boost::shared_ptr< TrajClient > torso_client_;
  boost::shared_ptr< GripperClient> gripper_client_;
  ros::ServiceClient mux_client_;

  // Are we doing a mux with navigation?
  bool use_mux_;
  std::string prev_topic_;

  bool active_;
  double last_publish_, last_command_;
};

#endif  // UBR_TELEOP_INCLUDE_UBR_TELEOP_ROBOT_CONTROLLER_H_
