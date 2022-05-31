/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-2022, Michael Ferguson
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

// Author: Michael Ferguson

#ifndef UBR_TELEOP_INCLUDE_UBR_TELEOP_ROBOT_CONTROLLER_H
#define UBR_TELEOP_INCLUDE_UBR_TELEOP_ROBOT_CONTROLLER_H

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>

using std::placeholders::_1;

class RobotController : public rclcpp::Node
{
  using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;
  using FollowJointTrajectoryClient = rclcpp_action::Client<FollowJointTrajectoryAction>;

  using GripperCommandAction = control_msgs::action::GripperCommand;
  using GripperCommandClient = rclcpp_action::Client<GripperCommandAction>;

public:
  RobotController(const std::string& name) :
    rclcpp::Node(name),
    active_(false)
  {
    rvx_ = rvw_ = vx_ = vw_ = 0.0;
    rpan_ = rtilt_ = pan_ = tilt_ = 0.0;
    rtorso_ = torso_ = 0.0;
    arm_active_ = false;
    gripper_open_ = gripper_close_ = false;
    arm_lx_ = arm_ly_ = arm_lz_ = 0.0;
    arm_ax_ = arm_ay_ = arm_az_ = 0.0;

    base_max_vel_x_ = declare_parameter<double>("base_max_vel_x", 0.75);
    base_max_vel_w_ = declare_parameter<double>("base_max_vel_w", 2.00);
    base_max_acc_x_ = declare_parameter<double>("base_max_acc_x", 0.75);
    base_max_acc_w_ = declare_parameter<double>("base_max_acc_w", 3.00);

    head_pan_min_pos_ = declare_parameter<double>("head_pan_min_pos", -1.57079633);
    head_pan_max_pos_ = declare_parameter<double>("head_pan_max_pos", 1.57079633);
    head_pan_max_vel_ = declare_parameter<double>("head_pan_max_vel", 0.5);

    head_tilt_min_pos_ = declare_parameter<double>("head_tilt_min_pos", -0.785398163);
    head_tilt_max_pos_ = declare_parameter<double>("head_tilt_max_pos", 1.57079633);
    head_tilt_max_vel_ = declare_parameter<double>("head_tilt_max_vel", 0.5);

    torso_min_pos_ = declare_parameter<double>("torso_min_pos", 0.0);
    torso_max_pos_ = declare_parameter<double>("torso_max_pos", 0.35);
    torso_max_vel_ = declare_parameter<double>("torso_max_vel", 0.05);

    gripper_open_pos_ = declare_parameter<double>("gripper_open_pos", 0.09);
    gripper_close_pos_ = declare_parameter<double>("gripper_close_pos", 0.0);
    gripper_max_effort_ = declare_parameter<double>("gripper_max_effort", 28.0);

    // Subscribe to joint_states to update positions when inactive
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 1,
                     std::bind(&RobotController::stateCb, this, _1));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_in", 1,
                     std::bind(&RobotController::cmdvelCb, this, _1));

    // Base and base mux
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 1);

    // Head action setup
    head_client_ = rclcpp_action::create_client<FollowJointTrajectoryAction>(
      this,
      "head_controller/follow_joint_trajectory"
    );

    // Torso action setup
    torso_client_ = rclcpp_action::create_client<FollowJointTrajectoryAction>(
      this,
      "torso_controller/follow_joint_trajectory"
    );

    // Gripper action setup
    gripper_client_ = rclcpp_action::create_client<GripperCommandAction>(
      this,
      "gripper_controller/command"
    );

    // Publisher for arm movements
    arm_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("arm_controller_command", 1);

    // Initialized
    last_publish_ = now();
    last_command_ = now();
    last_arm_command_ = now();
    last_base_command_ = now();
  }

  /**
   *  \brief Gain control of robot base.
   */
  void start()
  {
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
    geometry_msgs::msg::Twist cmd_msg;
    cmd_vel_pub_->publish(cmd_msg);

    // If arm control is currently enabled, send message to have it stop 
    if (arm_active_)
    {
      geometry_msgs::msg::TwistStamped arm_cmd_msg;
      arm_twist_pub_->publish(arm_cmd_msg);
      arm_active_ = false;
    }

    active_ = false;
  }

  /**
   *  \brief Send the latest commands. This should be called a steady periodic rate as internal
   *         calculations assume constant time between calls.
   */
  void sendCommands()
  {
    // Update time
    rclcpp::Time now = this->now();
    auto elapsed_ns = (now - last_publish_).nanoseconds();
    double elapsed = double (elapsed_ns) / 1e9;

    if (now - last_command_ > rclcpp::Duration(0, 5e8))
    {
      rvx_ = rvw_ = 0.0;
    }

    last_publish_ = now;

    if (active_)
    {
      // Publish cmd_vel (limit acceleration)
      vx_ = integrate(rvx_, vx_, base_max_acc_x_, elapsed);
      vw_ = integrate(rvw_, vw_, base_max_acc_w_, elapsed);
      geometry_msgs::msg::Twist cmd_msg;
      cmd_msg.linear.x = vx_;
      cmd_msg.angular.z = vw_;
      cmd_vel_pub_->publish(cmd_msg);

      // Head action
      double pan_vel = integrate(rpan_-pan_, 0, head_pan_max_vel_, 1.0);
      double tilt_vel = integrate(rtilt_-tilt_, 0, head_tilt_max_vel_, 1.0);
      pan_ = integrate(rpan_, pan_, std::fabs(pan_vel), 4*elapsed);
      tilt_ = integrate(rtilt_, tilt_, std::fabs(tilt_vel), 4*elapsed);

      if (std::fabs(pan_vel) > 0.01 || std::fabs(tilt_vel) > 0.01)
      {
        FollowJointTrajectoryAction::Goal goal;
        goal.trajectory.joint_names.push_back("head_pan_joint");
        goal.trajectory.joint_names.push_back("head_tilt_joint");
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions.push_back(pan_);
        p.positions.push_back(tilt_);
        p.velocities.push_back(pan_vel);
        p.velocities.push_back(tilt_vel);
        p.time_from_start = rclcpp::Duration(0, 4 * elapsed_ns);
        goal.trajectory.points.push_back(p);
        goal.goal_time_tolerance = rclcpp::Duration(0, 0);
        head_client_->async_send_goal(goal);
      }

      // Torso action
      double torso_vel = integrate(rtorso_-torso_, 0, torso_max_vel_, 1.0);
      torso_ = integrate(rtorso_, torso_, std::fabs(torso_vel), 4*elapsed);
      if (std::fabs(torso_vel) > 0.01)
      {
        FollowJointTrajectoryAction::Goal goal;
        goal.trajectory.joint_names.push_back("torso_lift_joint");
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions.push_back(torso_);
        p.velocities.push_back(torso_vel);
        p.time_from_start = rclcpp::Duration(0, 4 * elapsed_ns);
        goal.trajectory.points.push_back(p);
        goal.goal_time_tolerance = rclcpp::Duration(0, 0);
        torso_client_->async_send_goal(goal);
      }

      // Gripper action
      if (gripper_open_)
      {
        GripperCommandAction::Goal goal;
        goal.command.position = gripper_open_pos_;
        goal.command.max_effort = gripper_max_effort_;
        gripper_client_->async_send_goal(goal);
        gripper_open_ = false;
      }
      else if (gripper_close_)
      {
        GripperCommandAction::Goal goal;
        goal.command.position = gripper_close_pos_;
        goal.command.max_effort = gripper_max_effort_;
        gripper_client_->async_send_goal(goal);
        gripper_close_ = false;
      }

      // Arm twist
      if (arm_active_)
      {
        geometry_msgs::msg::TwistStamped arm_cmd_msg;
        if (now - last_arm_command_ > rclcpp::Duration(0, 5e8))
        {
          // disable arm and send final zero command
          arm_active_ = false;
        }
        else
        {
          arm_cmd_msg.header.stamp = this->now();
          arm_cmd_msg.header.frame_id = "wrist_roll_link";
          arm_cmd_msg.twist.linear.x = arm_lx_;
          arm_cmd_msg.twist.linear.y = arm_ly_;
          arm_cmd_msg.twist.linear.z = arm_lz_;
          arm_cmd_msg.twist.angular.x = arm_ax_;
          arm_cmd_msg.twist.angular.y = arm_ay_;
          arm_cmd_msg.twist.angular.z = arm_az_;
        }
        arm_twist_pub_->publish(arm_cmd_msg);
      }
    }
    else
    {
      // Forward base commands from subscriber
      if (last_base_msg_ && (this->now() - last_base_command_) < rclcpp::Duration(0, 2e8))
      {
        cmd_vel_pub_->publish(*last_base_msg_);
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
    last_command_ = now();
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
    last_command_ = now();
  }

  double getHeadPan() { return pan_; }
  double getHeadTilt() { return tilt_; }

  /** \brief Set the desired torso position, in meters. */
  void setTorsoPosition(double torso)
  {
    // Limit position
    rtorso_ = fmax(fmin(torso, torso_max_pos_), torso_min_pos_);
    last_command_ = now();
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
    last_arm_command_ = now();
  }


  /** \brief Disable arm, if arm was being controlled, send command to stop moving arm */
  void disableArm()
  {
    // If arm control is currently enabled, send message to have it stop  
    if (arm_active_)
    {
      arm_active_ = false;
      geometry_msgs::msg::TwistStamped cmd_msg;
      arm_twist_pub_->publish(cmd_msg);
    }
  }

protected:

  /**
   *  \brief Integrate a new value based on a desired value, the actual value,
   *         max rate of change and time elapsed since last update.
   */
  double integrate(double desired, double actual, double rate, double elapsed)
  {
    if (desired > actual)
      return fmin(desired, actual + rate * elapsed);
    else
      return fmax(desired, actual - rate * elapsed);
  }

  /**
   *  \brief Callback for joint_states, updates current requested position
   *         to actual position when we are not commanded.
   */
  void stateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
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

  /**
   *  \brief Mux doesn't exist in ROS2 (and might never).
   *         Pass outside commands through teleop to stop them.
   */
  void cmdvelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_base_msg_ = msg;
    last_base_command_ = now();
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
  bool arm_active_;

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
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
  FollowJointTrajectoryClient::SharedPtr head_client_;
  FollowJointTrajectoryClient::SharedPtr torso_client_;
  GripperCommandClient::SharedPtr gripper_client_;

  // Pass through
  std::shared_ptr<geometry_msgs::msg::Twist> last_base_msg_;

  bool active_;
  rclcpp::Time last_publish_, last_command_, last_arm_command_, last_base_command_;
};

#endif  // UBR_TELEOP_INCLUDE_UBR_TELEOP_ROBOT_CONTROLLER_H
