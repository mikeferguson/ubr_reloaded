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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ubr_teleop/robot_controller.h>

class JoyTeleop
{
public:
  JoyTeleop() : deadman_(false), deadman_head_(false),
                deadman_arm_linear_(false), deadman_arm_angular_(false)
  {
    vx = vw = 0.0;
    pan = tilt = 0.0;
    torso = 0.0;
    arm_lx, arm_ly, arm_lz, arm_ax, arm_ay, arm_az = 0.0;
  }

  void init(ros::NodeHandle n)
  {
    n.param("base_max_vel_x", base_max_vel_x_, 1.0);
    n.param("base_max_vel_w", base_max_vel_w_, 3.0);

    n.param("head_pan_step", head_pan_step_, 0.35);
    n.param("head_tilt_step", head_tilt_step_, -0.35);
    n.param("torso_step", torso_step_, 0.0125);

    n.param("deadman_button", deadman_button_, 10);
    n.param("deadman_head_button", deadman_head_button_, 8);

    n.param("torso_up_button", torso_up_button_, 12);
    n.param("torso_down_button", torso_down_button_, 14);

    n.param("gripper_open", gripper_open_button_, 0);
    n.param("gripper_close", gripper_close_button_, 3);

    n.param("deadman_arm_linear_button", deadman_arm_linear_button_, 11);
    n.param("deadman_arm_angular_button", deadman_arm_angular_button_, 9);
    n.param("arm_lx_axis", arm_lx_axis_, 3);
    n.param("arm_ly_axis", arm_ly_axis_, 2);
    n.param("arm_lz_axis", arm_lz_axis_, 1);
    n.param("arm_ax_axis", arm_ax_axis_, 2);
    n.param("arm_ay_axis", arm_ay_axis_, 3);
    n.param("arm_az_axis", arm_az_axis_, 0);
    n.param("arm_linear_scale", arm_linear_scale_, 0.5);
    n.param("arm_angular_scale", arm_angular_scale_, 0.5);

    n.param("axis_vx", base_vx_axis_, 3);
    n.param("axis_vw", base_vw_axis_, 0);

    n.param("axis_pan", head_pan_axis_, 0);
    n.param("axis_tilt", head_tilt_axis_, 3);

    controller_.init(n);

    ros::NodeHandle nh;
    sub_ = nh.subscribe("joy", 10, &JoyTeleop::joyCb, this);
  }

  /**
   *  \brief Callback for the joystick topic.
   */
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg)
  {
    last_recv_ = ros::Time::now();

    deadman_ = msg->buttons[deadman_button_];
    deadman_head_ = msg->buttons[deadman_head_button_];
    deadman_arm_angular_ = msg->buttons[deadman_arm_angular_button_];
    deadman_arm_linear_ = msg->buttons[deadman_arm_linear_button_];

    if (!deadman_)
    {
      controller_.stop();
      return;
    }

    controller_.start();

    if (deadman_arm_angular_ || deadman_arm_linear_)
    {
      // Stop moving base
      vx = 0.0;
      vw = 0.0;

      // Stop head
      pan = 0.0;
      tilt = 0.0;

      // Stop moving torso
      torso = 0.0;

      if (deadman_arm_linear_)
      {
        // Linear takes precedence over angular
        arm_lx = msg->axes[arm_lx_axis_]*arm_linear_scale_;
        arm_ly = msg->axes[arm_ly_axis_]*arm_linear_scale_;
        arm_lz = msg->axes[arm_lz_axis_]*arm_linear_scale_;
        arm_ax = 0.0;
        arm_ay = 0.0;
        arm_az = 0.0;
      }
      else
      {
        arm_lx = 0.0;
        arm_ly = 0.0;
        arm_lz = 0.0;
        arm_ax = -msg->axes[arm_ax_axis_]*arm_angular_scale_;
        arm_ay = msg->axes[arm_ay_axis_]*arm_angular_scale_;
        arm_az = msg->axes[arm_az_axis_]*arm_angular_scale_;
      }

      controller_.setArmTwist(arm_lx, arm_ly, arm_lz, arm_ax, arm_ay, arm_az);
    }
    else if (deadman_head_)
    {
      // Stop moving base
      vx = 0.0;
      vw = 0.0;

      // Get head
      pan = msg->axes[head_pan_axis_]*head_pan_step_;
      tilt = msg->axes[head_tilt_axis_]*head_tilt_step_;

      // Stop moving torso
      torso = 0.0;

      // Stop moving arm
      controller_.disableArm();
    }
    else
    {
      // Get base velocities
      vx = msg->axes[base_vx_axis_] * base_max_vel_x_;
      vw = msg->axes[base_vw_axis_] * base_max_vel_w_;

      // Limit velocities
      vx = fmax(fmin(vx, base_max_vel_x_), -base_max_vel_x_);
      vw = fmax(fmin(vw, base_max_vel_w_), -base_max_vel_w_);

      // Stop moving head
      pan = 0.0;
      tilt = 0.0;

      // Get torso
      if (msg->buttons[torso_down_button_])
        torso = -torso_step_;
      else if (msg->buttons[torso_up_button_])
        torso = torso_step_;
      else
        torso = 0.0;

      // Get Gripper
      if (msg->buttons[gripper_open_button_])
        controller_.openGripper();
      else if (msg->buttons[gripper_close_button_])
        controller_.closeGripper();

      // Stop moving arm
      controller_.disableArm();
    }
  }

  void update()
  {
    controller_.setBaseVelocity(vx, vw);
    controller_.setHeadPosition(controller_.getHeadPan() + pan,
                                controller_.getHeadTilt() + tilt);
    controller_.setTorsoPosition(controller_.getTorsoPosition() + torso);
    controller_.sendCommands();
  }

private:
  // Base Limits
  double base_max_vel_x_, base_max_vel_w_;

  // Head Limits
  double head_pan_step_;
  double head_tilt_step_;

  // Torso Limits
  double torso_step_;

  // Status
  bool deadman_;
  bool deadman_head_;
  bool deadman_arm_linear_;
  bool deadman_arm_angular_;

  // Command buttons/axis
  int base_vx_axis_, base_vw_axis_;
  int head_pan_axis_, head_tilt_axis_;
  int torso_up_button_, torso_down_button_;
  int gripper_open_button_, gripper_close_button_;

  // This button allows robot to move
  int deadman_button_;
  // This button switches to commanding the head
  int deadman_head_button_;
  // This button switches to commanding the linear cartesian position of the gripper
  int deadman_arm_linear_button_;
  // This button switches to commanding the angular cartesian position of the gripper
  int deadman_arm_angular_button_;

  // Joystick axis used for linear control
  int arm_lx_axis_, arm_ly_axis_, arm_lz_axis_;
  // Joystick axis used for angular control
  int arm_ax_axis_, arm_ay_axis_, arm_az_axis_;

  // Scaling factor for arm linear and angular scale
  double arm_linear_scale_, arm_angular_scale_;

  // Current publishing values
  double vx, vw;
  double pan, tilt;
  double torso;
  double arm_lx, arm_ly, arm_lz, arm_ax, arm_ay, arm_az;

  ros::Time last_recv_;
  ros::Subscriber sub_;

  RobotController controller_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  JoyTeleop teleop;

  ros::NodeHandle n("~");
  teleop.init(n);
  
  ros::Rate r(20.0);
  while (ros::ok())
  {
    ros::spinOnce();
    teleop.update();
    r.sleep();
  }

  return 0;
}
