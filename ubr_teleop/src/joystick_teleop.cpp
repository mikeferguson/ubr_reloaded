/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Michael Ferguson
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ubr_teleop/robot_controller.h>

class JoyTeleop : public RobotController
{
public:
  JoyTeleop(const std::string& name) :
      RobotController(name),
      deadman_(false),
      deadman_head_(false),
      deadman_arm_linear_(false),
      deadman_arm_angular_(false)
  {
    head_pan_step_ = declare_parameter<double>("head_pan_step", 0.35);
    head_tilt_step_ = declare_parameter<double>("head_tilt_step", -0.35);
    torso_step_ = declare_parameter<double>("torso_step", 0.0125);

    deadman_button_ = declare_parameter<int>("deadman_button", 4);
    deadman_head_button_ = declare_parameter<int>("deadman_head_button", 6);

    torso_up_button_ = declare_parameter<int>("torso_up_button", 2);
    torso_down_button_ = declare_parameter<int>("torso_down_button", 0);

    gripper_open_button_ = declare_parameter<int>("gripper_open_button", 3);
    gripper_close_button_ = declare_parameter<int>("gripper_close_button", 1);

    deadman_arm_linear_button_ = declare_parameter<int>("deadman_arm_linear_button", 5);
    deadman_arm_angular_button_ = declare_parameter<int>("deadman_arm_angular_button", 7);
    arm_lx_axis_ = declare_parameter<int>("arm_lx_axis", 4);
    arm_ly_axis_ = declare_parameter<int>("arm_ly_axis", 3);
    arm_lz_axis_ = declare_parameter<int>("arm_lz_axis", 1);
    arm_ax_axis_ = declare_parameter<int>("arm_ax_axis", 4);
    arm_ay_axis_ = declare_parameter<int>("arm_ay_axis", 3);
    arm_az_axis_ = declare_parameter<int>("arm_az_axis", 1);

    arm_linear_scale_ = declare_parameter<double>("arm_linear_scale", 0.5);
    arm_angular_scale_ = declare_parameter<double>("arm_angular_scale", 0.5);

    base_vx_axis_ = declare_parameter<int>("axis_vx", 4);
    base_vw_axis_ = declare_parameter<int>("axis_vw", 0);

    head_pan_axis_ = declare_parameter<int>("axis_pan", 0);
    head_tilt_axis_ = declare_parameter<int>("axis_tilt", 4);

    sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10,
             std::bind(&JoyTeleop::joyCb, this, _1));

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
               std::bind(&JoyTeleop::update, this));
  }

  /**
   *  \brief Callback for the joystick topic.
   */
  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_recv_ = now();

    deadman_ = msg->buttons[deadman_button_];
    deadman_head_ = msg->buttons[deadman_head_button_];
    deadman_arm_angular_ = msg->buttons[deadman_arm_angular_button_];
    deadman_arm_linear_ = msg->buttons[deadman_arm_linear_button_];

    if (!deadman_)
    {
      stop();
      return;
    }

    start();

    if (deadman_arm_angular_ || deadman_arm_linear_)
    {
      // Stop moving base
      setBaseVelocity(0.0, 0.0);

      // Stop head
      pan_command_ = 0.0;
      tilt_command_ = 0.0;

      // Stop moving torso
      torso_command_ = 0.0;

      double lx, ly, lz, ax, ay, az;
      if (deadman_arm_linear_)
      {
        // Linear takes precedence over angular
        lx = msg->axes[arm_lx_axis_]*arm_linear_scale_;
        ly = msg->axes[arm_ly_axis_]*arm_linear_scale_;
        lz = msg->axes[arm_lz_axis_]*arm_linear_scale_;
        ax = 0.0;
        ay = 0.0;
        az = 0.0;
      }
      else
      {
        lx = 0.0;
        ly = 0.0;
        lz = 0.0;
        ax = -msg->axes[arm_ax_axis_]*arm_angular_scale_;
        ay = msg->axes[arm_ay_axis_]*arm_angular_scale_;
        az = msg->axes[arm_az_axis_]*arm_angular_scale_;
      }
      setArmTwist(lx, ly, lz, ax, ay, az);
    }
    else if (deadman_head_)
    {
      // Stop moving base
      setBaseVelocity(0.0, 0.0);

      // Get head
      pan_command_ = msg->axes[head_pan_axis_]*head_pan_step_;
      tilt_command_ = msg->axes[head_tilt_axis_]*head_tilt_step_;

      // Stop moving torso
      torso_command_ = 0.0;

      // Stop moving arm
      disableArm();
    }
    else
    {
      // Get base velocities
      setBaseVelocity(msg->axes[base_vx_axis_] * base_max_vel_x_,
                      msg->axes[base_vw_axis_] * base_max_vel_w_);

      // Stop moving head
      pan_command_ = 0.0;
      tilt_command_ = 0.0;

      // Get torso
      if (msg->buttons[torso_down_button_])
        torso_command_ = -torso_step_;
      else if (msg->buttons[torso_up_button_])
        torso_command_ = torso_step_;
      else
        torso_command_ = 0.0;

      // Get Gripper
      if (msg->buttons[gripper_open_button_])
        openGripper();
      else if (msg->buttons[gripper_close_button_])
        closeGripper();

      // Stop moving arm
      disableArm();
    }
  }

  void update()
  {
    setHeadPosition(getHeadPan() + pan_command_,
                    getHeadTilt() + tilt_command_);
    setTorsoPosition(getTorsoPosition() + torso_command_);
    sendCommands();
  }

private:
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

  // Command to periodically apply
  double pan_command_, tilt_command_, torso_command_;

  rclcpp::Time last_recv_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<JoyTeleop> teleop(new JoyTeleop("teleop"));
  rclcpp::spin(teleop);
}
