/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-2024, Michael Ferguson
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

#ifndef UBR1_GAZEBO_JOINT_HANDLE_H_
#define UBR1_GAZEBO_JOINT_HANDLE_H_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>

#include <ignition/physics/Joint.hh>
#include <robot_controllers_interface/joint_handle.h>
#include <control_toolbox/pid.hpp>

namespace ubr1_gazebo
{

enum CommandState
{
  MODE_DISABLED,
  MODE_CONTROL_EFFORT,
  MODE_CONTROL_VELOCITY,
  MODE_CONTROL_POSITION
};

class GazeboJointHandle : public robot_controllers_interface::JointHandle
{
public:
  GazeboJointHandle(/*ignition::physics::JointPtr joint_ptr*/) :
    joint_(joint_ptr),
    mode_(MODE_DISABLED)
  {
    ros::NodeHandle nh("~");

    // Load controller parameters
    //position_pid_.init(ros::NodeHandle(nh, getName() + "/position"));
    //velocity_pid_.init(ros::NodeHandle(nh, getName() + "/velocity"));

    // Load optional effort_limit as a workaround to gzsdf limitation
    nh.param(getName() + "/effort_limit", effort_limit_, -1.0);

    // Extra force input (for torso gas spring)
    nh.param(getName() + "/effort_offset", effort_offset_, 0.0);

    // Should we put out debug info?
    nh.param(getName() + "/debug", debug_, false);

    if (debug_)
      ROS_INFO_STREAM(getName() << " has limit of " << getEffortMax() << " and offset of " << effort_offset_);
  }
  virtual ~GazeboJointHandle()
  {
  }

  /**
   *  \brief Used by controllers to set the desired position command of a joint.
   *  \param position The desired position, in radians or meters.
   *  \param velocity The desired velocity, in radians/sec or meters/sec.
   *  \param effort The desired effort, in Nm or N.
   */
  virtual void setPosition(double position,
                           double velocity,
                           double effort)
  {
    // ControllerManager resets these each cycle, so accumulate
    desired_position_ += position;
    desired_velocity_ += velocity;
    desired_effort_ += effort;
    mode_ = MODE_CONTROL_POSITION;
  }

  /**
   *  \brief Used by controllers to set the desired velocity command of a joint.
   *  \param velocity The desired velocity, in radians/sec or meters/sec.
   *  \param effort The desired effort, in Nm or N.
   */
  virtual void setVelocity(double velocity,
                           double effort)
  {
    // ControllerManager resets these each cycle, so accumulate
    desired_velocity_ += velocity;
    desired_effort_ += effort;

    if (!isPositionControlled())
    {
      mode_ = MODE_CONTROL_VELOCITY;
    }
  }

  /**
   *  \brief Used by controllers to set the desired effort of a joint.
   *  \param effort The desired effort, in Nm or N.
   */
  virtual void setEffort(double effort)
  {
    // ControllerManager resets these each cycle, so accumulate
    desired_effort_ += effort;
    if (!isPositionControlled() && !isVelocityControlled())
    {
      mode_ = MODE_CONTROL_EFFORT;
    }
  }

  /** \brief Returns the position of the joint. */
  virtual double getPosition()
  {
    return joint_->Position(0);
  }

  /** \brief Returns the velocity of the joint. */
  virtual double getVelocity()
  {
    return joint_->GetVelocity(0);
  }

  /** \brief Returns the effort applied to the joint. */
  virtual double getEffort()
  {
    return applied_effort_;
  }

  /** \brief Is this joint continuous */
  virtual bool isContinuous()
  {
    // TODO
    return false;
  }

  /** \brief Get the lower positional limit */
  virtual double getPositionMin()
  {
    return joint_->LowerLimit(0);
  }

  /** \brief Get the upper positional limit */
  virtual double getPositionMax()
  {
    return joint_->UpperLimit(0);
  }

  /** \brief Get the velocity limit */
  virtual double getVelocityMax()
  {
    return joint_->GetVelocityLimit(0);
  }

  /** \brief Get the effort limit */
  virtual double getEffortMax()
  {
    /*
     * gzsdf has a major flaw when using continuous joints. It appears gazebo
     * cannot handle continuous joints and so it sets the limits to +/-1e16.
     * This is fine, except it drops the limit effort and limit velocity.
     * Lack of limit effort causes the robot to implode to the origin if the
     * controllers are not tuned or experience a disturbance. This little hack
     * lets us limit the controller effort internally.
     */
    if (effort_limit_ < 0.0)
      return joint_->GetEffortLimit(0);
    else
      return effort_limit_;
  }

  virtual std::string getName()
  {
    // TODO pointer check on joint_ ???
    return joint_->GetName();
  }

  void reset()
  {
    desired_position_ = 0.0;
    desired_velocity_ = 0.0;
    desired_effort_ = 0.0;
    mode_ = MODE_DISABLED;
  }

  bool isPositionControlled()
  {
    return mode_ == MODE_CONTROL_POSITION;
  }

  bool isVelocityControlled()
  {
    return mode_ == MODE_CONTROL_VELOCITY;
  }

  bool isEffortControlled()
  {
    return mode_ == MODE_CONTROL_EFFORT;
  }

  void update(const rclcpp::Time& now, const rclcpp::Duration& dt)
  {
    float effort = 0.0;
    if (isEffortControlled())
    {
      effort = desired_effort_;
    }
    else if (isPositionControlled())
    {
      float p_error = angles::shortest_angular_distance(getPosition(), desired_position_);
      float t = position_pid_.computeCommand(p_error, dt) +
                velocity_pid_.computeCommand(desired_velocity_ - getVelocity(), dt);
      effort = t + desired_effort_;
    }
    else if (isVelocityControlled())
    {
      float t = velocity_pid_.computeCommand(desired_velocity_ - getVelocity(), dt);
      effort = t + desired_effort_;
    }

    // Limit effort so robot doesn't implode
    float lim = getEffortMax();
    applied_effort_ = std::max(-lim, std::min(effort, lim));

    if (debug_)
      RCLCPP_INFO_STREAM(rclcpp::get_logger(getName()),
                         getName() << " commanded effort of " << effort);

    // Actually update
    joint_->SetForce(0, applied_effort_ + effort_offset_);
  }

  /** \brief Used only by the gripper */
  double setMaxEffort(double effort)
  {
    effort_limit_ = effort;
    return getEffortMax();
  }

private:
  ignition::physics::JointPtr joint_;

  float desired_position_;
  float desired_velocity_;
  float desired_effort_;
  
  /// control mode
  int mode_;

  control_toolbox::Pid position_pid_;
  control_toolbox::Pid velocity_pid_;

  /// Hack for continuous joints that fail to have effort limits
  double effort_limit_;

  /// Hack for joints with gas springs attached
  double effort_offset_;

  /// GetForce(0u) is not always right
  double applied_effort_;

  /// By-joint debug capability
  bool debug_;

  // You no copy...
  GazeboJointHandle(const GazeboJointHandle&);
  GazeboJointHandle& operator=(const GazeboJointHandle&);
};

typedef std::shared_ptr<GazeboJointHandle> GazeboJointHandlePtr;

}  // namespace ubr1_gazebo

#endif  // UBR1_GAZEBO_JOINT_HANDLE_H
