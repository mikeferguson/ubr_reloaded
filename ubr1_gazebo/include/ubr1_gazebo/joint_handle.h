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

#ifndef UBR1_GAZEBO_JOINT_HANDLE_H_
#define UBR1_GAZEBO_JOINT_HANDLE_H_

#include <ros/ros.h>
#include <angles/angles.h>

#include <gazebo/physics/physics.hh>
#include <ubr_controllers/joint_handle.h>
#include <control_toolbox/pid.h>

namespace ubr1_gazebo
{

enum CommandState
{
  MODE_DISABLED,
  MODE_CONTROL_EFFORT,
  MODE_CONTROL_VELOCITY,
  MODE_CONTROL_POSITION
};

class GazeboJointHandle : public ubr_controllers::JointHandle
{
public:
  GazeboJointHandle(gazebo::physics::JointPtr joint_ptr) :
    joint_(joint_ptr),
    mode_(MODE_DISABLED)
  {
    ros::NodeHandle nh("~");

    // Load controller parameters
    position_pid_.init(ros::NodeHandle(nh, getName() + "/position"));
    velocity_pid_.init(ros::NodeHandle(nh, getName() + "/velocity"));

    // Load optional effort_limit as a workaround to gzsdf limitation
    nh.param(getName() + "/effort_limit", effort_limit_, -1.0);

    // Extra force input (for torso gas spring)
    nh.param(getName() + "/effort_offset", effort_offset_, 0.0);

    // Should we put out debug info?
    nh.param(getName() + "/debug", debug_, false);

    if (debug_)
      ROS_INFO_STREAM(getName() << " has limit of " << getEffortLimit() << " and offset of " << effort_offset_);
  }
  virtual ~GazeboJointHandle()
  {
  }

  /**
   *  \brief Used by controllers to set the desired position command of a joint.
   *  \param position The desired position, in radians or meters.
   *  \param velocity The desired velocity, in radians/sec or meters/sec.
   *  \param effort The desired effort, in Nm or N.
   *  \returns true if success, false otherwise.
   */
  virtual bool setPositionCommand(const float position,
                                  const float velocity,
                                  const float effort,
                                  bool update = false)
  {
    if (update)
    {
      desired_position_ += position;
      desired_velocity_ += velocity;
      desired_effort_ += effort;
      mode_ = MODE_CONTROL_POSITION;
      return true;
    }
    desired_position_ = position;
    desired_velocity_ = velocity;
    desired_effort_ = effort;
    mode_ = MODE_CONTROL_POSITION;
    return true;
  }

  /**
   *  \brief Used by controllers to set the desired velocity command of a joint.
   *  \param velocity The desired velocity, in radians/sec or meters/sec.
   *  \param effort The desired effort, in Nm or N.
   *  \returns true if success, false otherwise.
   */
  virtual bool setVelocityCommand(const float velocity,
                                  const float effort,
                                  bool update = false)
  {
    if (update)
    {
      if (isPositionControlled())
      {
        desired_velocity_ += velocity;
        desired_effort_ += effort;
        return true;
      }
      desired_velocity_ += velocity;
      desired_effort_ += effort;
      mode_ = MODE_CONTROL_VELOCITY;
      return true;
    }
    desired_velocity_ = velocity;
    desired_effort_ = effort;
    mode_ = MODE_CONTROL_VELOCITY;
    return true;
  }

  /**
   *  \brief Used by controllers to set the desired effort of a joint.
   *  \param effort The desired effort, in Nm or N.
   *  \returns true if success, false otherwise.
   */
  virtual bool setEffortCommand(const float effort,
                                bool update = false)
  {
    if (update)
    {
      if (isPositionControlled())
      {
        desired_effort_ += effort;
        return true;
      }
      else if (isVelocityControlled())
      {
        desired_effort_ += effort;
        return true;
      }
      else
      {
        desired_effort_ = effort;
        mode_ = MODE_CONTROL_EFFORT;
        return true;
      }
    }
    desired_effort_ = effort;
    mode_ = MODE_CONTROL_EFFORT;
    return true;
  }

  /** \brief Returns the position of the joint. */
  virtual double getPosition()
  {
    return joint_->GetAngle(0).Radian();
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

  /** \brief Get the lower positional limit */
  virtual float getPositionLowerLimit()
  {
    return joint_->GetLowerLimit(0).Radian();
  }

  /** \brief Get the upper positional limit */
  virtual float getPositionUpperLimit()
  {
    return joint_->GetUpperLimit(0).Radian();
  }

  /** \brief Get the velocity limit */
  virtual float getVelocityLimit()
  {
    return joint_->GetVelocityLimit(0);
  }

  /** \brief Get the effort limit */
  virtual float getEffortLimit()
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

  void clear()
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

  void update(const ros::Time now, const ros::Duration dt)
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
    float lim = getEffortLimit();
    applied_effort_ = std::max(-lim, std::min(effort, lim));

    if (debug_)
      ROS_INFO_STREAM(getName() << " commanded effort of " << effort);

    // Actually update
    joint_->SetForce(0, applied_effort_ + effort_offset_);
  }

  /** \brief Used only by the gripper */
  double setMaxEffort(double effort)
  {
    effort_limit_ = effort;
    return getEffortLimit();
  }

private:
  gazebo::physics::JointPtr joint_;

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

}  // namespace ubr1_gazebo

#endif  // UBR1_GAZEBO_JOINT_HANDLE_H
