/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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

#ifndef UBR_CONTROLLERS_JOINT_HANDLE_H_
#define UBR_CONTROLLERS_JOINT_HANDLE_H_

#include <string>

namespace ubr_controllers
{

/**
 *  \brief Base class for joint handles, typically not instantiated.
 */
class JointHandle
{
public:
  JointHandle()
  {
  }
  virtual ~JointHandle()
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
    return false;
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
    return false;
  }

  /**
   *  \brief Used by controllers to set the desired effort of a joint.
   *  \param effort The desired effort, in Nm or N.
   *  \returns true if success, false otherwise.
   */
  virtual bool setEffortCommand(const float effort,
                                bool update = false)
  {
    return false;
  }

  /**  \brief Returns the position of the joint. */
  virtual double getPosition() { return 0.0; }

  /** \brief Returns the velocity of the joint. */
  virtual double getVelocity() { return 0.0; }

  /** \brief Returns the effort applied to the joint. */
  virtual double getEffort() { return 0.0; }

  /** \brief Get the lower positional limit */
  virtual float getPositionLowerLimit() { return 0.0; }

  /** \brief Get the upper positional limit */
  virtual float getPositionUpperLimit() { return 0.0; }

  /** \brief Get the velocity limit */
  virtual float getVelocityLimit() { return 0.0; }

  /** \brief Get the effort limit */
  virtual float getEffortLimit() { return 0.0; }

  /** \brief Get the name of this joint. */
  virtual std::string getName() { return "invalid"; }

private:
  // You no copy...
  JointHandle(const JointHandle&);
  JointHandle& operator=(const JointHandle&);
};

}  // namespace ubr_controllers

#endif  // UBR_CONTROLLERS_JOINT_HANDLE_H_
