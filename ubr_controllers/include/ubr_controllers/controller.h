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

/* Author: Michael Ferguson */

#ifndef UBR_CONTROLLERS_CONTROLLER_
#define UBR_CONTROLLERS_CONTROLLER_

#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

/**
 *  \namespace ubr_controllers
 *  \brief Namespace for pluggable controller infrastructure.
 */
namespace ubr_controllers
{

class ControllerManager;

/**
 *  \class Controller
 *  \brief Base class for controller plugins.
 */
class Controller
{
public:
  Controller()
  {
    // TODO: track constructed vs. initialized
  }
  virtual ~Controller() {}

  /**
   *  \brief Initialize parameters, interfaces
   */
  virtual bool init(ros::NodeHandle& nh, ControllerManager* manager)
  {
    manager_ = manager;
    name_ = nh.getNamespace();
    if (name_[0] == '/')
      name_.erase(0, 1);
  }

  /**
   *  \brief Start the controller.
   */
  virtual bool start()
  {
    // error, no controller here.
    return false;
  }

  /**
   *  \brief Is this controller the head of the list?
   */
  virtual bool authoritative()
  {
    // most controllers return true.
    return true;
  }

  /**
   *  \brief Preempt this controller.
   *  \param force If true, this controller will be stopped regardless
   *         of return value.
   *  \returns true if controller preempted successfully.
   */
  virtual bool preempt(bool force)
  {
    return true;
  }

  /**
   *  \brief Update controller
   */
  virtual bool update(const ros::Time now, const ros::Duration dt)
  {
    // return false if we don't actually have output
    return false;
  }

  /**
   *  \brief Get the name of this controller.
   */
  virtual std::string getName()
  {
    return name_;
  }

  /**
   *  \brief Get a list of joints this controls.
   */
  virtual std::vector<std::string> getJointNames()
  {
    std::vector<std::string> names;
    return names;
  }

protected:
  ControllerManager* manager_;
  std::string name_;
};

}  // namespace ubr_controllers

// drag in here for derived controllers which need to use manager_
#include <ubr_controllers/controller_manager.h>

#endif
