/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-2024, Michael Ferguson
 *  Copyright (c) 2014, Unbounded Robotics Inc.
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

#ifndef UBR1_GAZEBO_CONTROLLERS_SIMULATED_BELLOWS_CONTROLLER_H_
#define UBR1_GAZEBO_CONTROLLERS_SIMULATED_BELLOWS_CONTROLLER_H_

#include <string>

#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers_interface/joint_handle.h>

namespace ubr1_gazebo_controllers
{

/**
 *  \brief World's stupidest controller -- this makes the bellows
 *         be in the right place. If you ever find yourself reading this
 *         code... I apologize in advance.
 */
class SimulatedBellowsController : public robot_controllers_interface::Controller
{
public:
  SimulatedBellowsController() : initialized_(false) {}
  virtual ~SimulatedBellowsController() {}

  /** \brief Initialize parameters, interfaces */
  virtual int init(const std::string& name,
                   rclcpp::Node::SharedPtr node,
                   robot_controllers_interface::ControllerManagerPtr manager);

  /** \brief Start the controller. */
  virtual bool start();

  /** \brief Stop the controller. */
  virtual bool stop(bool force);

  /** \brief Reset the controller. */
  virtual bool reset();

  /** \brief Update controller, called from controller_manager update */
  virtual void update(const rclcpp::Time& now, const rclcpp::Duration& dt);

  virtual std::string getType()
  {
    return "SimulatedBellowsController";
  }

  /** \brief Get a list of joints this controls. */
  virtual std::vector<std::string> getCommandedNames();
  virtual std::vector<std::string> getClaimedNames();

private:
  bool initialized_;
  robot_controllers_interface::JointHandlePtr bellows_;
  robot_controllers_interface::JointHandlePtr torso_lift_;
};

}  // namespace ubr1_gazebo_controllers

#endif  // UBR1_GAZEBO_CONTROLLERS_SIMULATED_BELLOWS_CONTROLLER_H_
