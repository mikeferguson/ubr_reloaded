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

// Author: Michael Ferguson

#include <pluginlib/class_list_macros.hpp>
#include <ubr1_gazebo/simulated_bellows_controller.h>

PLUGINLIB_EXPORT_CLASS(ubr1_gazebo_controllers::SimulatedBellowsController,
                       robot_controllers_interface::Controller)

namespace ubr1_gazebo_controllers
{

int SimulatedBellowsController::init(const std::string& name,
                                     rclcpp::Node::SharedPtr node,
                                     robot_controllers_interface::ControllerManagerPtr manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  robot_controllers_interface::Controller::init(name, node, manager);

  // Get Joint Handles
  bellows_ = manager->getJointHandle("bellows_joint");
  torso_lift_ = manager->getJointHandle("torso_lift_joint");

  initialized_ = true;

  // Start this controller
  manager->requestStart(getName());

  return 0;
}

bool SimulatedBellowsController::start()
{
  if (!initialized_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()), "Unable to start, not initialized.");
    return false;
  }

  return true;
}

bool SimulatedBellowsController::stop(bool force)
{
  if (!initialized_)
    return true;

  if (force)
    return true;

  // If we preempt -- the bellows will fall -- and the world will end!
  return false;
}

bool SimulatedBellowsController::reset()
{
  return true;
}

void SimulatedBellowsController::update(const rclcpp::Time&, const rclcpp::Duration&)
{
  // I warned you this controller was stupid
  bellows_->setPosition(torso_lift_->getPosition() / -2.0, 0.0, 0.0);
}

std::vector<std::string> SimulatedBellowsController::getCommandedNames()
{
  std::vector<std::string> names;
  names.push_back("bellows_joint");
  return names;
}

std::vector<std::string> SimulatedBellowsController::getClaimedNames()
{
  std::vector<std::string> names;
  // No claimed names
  return names;
}

}  // namespace ubr1_gazebo_controllers
