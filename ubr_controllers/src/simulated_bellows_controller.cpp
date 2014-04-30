/*********************************************************************
 *  Software License Agreement (BSD License)
 *
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

#include <pluginlib/class_list_macros.h>
#include <ubr_controllers/simulated_bellows_controller.h>

PLUGINLIB_EXPORT_CLASS(ubr_controllers::SimulatedBellowsController, ubr_controllers::Controller)

namespace ubr_controllers
{

bool SimulatedBellowsController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  /* We absolutely need access to the controller manager */
  if (!manager)
  {
    initialized_ = false;
    return false;
  }

  Controller::init(nh, manager);

  /* Get Joint Handles */
  bellows_ = manager_->getJointHandle("bellows_joint");
  torso_lift_ = manager_->getJointHandle("torso_lift_joint");

  initialized_ = true;
  return initialized_;
}

bool SimulatedBellowsController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("SimulatedBellowsController",
                    "Unable to start, not initialized.");
    return false;
  }

  return true;
}

bool SimulatedBellowsController::preempt(bool force)
{
  if (!initialized_)
    return true;

  if (force)
    return true;

  /* If we preempt -- the bellows will fall -- and the world will end! */
  return false;
}

bool SimulatedBellowsController::update(const ros::Time now, const ros::Duration dt)
{
  if (!initialized_)
    return false;

  /* I warned you this controller was stupid */
  bellows_->setPositionCommand(torso_lift_->getPosition() / -2.0, 0.0, 0.0);

  return true;
}

std::vector<std::string> SimulatedBellowsController::getJointNames()
{
  std::vector<std::string> names;
  names.push_back("bellows_joint");
  return names;
}

}  // namespace ubr_controllers
