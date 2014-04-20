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
#include <ubr_controllers/simulated_gripper_controller.h>

PLUGINLIB_EXPORT_CLASS(ubr_controllers::SimulatedGripperController, ubr_controllers::Controller)

namespace ubr_controllers
{

bool SimulatedGripperController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  /* We absolutely need access to the controller manager */
  if (!manager)
  {
    initialized_ = false;
    return false;
  }

  Controller::init(nh, manager);

  /* Set Joint Names */
  joint_names_.push_back("left_gripper_joint");
  joint_names_.push_back("right_gripper_joint");

  /* Get Joint Handles */
  left_ = manager_->getJointHandle("left_gripper_joint");
  right_ = manager_->getJointHandle("right_gripper_joint");

  /* Setup ROS interfaces */
  server_.reset(new server_t(nh, "", /*"gripper_controller/gripper_action",*/
                             boost::bind(&SimulatedGripperController::executeCb, this, _1),
                             false));
  server_->start();

  initialized_ = true;

  /* Set gripper open, as would be calibrated to */
  left_effort_ = right_effort_ = 28.0;
  manager_->requestStart(name_);

  return true;
}

bool SimulatedGripperController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("SimulatedGripperController",
                    "Unable to start, not initialized.");
    return false;
  }

  return true;
}

bool SimulatedGripperController::preempt(bool force)
{
  if (!initialized_)
    return true;

  if (server_->isActive())
  {
    if (force)
    {
      /* Shut down the action */
      control_msgs::GripperCommandResult result;
      server_->setAborted(result, "Controller manager forced preemption.");
      return true;
    }
    /* Do not abort unless forced */
    return false;
  }

  /* Just holding position, go ahead and preempt us */
  return true;
}

bool SimulatedGripperController::update(const ros::Time now, const ros::Duration dt)
{
  if (!initialized_)
    return false;

  left_->setEffortCommand(left_effort_);
  right_->setEffortCommand(right_effort_);

  return true;
}

void SimulatedGripperController::executeCb(const control_msgs::GripperCommandGoalConstPtr& goal)
{
  control_msgs::GripperCommandFeedback feedback;
  control_msgs::GripperCommandResult result;

  if (!initialized_)
  {
    server_->setAborted(result, "Controller is not initialized.");
    return;
  }

  if (!manager_->requestStart(name_))
  {
    server_->setAborted(result, "Cannot execute, unable to start controller.");
    ROS_ERROR("Cannot execute, unable to start controller.");
    return;
  }

  /* If effort == 0.0, assume that user did not fill it in, and use max effort. */
  double effort = goal->command.max_effort;
  if (effort == 0.0)
    effort = 28.0;

  /* Set goal effort */
  float last_position_ = left_->getPosition() + right_->getPosition();
  ros::Time last_position_time_ = ros::Time::now();
  if (goal->command.position > last_position_)
  {
    left_effort_ = right_effort_ = effort;
  }
  else
  {
    left_effort_ = right_effort_ = -effort;
  }

  ros::Rate r(50);
  while (true)
  {
    /* Abort detection. */
    if (server_->isPreemptRequested() || !ros::ok())
    {
      ROS_DEBUG_NAMED("DefaultGripperPlugin", "Command preempted.");
      server_->setPreempted();
      break;
    }

    /* Publish feedback before possibly completing. */
    feedback.position = left_->getPosition() + right_->getPosition();
    feedback.effort = left_->getEffort() + right_->getEffort();
    feedback.reached_goal = false;
    feedback.stalled = false;
    server_->publishFeedback(feedback);

    /* Goal detection. */
    if (fabs(feedback.position - goal->command.position) < 0.002)
    {
      result.position = feedback.position;
      result.effort = feedback.effort;
      result.reached_goal = true;
      result.stalled = false;
      ROS_DEBUG_NAMED("DefaultGripperPlugin", "Command Succeeded.");
      server_->setSucceeded(result);
      return;
    }

    /* Stall detection. */
    if (fabs(feedback.position - last_position_) > 0.005)
    {
      last_position_ = feedback.position;
      last_position_time_ = ros::Time::now();
    }
    else
    {
      if (ros::Time::now() - last_position_time_ > ros::Duration(2.0))
      {
        result.position = feedback.position;
        result.effort = feedback.effort;
        result.reached_goal = false;
        result.stalled = true;
        ROS_DEBUG_NAMED("DefaultGripperPlugin", "Gripper stalled, but succeeding.");
        server_->setSucceeded(result);
        return;
      }
    }

    r.sleep();
  }
}

std::vector<std::string> SimulatedGripperController::getJointNames()
{
  return joint_names_;
}

}  // namespace ubr_controllers
