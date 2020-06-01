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

/* Author: Michael Ferguson */

#ifndef UBR1_GAZEBO_CONTROLLERS_SIMULATED_GRIPPER_CONTROLLER_H_
#define UBR1_GAZEBO_CONTROLLERS_SIMULATED_GRIPPER_CONTROLLER_H_

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers_interface/joint_handle.h>
#include <ubr1_gazebo/joint_handle.h>

#include <control_msgs/GripperCommandAction.h>

namespace ubr1_gazebo_controllers
{

/**
 *  \brief Controller for simulating the gripper
 */
class SimulatedGripperController : public robot_controllers::Controller
{
  typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> server_t;

public:
  SimulatedGripperController() : initialized_(false) {}
  virtual ~SimulatedGripperController() {}

  /** \brief Initialize parameters, interfaces */
  virtual int init(ros::NodeHandle& nh, robot_controllers::ControllerManager* manager);

  /** \brief Start the controller. */
  virtual bool start();

  /** \brief Stop the controller. */
  virtual bool stop(bool force);

  /** \brief Reset the controller. */
  virtual bool reset();

  /** \brief Update controller, called from controller_manager update */
  virtual void update(const ros::Time& now, const ros::Duration& dt);

  /** \brief Get a list of joints this controls. */
  virtual std::vector<std::string> getCommandedNames();
  virtual std::vector<std::string> getClaimedNames();

private:
  /** \brief Callback for goal */
  void executeCb(const control_msgs::GripperCommandGoalConstPtr& goal);

  bool initialized_;
  std::vector<std::string> joint_names_;
  robot_controllers::ControllerManager* manager_;

  ubr1_gazebo::GazeboJointHandlePtr left_;
  ubr1_gazebo::GazeboJointHandlePtr right_;

  // The goal pose for the gripper
  double goal_;
  double fudge_scale_;

  boost::shared_ptr<server_t> server_;
};

}  // namespace ubr1_gazebo_controllers

#endif  // UBR1_GAZEBO_CONTROLLERS_SIMULATED_GRIPPER_CONTROLLER_H_
