/*********************************************************************
 *  Software License Agreement (BSD License)
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

#ifndef UBR_CONTROLLERS_SIMULATED_GRIPPER_CONTROLLER_H_
#define UBR_CONTROLLERS_SIMULATED_GRIPPER_CONTROLLER_H_

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ubr_controllers/controller.h>
#include <ubr_controllers/joint_handle.h>

#include <control_msgs/GripperCommandAction.h>

namespace ubr_controllers
{

/**
 *  \brief Controller for simulating the gripper
 */
class SimulatedGripperController : public Controller
{
  typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> server_t;

public:
  SimulatedGripperController() : initialized_(false) {}
  virtual ~SimulatedGripperController() {}

  /** \brief Initialize parameters, interfaces */
  virtual bool init(ros::NodeHandle& nh, ControllerManager* manager);

  /** \brief Start the controller. */
  virtual bool start();

  /** \brief Is this controller the head of the list? */
  virtual bool authoritative()
  {
    // should not run things on top of this
    return true;
  }

  /**
   *  \brief Preempt this controller.
   *  \param force If true, this controller will be stopped regardless
   *         of return value.
   *  \returns true if controller preempted successfully.
   */
  virtual bool preempt(bool force);

  /** \brief Update controller, called from controller_manager update */
  virtual bool update(const ros::Time now, const ros::Duration dt);

  /** \brief Get a list of joints this controls. */
  virtual std::vector<std::string> getJointNames();

private:
  /** \brief Callback for goal */
  void executeCb(const control_msgs::GripperCommandGoalConstPtr& goal);

  bool initialized_;
  std::vector<std::string> joint_names_;
  JointHandle* left_;
  JointHandle* right_;
  double left_effort_, right_effort_;

  boost::shared_ptr<server_t> server_;
};

}  // namespace ubr_controllers

#endif  // UBR_CONTROLLERS_SIMULATED_GRIPPER_CONTROLLER_H_
