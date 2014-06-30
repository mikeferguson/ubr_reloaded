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

// Author: Michael Ferguson

#ifndef UBR_CONTROLLERS_FOLLOW_JOINT_TRAJECTORY_H
#define UBR_CONTROLLERS_FOLLOW_JOINT_TRAJECTORY_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ubr_controllers/controller.h>
#include <ubr_controllers/joint_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

#include <angles/angles.h>
#include <ubr_controllers/trajectory.h>
#include <ubr_controllers/trajectory_spline_sampler.h>

namespace ubr_controllers
{

/**
 *  \class FollowJointTrajectoryController
 *  \brief This ROS interface implements a FollowJointTrajectoryAction
 *         interface for controlling (primarily) robot arms.
 */
class FollowJointTrajectoryController : public Controller
{
  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> server_t;

public:
  FollowJointTrajectoryController() : initialized_(false) {}
  virtual ~FollowJointTrajectoryController() {}

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
  void executeCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  /** \brief Get a trajectory point from the current position/velocity/acceleration. */
  TrajectoryPoint getPointFromCurrent(bool incl_vel, bool incl_acc, bool zero_vel);

  bool initialized_;
  std::vector<JointHandle*> joints_;
  std::vector<std::string> joint_names_;
  boost::shared_ptr<server_t> server_;

  boost::shared_ptr<TrajectorySampler> sampler_;
  boost::mutex sampler_mutex_;

  bool stop_with_action_;  /// should we stop this controller when the
                           /// action has terminated (or hold position)?

  /*
   * In certain cases, we want to start a trajectory at our last sample,
   * for instance if we were pre-empted (as is often the case with teleop)
   * we need to use the velocity and position of the last sample as a
   * starting point.
   */
  TrajectoryPoint last_sample_;
  bool preempted_;  /// action was preempted
                    /// (has nothing to do with preempt() above).
  bool has_path_tolerance_;
  TrajectoryPoint path_tolerance_;

  bool has_goal_tolerance_;
  TrajectoryPoint goal_tolerance_;
  double goal_time_tolerance_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
};

}  // namespace ubr_controllers

#endif  // UBR_CONTROLLERS_FOLLOW_JOINT_TRAJECTORY_H_
