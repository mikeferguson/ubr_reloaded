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

#ifndef UBR_CONTROLLERS_POINT_HEAD_H_
#define UBR_CONTROLLERS_POINT_HEAD_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ubr_controllers/controller.h>
#include <ubr_controllers/joint_handle.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/server/simple_action_server.h>

#include <ubr_controllers/trajectory.h>
#include <ubr_controllers/trajectory_spline_sampler.h>

#include <kdl/tree.hpp>

namespace ubr_controllers
{

class PointHeadController : public Controller
{
  typedef actionlib::SimpleActionServer<control_msgs::PointHeadAction> head_server_t;

public:
  PointHeadController() : initialized_(false) {}
  virtual ~PointHeadController() {}

  /** \brief Initialize parameters, interfaces. */
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
  void executeCb(const control_msgs::PointHeadGoalConstPtr& goal);

  bool initialized_;
  control_msgs::PointHeadResult result_;
  boost::shared_ptr<TrajectorySampler> sampler_;
  boost::mutex sampler_mutex_;

  /*
   * In certain cases, we want to start a trajectory at our last sample,
   * for instance if we were pre-empted (as is often the case with teleop)
   * we need to use the velocity and position of the last sample as a
   * starting point.
   */
  TrajectoryPoint last_sample_;
  bool preempted_;  /// action was preempted (has nothing to do with preempt() above

  std::string root_link_;
  JointHandle* head_pan_;
  JointHandle* head_tilt_;
  boost::shared_ptr<head_server_t> server_;

  KDL::Tree kdl_tree_;
  tf::TransformListener listener_;
};

}  // namespace ubr_controllers

#endif  // UBR_CONTROLLERS_POINT_HEAD_H_
