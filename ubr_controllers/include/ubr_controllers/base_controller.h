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

#ifndef UBR_CONTROLLERS_BASE_CONTROLLER_H_
#define UBR_CONTROLLERS_BASE_CONTROLLER_H_

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ubr_controllers/controller.h>
#include <ubr_controllers/joint_handle.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace ubr_controllers
{

/**
 *  \brief ROS-aware controller to manage a differential drive mobile base. This
 *         subcribes to cmd_vel topic, publishes odom and tf, and manages the two
 *         wheel joints.
 */
class BaseController : public Controller
{
public:
  BaseController() : initialized_(false)
  {
    theta_ = 0.0;

    odom_.pose.pose.orientation.z = 0.0;
    odom_.pose.pose.orientation.w = 1.0;
    odom_.header.frame_id = "odom";

    last_sent_x_ = desired_x_ = 0.0;
    last_sent_r_ = desired_r_ = 0.0;

    left_last_timestamp_ = right_last_timestamp_ = 0.0;
    last_command_ = last_update_ = ros::Time(0.0);
  }
  virtual ~BaseController() {}

  /**
   *  \brief Initialize parameters, interfaces
   */
  virtual bool init(ros::NodeHandle& nh, ControllerManager* manager);

  /**
   *  \brief Start the controller.
   */
  virtual bool start();

  /**
   *  \brief Is this controller the head of the list?
   */
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

  /**
   *  \brief Update controller
   */
  virtual bool update(const ros::Time now, const ros::Duration dt);

  /**
   *  \brief Get a list of joints this controls.
   */
  virtual std::vector<std::string> getJointNames()
  {
    std::vector<std::string> names;
    if (left_)
      names.push_back(left_->getName());
    if (right_)
      names.push_back(right_->getName());
    return names;
  }

  /**
   *  \brief Command callback from either a ROS topic, or a higher controller.
   */
  void command(const geometry_msgs::TwistConstPtr& msg);

  /** \brief Publish odom, possibly tf */
  bool publish(ros::Time time);

private:
  bool initialized_;

  void updateCallback(const ros::WallTimerEvent& event);

  // Set base wheel speeds in m/s
  void setCommand(float left, float right);

  JointHandle* left_;
  JointHandle* right_;

  double track_width_;
  double radians_per_meter_;
  double theta_;

  double moving_threshold_;

  double max_velocity_x_;
  double max_velocity_r_;
  double max_acceleration_x_;
  double max_acceleration_r_;

  // These are the inputs from the ROS topic
  float desired_x_;
  float desired_r_;

  // These are from controller update
  float last_sent_x_;
  float last_sent_r_;

  float left_last_position_;
  float right_last_position_;
  double left_last_timestamp_;
  double right_last_timestamp_;

  ros::Time last_command_;
  ros::Time last_update_;
  ros::Duration timeout_;

  nav_msgs::Odometry odom_;
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_sub_;

  boost::shared_ptr<tf::TransformBroadcaster> broadcaster_;
  bool publish_tf_;

  std::string odometry_frame_;
  std::string base_frame_;

  bool enabled_;
  bool ready_;
};

}  // namespace ubr_controllers

#endif  // UBR_CONTROLLERS_BASE_CONTROLLER_H_
