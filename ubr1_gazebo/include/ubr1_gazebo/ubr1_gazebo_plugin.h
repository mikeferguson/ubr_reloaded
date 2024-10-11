/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-2024, Michael Ferguson
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

/* Authors: John Hsu, Michael Ferguson */

#ifndef UBR1_GAZEBO_UBR1_GAZEBO_PLUGIN_H
#define UBR1_GAZEBO_UBR1_GAZEBO_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <robot_controllers_interface/controller_manager.h>
#include <ubr1_gazebo/joint_handle.h>

#include <sensor_msgs/JointState.h>

namespace gazebo
{

class UBR1GazeboPlugin : public ModelPlugin
{
public:
  UBR1GazeboPlugin();
  ~UBR1GazeboPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();

private:

  /** \brief Callback to handle periodic updating for Gazebo */
  void OnUpdate();

  event::ConnectionPtr updateConnection;

  physics::ModelPtr model;
  std::vector<std::string> jointNames;

  common::Time prevUpdateTime;

  robot_controllers::ControllerManager* manager_;
  std::vector<ubr1_gazebo::GazeboJointHandlePtr> joints_;

  ros::Publisher joint_state_pub_;
  ros::NodeHandle nh_;

  ros::Time last_publish_;
};

}  // namespace gazebo

#endif  // UBR1_GAZEBO_UBR1_GAZEBO_PLUGIN_H
