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

// Authors: John Hsu, Michael Ferguson

#include <ubr1_gazebo/joint_handle.h>
#include <ubr1_gazebo/ubr1_gazebo_plugin.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(UBR1GazeboPlugin)

UBR1GazeboPlugin::UBR1GazeboPlugin() : nh_()
{
}

UBR1GazeboPlugin::~UBR1GazeboPlugin()
{
  delete this->manager_;
}

void UBR1GazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;
  last_publish_ = ros::Time(this->model->GetWorld()->SimTime().Double());

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&UBR1GazeboPlugin::OnUpdate, this));
}

void UBR1GazeboPlugin::Init()
{
  // Create ControllerManager instance
  this->manager_ = new robot_controllers::ControllerManager();

  // Setup joints
  gazebo::physics::Joint_V joints = this->model->GetJoints();
  for (auto it = joints.begin(); it != joints.end(); ++it)
  {
    joints_.emplace_back(new ubr1_gazebo::GazeboJointHandle(*it));
    robot_controllers::JointHandlePtr jh = joints_.back();
    this->manager_->addJointHandle(jh);
  }

  // Setup controllers
  ros::NodeHandle nh("~");
  this->manager_->init(nh);

  // Publish joint states only after controllers are fully ready
  this->joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  ROS_INFO("Finished initializing UBR1GazeboPlugin");
}

void UBR1GazeboPlugin::OnUpdate()
{
  // Don't try to update/publish if we are shutting down
  if (!ros::ok()) return;

  // Get time and timestep for controllers
  common::Time currTime = this->model->GetWorld()->SimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;
  double dt = stepTime.Double();
  ros::Time now = ros::Time(currTime.Double());

  // Update controllers
  this->manager_->update(now, ros::Duration(dt));

  // Copy commands to Gazebo
  for (auto joint : joints_)
  {
    joint->update(now, ros::Duration(dt));
  }

  // Limit publish rate
  if (now - last_publish_ < ros::Duration(0.01))
    return;

  // Publish joint_state message
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time(currTime.Double());
  gazebo::physics::Joint_V joints = this->model->GetJoints();
  for (gazebo::physics::Joint_V::iterator it = joints.begin(); it != joints.end(); ++it)
  {
    robot_controllers::JointHandlePtr j = this->manager_->getJointHandle((*it)->GetName());
    js.name.push_back((*it)->GetName());
    js.position.push_back(j->getPosition());
    js.velocity.push_back(j->getVelocity());
    js.effort.push_back(j->getEffort());
  }
  joint_state_pub_.publish(js);

  last_publish_ = now;
}

