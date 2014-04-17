/*********************************************************************
 *  Software License Agreement (BSD License)
 *
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

#include <ubr1_gazebo/ubr1_gazebo_plugin.h>
#include <ubr_controllers/base_controller.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(UBR1GazeboPlugin)

UBR1GazeboPlugin::UBR1GazeboPlugin() : nh_()
{
  /* Create a periodic timer to publish joint_states, odom, etc */
  pubtimer_ = nh_.createTimer(ros::Duration(0.01), &UBR1GazeboPlugin::OnTimer, this);
}

UBR1GazeboPlugin::~UBR1GazeboPlugin()
{
  delete this->manager_;
}

void UBR1GazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&UBR1GazeboPlugin::OnUpdate, this));
}

void UBR1GazeboPlugin::Init()
{
  this->joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
  this->manager_ = new ubr1_gazebo::GazeboControllerManager(this->model);
  ROS_INFO("Finished initializing UBR1GazeboPlugin");
}

void UBR1GazeboPlugin::OnUpdate()
{
  /* Get time and timestep for controllers */
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;
  double dt = stepTime.Double();

  /* Update controllers */
  this->manager_->update(ros::Time(currTime.Double()), ros::Duration(dt));
}

void UBR1GazeboPlugin::OnTimer(const ros::TimerEvent& event)
{
  /* Don't try to publish if we are shutting down. */
  if (!ros::ok()) return;

  /* Get gazebo time */
  common::Time currTime = this->model->GetWorld()->GetSimTime();

  /* Publish joint_state message */
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time(currTime.Double());
  gazebo::physics::Joint_V joints = this->model->GetJoints();
  for (gazebo::physics::Joint_V::iterator it = joints.begin(); it != joints.end(); ++it)
  {
    js.name.push_back((*it)->GetName());
    js.position.push_back((*it)->GetAngle(0).Radian());
    js.velocity.push_back((*it)->GetVelocity(0));
    js.effort.push_back((*it)->GetForce(0u));
  }
  joint_state_pub_.publish(js);

  /* TODO: Publish IMU */

  /* Publish Base Odometry */
  ubr_controllers::Controller * base = this->manager_->getController("base_controller");
  if (base)
  {
    dynamic_cast<ubr_controllers::BaseController*>(base)->publish(ros::Time::now());
  }
}
