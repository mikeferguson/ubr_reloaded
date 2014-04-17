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

/* Author: Michael Ferguson */

#ifndef UBR1_GAZEBO_CONTROLLER_MANAGER_H_
#define UBR1_GAZEBO_CONTROLLER_MANAGER_H_

#include <ros/ros.h>

#include <ubr_controllers/controller.h>
#include <ubr_controllers/controller_manager.h>
#include <ubr1_gazebo/joint_handle.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace ubr1_gazebo
{

class GazeboControllerManager : public ubr_controllers::ControllerManager
{
public:
  GazeboControllerManager(gazebo::physics::ModelPtr robot) : nh_("~"), robot_(robot)
  {
    this->joints_ = robot->GetJoints();

    for (gazebo::physics::Joint_V::iterator it = this->joints_.begin(); it != this->joints_.end(); ++it)
    {
      boost::shared_ptr<GazeboJointHandle> jh;
      jh.reset(new GazeboJointHandle(*it));
      this->jointMap_[(*it)->GetName()] = jh;
    }

    init(nh_);
  }
  ~GazeboControllerManager()
  {
  }

  virtual bool init(ros::NodeHandle & nh)
  {
    /* Load default controllers */
    ubr_controllers::ControllerManager::init(nh);

    /* Start gravity compensation */
    requestStart("arm_controller/gravity_compensation");
  }

  virtual bool update(const ros::Time now, const ros::Duration dt)
  {
    /* Clear previous commands */
    for (std::map<std::string, boost::shared_ptr<GazeboJointHandle> >::const_iterator it = this->jointMap_.begin();
             it != this->jointMap_.end(); ++it)
    {
      it->second->clear();
    }

    /* Add controller updates */
    ControllerManager::update(now, dt);

    /* Set commands in Gazebo */
    for (std::map<std::string, boost::shared_ptr<GazeboJointHandle> >::const_iterator it = this->jointMap_.begin();
             it != this->jointMap_.end(); ++it)
    {
      it->second->update(now, dt);
    }

    return true;
  }

  /**
   *  \brief Return a handle to the internal joints.
   */
  virtual ubr_controllers::JointHandle* getJointHandle(const std::string& name)
  {
    std::map<std::string, boost::shared_ptr<GazeboJointHandle> >::iterator it = this->jointMap_.find(name);
    if (it != this->jointMap_.end())
      return it->second.get();
    else
    {
      ROS_ERROR("Did not find joint [%s]", name.c_str());
      return new ubr_controllers::JointHandle();
    }
  }

private:
  ros::NodeHandle nh_;
  gazebo::physics::ModelPtr robot_;
  gazebo::physics::Joint_V joints_;
  std::map<std::string, boost::shared_ptr<GazeboJointHandle> > jointMap_;
};

}  // namespace ubr1_gazebo

#endif  // UBR1_GAZEBO_CONTROLLER_MANAGER_H_
