/*********************************************************************
 * Software License Agreement (BSD License)
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

#ifndef UBR_CONTROLLERS_CONTROLLER_MANAGER_H_
#define UBR_CONTROLLERS_CONTROLLER_MANAGER_H_

#include <string>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <ubr_controllers/joint_handle.h>
#include <ubr_controllers/controller.h>

#include <ubr_msgs/ControllerInfo.h>
#include <ubr_msgs/UpdateControllers.h>

namespace ubr_controllers
{

/**
 *  \brief Base class for managing controllers.
 */
class ControllerManager
{
public:
  ControllerManager() :
    loader_("ubr_controllers", "ubr_controllers::Controller")
  {
  }
  virtual ~ControllerManager()
  {
  }

  virtual bool init(ros::NodeHandle& nh)
  {
    // Start default controllers
    XmlRpc::XmlRpcValue names;
    if (nh.getParam("controllers", names))
    {
      if (names.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int i = 0; i < names.size(); ++i)
        {
          XmlRpc::XmlRpcValue &name_value = names[i];
          if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
            continue;
          loadController(static_cast<std::string>(name_value));
        }
      }
      else
      {
        ROS_ERROR("controllers is not in a list");
      }
    }
    else
    {
      ROS_WARN("No controllers specified");
    }

    update_service_ = nh.advertiseService("update_controllers", &ControllerManager::updateCallback, this);
  }

  /**
   *  \brief Request a controller be started.
   *  \param name Name of the controller.
   *  \returns true if successful.
   */
  virtual bool requestStart(const std::string& name)
  {
    boost::recursive_mutex::scoped_lock lock(update_lock_);

    // Check that controller is not already running
    for (size_t i = 0; i < active_.size(); ++i)
    {
      if (active_[i]->getName() == name)
      {
        return true;
      }
    }

    // Find controller
    int idx = -1;
    for (size_t i = 0; i < controllers_.size(); ++i)
    {
      if (controllers_[i]->getName() == name)
      {
        idx = i;
        break;
      }
    }

    // No controller to load
    if (idx == -1)
    {
      ROS_ERROR_STREAM("No such controller to start: " << name);
      return false;
    }

    // Check if controller can be started or is in conflict with another
    if (active_.size() > 0)
    {
      std::vector<std::string> joints = controllers_[idx]->getJointNames();
      for (size_t a = 0; a < active_.size(); ++a)
      {
        if (!compareController(joints, active_[a]))
        {
          ROS_ERROR("Controller conflicts with an active controller.");
          return false;
        }
      }
    }

    // Activate it
    if (controllers_[idx]->start())
    {
      ROS_INFO_STREAM("Started " << name);
      active_.push_back(controllers_[idx]);
      return true;
    }

    ROS_ERROR_STREAM("Failed to start " << name);
    return false;
  }

  /**
   *  \brief Request a controller be stopped.
   *  \param name Name of the controller.
   *  \returns true if successful.
   */
  virtual bool requestStop(const std::string& name)
  {
    boost::recursive_mutex::scoped_lock lock(update_lock_);

    for (size_t i = 0; i < active_.size(); ++i)
    {
      if (active_[i]->getName() == name)
      {
        active_.erase(active_.begin() + i);
        ROS_INFO_STREAM("Stopped " << name);
        return true;
      }
    }
    return false;
  }

  virtual bool update(const ros::Time now, const ros::Duration dt)
  {
    boost::recursive_mutex::scoped_lock lock(update_lock_);

    /*
     * NOTE: When setting this up on a new system, you should be
     *       sure to clear out the previous setpoints in your update().
     *       However, this is JointHandle-dependent, and left to the
     *       derived implementation which should then call this one.
     */

    for (size_t i = 0; i < active_.size(); ++i)
    {
      int idx = active_.size() - i - 1;
      active_[idx]->update(now, dt);
    }

    return true;
  }

  virtual bool loadController(const std::string& name)
  {
    boost::recursive_mutex::scoped_lock lock(update_lock_);

    ros::NodeHandle nh(name);

    std::string type;
    if (nh.getParam("type", type))
    {
      boost::shared_ptr<ubr_controllers::Controller> c;

      c = loader_.createInstance(type);
      c->init(nh, this);

      controllers_.push_back(c);
      return true;
    }

    ROS_ERROR("Could not load controller as type is not specified.");
    return false;
  }

  virtual JointHandle* getJointHandle(const std::string& name)
  {
    /*
     * NOTE: This has to be defined in derived manager
     */
    return new JointHandle();
  }

  /**
   *  \brief Get a controller.
   */
  virtual ubr_controllers::Controller* getController(const std::string& name)
  {
    for (int i = 0; i < controllers_.size(); ++i)
    {
      if (controllers_[i]->getName() == name)
      {
        return controllers_[i].get();
      }
    }
    return 0;
  }

protected:
  /* Return true if we are ok to continue */
  bool compareController(std::vector<std::string> joints,
         boost::shared_ptr<ubr_controllers::Controller> controller)
  {
    if (controller->authoritative())
    {
      std::vector<std::string> joints2 = controller->getJointNames();
      for (size_t i = 0; i < joints.size(); ++i)
      {
        for (size_t j = 0; j < joints2.size(); ++j)
        {
          if (joints[i] == joints2[j])
          {
            // conflict found, try to unload this controller
            if (requestStop(controller->getName()))
            {
              ROS_INFO_STREAM("Stopped " << controller->getName());
              return true;
            }
            return false;
          }
        }
      }
    }
    return true;
  }

  bool updateCallback(ubr_msgs::UpdateControllers::Request& req, ubr_msgs::UpdateControllers::Response& resp)
  {
    boost::recursive_mutex::scoped_lock lock(update_lock_);

    for (size_t i = 0; i < req.start.size(); ++i)
      requestStart(req.start[i]);

    for (size_t i = 0; i < req.stop.size(); ++i)
      requestStop(req.stop[i]);

    // TODO fill in active and available controller lists

    return true;
  }

  boost::recursive_mutex update_lock_;
  pluginlib::ClassLoader<ubr_controllers::Controller> loader_;
  std::vector< boost::shared_ptr<ubr_controllers::Controller> > controllers_;
  std::vector< boost::shared_ptr<ubr_controllers::Controller> > active_;

  ros::ServiceServer update_service_;
};

}  // namespace ubr_controllers

#endif  // UBR_CONTROLLERS_CONTROLLER_MANAGER_H_
