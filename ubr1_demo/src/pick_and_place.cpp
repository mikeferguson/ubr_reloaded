/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2024 Michael Ferguson
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <simple_actions/simple_client.hpp>
#include <grasping_msgs/action/find_graspable_objects.hpp>
#include <ubr1_demo/pick_place_task.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

using grasping_msgs::action::FindGraspableObjects;

class PickAndPlace
{
public:
  PickAndPlace(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  bool doTask(double x_offset = 0.5);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PickPlaceTask> task_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  simple_actions::SimpleActionClient<FindGraspableObjects> client_;
  bool verbose_;
};

PickAndPlace::PickAndPlace(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("pick_and_place", options) },
    client_(node_, "find_objects"),
    verbose_(true)
{
  //client_.waitForServer();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PickAndPlace::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

bool PickAndPlace::doTask(double x_offset)
{
  // Find graspable objects
  FindGraspableObjects::Goal goal;
  goal.plan_grasps = true;
  // Have to set spin_locally to false, else we get error about already being added to executor
  auto find_result = client_.execute(goal, nullptr, false);

  RCLCPP_INFO(LOGGER, "Found %lu objects", find_result.objects.size());
  RCLCPP_INFO(LOGGER, "Found %lu surfaces", find_result.support_surfaces.size());

  // Remove all previous objects
  {
    auto objects = planning_scene_.getKnownObjectNames();
    planning_scene_.removeCollisionObjects(objects);
  }

  // Insert objects, find the one to grasp
  int best_object_idx = -1;
  double best_object_dist = 0.35;
  for (size_t i = 0; i < find_result.objects.size(); ++i)
  {
    auto & obj = find_result.objects[i];

    // Don't add far away objects
    if (std::fabs(obj.object.primitive_poses[0].position.x) > 1.0 ||
        std::fabs(obj.object.primitive_poses[0].position.y) > 1.0)
    {
      continue;
    }

    // Add object to planning scene
    moveit_msgs::msg::CollisionObject object;
    object.id = "object" + std::to_string(i);
    object.header.frame_id = obj.object.header.frame_id;
    object.primitives = obj.object.primitives;
    object.primitive_poses = obj.object.primitive_poses;
    if (!planning_scene_.applyCollisionObject(object))
    {
      throw std::runtime_error("Failed to insert: " + object.id);
    }

    if (verbose_)
    {
      RCLCPP_INFO(LOGGER, "Object %s", object.id.c_str());
      RCLCPP_INFO(LOGGER, "  %f %f %f", object.primitive_poses[0].position.x,
                                        object.primitive_poses[0].position.y,
                                        object.primitive_poses[0].position.z);
    }

    // Object must have grasps to be considered
    if (obj.grasps.empty()) continue;

    // Choose the object in front of the robot
    double dx = obj.object.primitive_poses[0].position.x - x_offset;
    double dy = obj.object.primitive_poses[0].position.y;
    double d = std::hypot(dx, dy);
    if (d < best_object_dist)
    {
      best_object_idx = i;
      best_object_dist = d;
    }
  }

  if (best_object_idx < 0)
  {
    RCLCPP_ERROR(LOGGER, "Nothing to grasp!");
    return false;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Grasping object %d", best_object_idx);
  }

  auto obj = find_result.objects[best_object_idx];
  obj.object.name = "object" + std::to_string(best_object_idx);

  // Insert Table
  for (auto & obj : find_result.support_surfaces)
  {
    // Don't add far away surfaces
    if (std::fabs(obj.primitive_poses[0].position.x) > 1.0 ||
        std::fabs(obj.primitive_poses[0].position.y) > 1.0)
    {
      continue;
    }

    moveit_msgs::msg::CollisionObject surface;
    surface.id = obj.name;
    surface.header.frame_id = obj.header.frame_id;
    surface.primitives = obj.primitives;
    surface.primitive_poses = obj.primitive_poses;
    // Extend surface to floor so we don't sweep the (non-visible) legs
    double height = surface.primitive_poses[0].position.z;
    surface.primitives[0].dimensions[2] += height;
    surface.primitive_poses[0].position.z += -height / 2.0;
    // Make table wider (since camera has narrow FOV)
    surface.primitives[0].dimensions[1] = 1.5;
    if (!planning_scene_.applyCollisionObject(surface))
    {
      throw std::runtime_error("Failed to insert: " + surface.id);
    }

    if (verbose_)
    {
      RCLCPP_INFO(LOGGER, "Surface: %s", obj.name.c_str());
      RCLCPP_INFO(LOGGER, "  %f %f %f", obj.primitive_poses[0].position.x,
                                        obj.primitive_poses[0].position.y,
                                        obj.primitive_poses[0].position.z);
    }
  }

  // Set color of object we are grasping
  auto objects = planning_scene_.getObjects();
  std_msgs::msg::ColorRGBA orange;
  orange.r = 223.0 / 256.0;
  orange.g = 90.0 / 256.0;
  orange.b = 12.0 / 256.0;
  orange.a = 1.0;
  planning_scene_.applyCollisionObject(objects[obj.object.name], orange);
  std_msgs::msg::ColorRGBA black;
  black.a = 1.0;
  planning_scene_.applyCollisionObject(objects[obj.object.support_surface], black);

  // TODO: add exit if we are just doing object update?

  task_.reset(new PickPlaceTask("grasping_demo"));
  task_->configure(node_,
                   obj.object,
                   obj.grasps,
                   "arm",
                   "gripper",
                   "wrist_roll_link");
  if (task_->plan(5))
  {
    task_->execute();
    return true;
  }

  return false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto pnp = std::make_shared<PickAndPlace>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &pnp]() {
    executor.add_node(pnp->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(pnp->getNodeBaseInterface());
  });

  pnp->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
