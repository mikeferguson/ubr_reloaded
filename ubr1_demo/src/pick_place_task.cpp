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

#include <ubr1_demo/pick_place_task.hpp>
#include <ubr1_demo/generate_grasps_from_msg.hpp>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

namespace mtc = moveit::task_constructor;

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

bool PickPlaceTask::configure(
  const rclcpp::Node::SharedPtr& node,
  const grasping_msgs::msg::Object & object,
  const std::vector<moveit_msgs::msg::Grasp> & grasps,
  const std::string & arm_group_name,
  const std::string & gripper_group_name,
  const std::string & eef_frame)
{
  RCLCPP_INFO(LOGGER, "*** Initializing task pipeline ***");

  // Reset ROS introspection before constructing the new object
  task_.reset();
  task_.reset(new mtc::Task());

  // Individual movement stages are collected within the Task object
  mtc::Task& t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel(node);

  // Create planners used in various stages
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);  // TODO: parameterize
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  // Set task properties
  t.setProperty("group", arm_group_name);
  t.setProperty("eef", gripper_group_name);
  t.setProperty("hand", gripper_group_name);
  t.setProperty("ik_frame", eef_frame);

  /****************************************************
   *               Current State                      *
   ***************************************************/
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");

    // Verify that object is not attached
    auto applicability_filter =
        std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
    applicability_filter->setPredicate([obj_id = object.name](const mtc::SolutionBase& s, std::string& comment)
    {
      if (s.start()->scene()->getCurrentState().hasAttachedBody(obj_id))
      {
        comment = "object with id '" + obj_id + "' is already attached and cannot be picked";
        return false;
      }
      return true;
    });
    t.add(std::move(applicability_filter));
  }

  /****************************************************
   *                 Open Gripper                     *
   ***************************************************/
  mtc::Stage* initial_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", sampling_planner);
    stage->setGroup(gripper_group_name);
    stage->setGoal("open");
    stage->setTimeout(5.0);
    initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
    t.add(std::move(stage));
  }

  /****************************************************
   *                 Move to Pick                     *
   ***************************************************/
  // Connect initial open-hand state with pre-grasp pose defined in the following
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    t.add(std::move(stage));
  }

  /****************************************************
   *               Pick Object                        *
   ***************************************************/
  mtc::Stage* pick_stage_ptr = nullptr;
  {
    // A SerialContainer combines several sub-stages, here for picking the object
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    /****************************************************
     *               Approach Object                    *
     ***************************************************/
    {
      // Move the eef link forward along its z-axis by an amount within the given min-max range
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", eef_frame);  // link to perform IK for
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });  // inherit group from parent stage
      stage->setMinMaxDistance(grasps[0].pre_grasp_approach.min_distance, grasps[0].pre_grasp_approach.desired_distance);
      stage->setDirection(grasps[0].pre_grasp_approach.direction);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Generate Grasp Pose                *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspsFromMsg>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions
      stage->setGrasps(grasps);

      // Compute IK for sampled grasp poses
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);  // limit number of solutions
      wrapper->setMinSolutionDistance(1.0);
      // Define virtual frame to reach the target_pose
      Eigen::Isometry3d grasp_frame_transform =
        Eigen::Translation3d(0.0, 0.0, 0.0) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
      wrapper->setIKFrame(grasp_frame_transform, eef_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });  // inherit properties from parent
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });  // inherit property from child solution
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
     *          Allow collision (object support)        *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,surface)");
      stage->allowCollisions({ object.name }, { object.support_surface }, true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Allow Collision (hand object)      *
     ***************************************************/
    {
      // Modify planning scene (w/o altering the robot's pose) to allow touching the object for picking
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions(
          object.name,
          t.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Close Hand                         *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", sampling_planner);
      stage->setGroup(gripper_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object.name, eef_frame);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *                  Lift object                     *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(grasps[0].post_grasp_retreat.min_distance, grasps[0].post_grasp_retreat.desired_distance);
      stage->setIKFrame(eef_frame);
      stage->properties().set("marker_ns", "lift_object");
      stage->setDirection(grasps[0].post_grasp_retreat.direction);
      grasp->insert(std::move(stage));
    }

    /****************************************************
     *          Forbid collision (object support)       *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions({ object.name }, { object.support_surface }, false);
      grasp->insert(std::move(stage));
    }

    pick_stage_ptr = grasp.get();  // remember for monitoring place pose generator

    // Add grasp container to task
    t.add(std::move(grasp));
  }

  /******************************************************
   *          Move to Place                             *
   *****************************************************/
  {
    // Connect the grasped state to the pre-place state, i.e. realize the object transport
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    t.add(std::move(stage));
  }

  /******************************************************
   *          Place Object                              *
   *****************************************************/
  // All placing sub-stages are collected within a serial container again
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group" });

    /******************************************************
     *                 Lower Object                       *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", eef_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(grasps[0].pre_grasp_approach.min_distance, grasps[0].pre_grasp_approach.desired_distance);
      stage->setDirection(grasps[0].pre_grasp_approach.direction);
      place->insert(std::move(stage));
    }

    /******************************************************
     *              Generate Place Pose                   *
     *****************************************************/
    {
      // Generate Place Pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object.name);

      // Set target pose - drop cube slightly above table and on opposite side
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = object.header.frame_id;
      p.pose = object.primitive_poses[0];
      p.pose.position.y *= -1;
      p.pose.position.z += 0.06;
      stage->setPose(p);
      stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      // Define virtual frame to reach the target_pose
      Eigen::Isometry3d grasp_frame_transform =
        Eigen::Translation3d(0.15, 0.0, 0.0) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
      wrapper->setIKFrame(grasp_frame_transform, eef_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
     *                   Open Hand                        *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(gripper_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    /******************************************************
    *           Forbid collision (hand, object)           *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(object.name, *t.getRobotModel()->getJointModelGroup(gripper_group_name), false);
      place->insert(std::move(stage));
    }

    /******************************************************
     *                 Detach Object                      *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object.name, eef_frame);
      place->insert(std::move(stage));
    }

    /******************************************************
     *                 Retreat Motion                     *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(grasps[0].post_grasp_retreat.min_distance, grasps[0].post_grasp_retreat.desired_distance);
      stage->setIKFrame(eef_frame);
      stage->properties().set("marker_ns", "retreat");
      stage->setDirection(grasps[0].post_grasp_retreat.direction);
      place->insert(std::move(stage));
    }

    // Add place container to task
    t.add(std::move(place));
  }

  /******************************************************
   *                  Move to Home                      *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    t.add(std::move(stage));
  }

  // Prepare task structure for planning
  try
  {
    t.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
    return false;
  }

  return true;
}

bool PickPlaceTask::plan(const std::size_t max_solutions)
{
  RCLCPP_INFO(LOGGER, "*** Start searching for task solutions ***");
  if (!task_->plan(max_solutions))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return false;
  }
  task_->introspection().publishSolution(*(task_->solutions().front()));
  RCLCPP_INFO(LOGGER, "*** Done planning ***");
  return true;
}

bool PickPlaceTask::execute()
{
  RCLCPP_INFO(LOGGER, "Executing solution trajectory");
  moveit_msgs::msg::MoveItErrorCodes execute_result;

  execute_result = task_->execute(*(task_->solutions().front()));

  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
