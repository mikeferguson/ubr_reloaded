/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Michael Ferguson
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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
 *   * Neither the name of Bielefeld University nor the names of its
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

#include <ubr1_demo/generate_grasps_from_msg.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("GenerateGraspsFromMsg");

GenerateGraspsFromMsg::GenerateGraspsFromMsg(const std::string& name) : MonitoringGenerator(name)
{
  setCostTerm(std::make_unique<cost::Constant>(0.0));

  auto& p = properties();
  p.declare<std::vector<moveit_msgs::msg::Grasp>>("grasps", "grasps to pass on in spawned states");
}

void GenerateGraspsFromMsg::reset()
{
  upstream_solutions_.clear();
  MonitoringGenerator::reset();
}

void GenerateGraspsFromMsg::onNewSolution(const SolutionBase& s)
{
  // It's safe to store a pointer to this solution, as the generating stage stores it
  upstream_solutions_.push(&s);
}

bool GenerateGraspsFromMsg::canCompute() const
{
  return !upstream_solutions_.empty();
}

void GenerateGraspsFromMsg::compute()
{
  if (upstream_solutions_.empty())
    return;

  const SolutionBase& s = *upstream_solutions_.pop();
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene()->diff();

  std::vector<moveit_msgs::msg::Grasp> grasps = properties().get<std::vector<moveit_msgs::msg::Grasp>>("grasps");
  for (auto grasp : grasps)
  {
    InterfaceState state(scene);
    forwardProperties(*s.end(), state);  // forward registered properties from received solution

    state.properties().set("target_pose", grasp.grasp_pose);

    SubTrajectory trajectory;
    trajectory.setCost(0.0);

    rviz_marker_tools::appendFrame(trajectory.markers(), grasp.grasp_pose, 0.1, "pose frame");
    spawn(std::move(state), std::move(trajectory));
  }

}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
