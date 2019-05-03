/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019 Plus One Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 #include <moveit_simple/taylor_trajectory_planner.h>
#include <moveit_simple/conversions.h>
#include <moveit_simple/robot.h>

namespace moveit_simple {

bool TrajectoryPlanner::cartesianInterpolation(Robot& robot, const std::unique_ptr<TrajectoryPoint>& traj_point,
                                   std::vector<trajectory_msgs::JointTrajectoryPoint>& points, unsigned int num_steps,
                                   bool collision_check)
{
  return false;
}


} // namespace moveit_simple
