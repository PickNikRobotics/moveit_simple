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

#ifndef TAYLOR_TRAJECTORY_PLANNER_H
#define TAYLOR_TRAJECTORY_PLANNER_H

#include <mutex>

#include <ros/ros.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit_simple/point_types.h>
#include <moveit_simple/exceptions.h>
#include <moveit_simple/trajectory_planner.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace moveit_simple {

/**
 * @brief TaylorTrajectoryPlanner manages the generation, lookup and planning of trajectories using Taylor.
 * Assumptions are:
 *  - single arm manipulator (one joint group)
 *  - all cartesian poses are of the tool of the robot (could be fixed in the future)
 *  - point names are stored as joint positions in the SRDF or frame names in the
 * URDF
 * - frame transformations outside the URDF are provided by TF
 */

class TaylorTrajectoryPlanner : public TrajectoryPlanner {
public:
  /**
  * @brief Constructor
  */
  TaylorTrajectoryPlanner()
  : TrajectoryPlanner()
  , default_trajectory_tolerance_(TrajectoryTolerance())
  {}

  /**
   * @brief Destructor
   */
  ~TaylorTrajectoryPlanner() {}

  bool setDefaultTrajectoryTolerance(const TrajectoryTolerance& trajectory_tolerance)
  {
    this->default_trajectory_tolerance_ = trajectory_tolerance;
  }

  void addTrajPoint(const std::string &traj_name, std::unique_ptr<TrajectoryPoint> &point,
    const InterpolationType &type = interpolation_type::JOINT,
    const unsigned int num_steps = 0) override
  {
    addTrajPoint(traj_name, point, default_trajectory_tolerance_, type, num_steps);
  }

  void addTrajPoint(const std::string &traj_name, std::unique_ptr<TrajectoryPoint> &point,
    const TrajectoryTolerance &trajectory_tolerance,
    const InterpolationType &type = interpolation_type::JOINT,
    const unsigned int num_steps = 0);
  {
    std::lock_guard<std::recursive_mutex> guard(trajectory_info_map_mutex_);
    traj_info_map_[traj_name].push_back({ std::move(point), type, num_steps, trajectory_tolerance });
  }

  /**
   * @brief  cartesianInterpolation - Cartesian Interpolation from last added point to
   * current trajectory point(traj_point).
   * @param robot - Robot to be used for planning
   * @param traj_point: target traj_point for cartesian interpolation
   * @param points: Vector of Joint Trajectory Point to be executed
   * @param num_steps: number of steps to be interpolated between current point and traj_point
   * @param collision_check - bool to turn check for collision on\off
   * @return true if all the points including traj_point are added to the points.
   */
  virtual bool cartesianInterpolation(Robot& robot,
    const std::unique_ptr<TrajectoryPoint> &traj_point,
    std::vector<trajectory_msgs::JointTrajectoryPoint> &points, const unsigned int num_steps,
    bool collision_check = false);


private:
  TrajectoryTolerance default_trajectory_tolerance_;
};
} // namespace moveit_simple
#endif // TAYLOR_TRAJECTORY_PLANNER_H
