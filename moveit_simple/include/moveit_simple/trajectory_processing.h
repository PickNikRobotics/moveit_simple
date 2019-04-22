/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Henning Kayser
   Desc: Functions for validating and fixing trajectory waypoints and timestamps
*/

#ifndef TRAJECTORY_PROCESSING_H
#define TRAJECTORY_PROCESSING_H

#include <moveit_simple/exceptions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace moveit_simple
{
namespace trajectory_processing
{
struct TrajectoryValidationResult
{
  enum { Success,
         InvalidPosition,
         InvalidVelocity,
         InvalidAcceleration,
         InvalidTimestamp} value;
  std::string error_message;
};
TrajectoryValidationResult validateTrajectory(robot_model::RobotModelConstPtr robot_model,
                                              const trajectory_msgs::JointTrajectory& trajectory)
{
  TrajectoryValidationResult result;
  const std::vector<std::string>& joint_names = trajectory.joint_names;
  trajectory_msgs::JointTrajectoryPoint previous_point;
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    // validate waypoint bounds
    const auto& point = trajectory.points[i];
    for (size_t j = 0; j < joint_names.size(); ++j)
    {
      const auto& joint_model = robot_model->getJointModel(joint_names[j]);
      const auto& bounds = joint_model->getVariableBounds()[0];
      // validate position bounds and throw exception since this can't be fixed easily
      if (bounds.position_bounded_ && !joint_model->satisfiesPositionBounds(&point.positions[j]))
      {
        result.value = TrajectoryValidationResult::InvalidPosition;
        result.error_message = "Trajectory invalidates position bounds in waypoint " + std::to_string(i) + " and joint "
          + std::to_string(j);
        return result;
      }
      // validate velocity/acceleration bounds and pair-wise time difference
      double dt = (point.time_from_start - previous_point.time_from_start).toSec();
      if (bounds.velocity_bounded_)
      {
        if (bounds.max_velocity_ < std::abs(point.velocities[j]))
        {
          result.value = TrajectoryValidationResult::InvalidVelocity;
          result.error_message = "Trajectory invalidates velocity bounds in waypoint " + std::to_string(i)
            + " and joint " + std::to_string(j);
          return result;
        }
        if (i > 0 && bounds.max_velocity_ * dt < std::abs(point.positions[j] - previous_point.positions[j]))
        {
          result.value = TrajectoryValidationResult::InvalidVelocity;
          result.error_message = "Position of trajectory waypoint " + std::to_string(i)
            + " is not reachable given target time and velocity limits";
          return result;
        }
      }
      // validate acceleration bounds and pair-wise velocty difference
      if (bounds.acceleration_bounded_)
      {
        if (bounds.max_acceleration_ < std::abs(point.accelerations[j]))
        {
          result.value = TrajectoryValidationResult::InvalidAcceleration;
          result.error_message = "Trajectory invalidates acceleration bounds in waypoint " + std::to_string(i)
            + " and joint " + std::to_string(j);
          return result;
        }
        if (i > 0 && bounds.max_acceleration_ * dt < std::abs(point.velocities[j] - previous_point.velocities[j]))
        {
          result.value = TrajectoryValidationResult::InvalidAcceleration;
          result.error_message = "Velocity of trajectory waypoint " + std::to_string(i) + " and joint "
            + std::to_string(j) + " is not reachable given target time and acceleration limit";
          return result;
        }
      }
    }
    previous_point = point;
  }
  result.value = TrajectoryValidationResult::Success;
  return result;
}

bool retimeTrajectory(robot_model::RobotModelConstPtr robot_model, const std::string& group_name,
                      trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // convert trajectory message to robot trajectory
  robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);
  moveit::core::RobotState robot_state(robot_model);
  trajectory.setRobotTrajectoryMsg(robot_state, joint_trajectory);
  ::trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(trajectory))
    throw InvalidTrajectoryException("Failed to recompute trajectory time stamps");
  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory.getRobotTrajectoryMsg(trajectory_msg);
  joint_trajectory = trajectory_msg.joint_trajectory;
}

void validateTrajectory(robot_model::RobotModelConstPtr robot_model,
                        const std::string& group_name,
                        trajectory_msgs::JointTrajectory& trajectory,
                        bool retime_trajectory)
{
  TrajectoryValidationResult result = validateTrajectory(robot_model, trajectory);
  if (result.value != TrajectoryValidationResult::Success)
  {
    // we can't fix invalid positions by trajectory parameterization
    if (retime_trajectory && result.value != TrajectoryValidationResult::InvalidPosition)
      retimeTrajectory(robot_model, group_name, trajectory);
    else
      throw InvalidTrajectoryException(result.error_message);
  }
}
}  // namespace trajectory_processing
}  // namespace moveit_simple
#endif  // TRAJECTORY_PROCESSING_H
