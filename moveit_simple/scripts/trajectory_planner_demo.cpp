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

#include <ros/ros.h>

#include <moveit_simple/moveit_simple.h>
#include <moveit_simple/prettyprint.hpp>
#include <moveit_simple/online_robot.h>
#include <moveit_simple/point_types.h>

namespace moveit_simple
{
class RobotDemo : OnlineRobot
{
public:
  RobotDemo()
    : OnlineRobot(ros::NodeHandle(), "robot_description", "manipulator")
  {}

  void initializeExamplePickPlace()
  {
    // init pick pose
    Eigen::Isometry3d pick_pose(Eigen::Translation3d(Eigen::Vector3d(1.5, 0, 1)));
    pick_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));

    // init place pose
    Eigen::Isometry3d place_pose = pick_pose;

    // set robot end effector symmetry to circular

    setEndEffectorSymmetry(moveit_simple::EndEffectorSymmetry::Circular);

    // compute pick/place pose
    std::vector<double> seed, pick_state, place_state;
    bool success = getPickPlaceJointSolutions(pick_pose, place_pose, 1.0, seed, pick_state, place_state);


    std::unique_ptr<moveit_simple::TrajectoryPoint> pick_point1 =
       std::unique_ptr<moveit_simple::TrajectoryPoint>
       (new moveit_simple::JointTrajectoryPoint(pick_state, 2.0, "pick_point1"));

    std::unique_ptr<moveit_simple::TrajectoryPoint> place_point1 =
       std::unique_ptr<moveit_simple::TrajectoryPoint>
       (new moveit_simple::JointTrajectoryPoint(place_state, 2.0, "place_point2"));



    // move place pose and compute pick/place
    place_pose *= Eigen::Translation3d(Eigen::Vector3d(0, 0.2, 0));
    getPickPlaceJointSolutions(pick_pose, place_pose, 1.0, seed, pick_state, place_state);

    std::unique_ptr<moveit_simple::TrajectoryPoint> pick_point2 =
       std::unique_ptr<moveit_simple::TrajectoryPoint>
       (new moveit_simple::JointTrajectoryPoint(pick_state, 10.0, "pick_point2"));

    std::unique_ptr<moveit_simple::TrajectoryPoint> place_point2 =
       std::unique_ptr<moveit_simple::TrajectoryPoint>
       (new moveit_simple::JointTrajectoryPoint(place_state, 10.0, "place_point2"));

    // rotate place pose and compute pick/place
    place_pose *= Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
    getPickPlaceJointSolutions(pick_pose, place_pose, 1.0, seed, pick_state, place_state);


    std::unique_ptr<moveit_simple::TrajectoryPoint> pick_point3 =
       std::unique_ptr<moveit_simple::TrajectoryPoint>
       (new moveit_simple::JointTrajectoryPoint(pick_state, 10.0, "pick_point3"));

    std::unique_ptr<moveit_simple::TrajectoryPoint> place_point3 =
       std::unique_ptr<moveit_simple::TrajectoryPoint>
       (new moveit_simple::JointTrajectoryPoint(place_state, 10.0, "place_point3"));


    ROS_INFO_STREAM("Adding pick and place to a trajectory and executing");
    int time = 10;
    int time_inc = 10;
    addTrajPoint(pick_place_trajectory_name, "home",      time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, pick_point1, joint, time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, place_point1, joint, time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, "home",      time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, pick_point2, joint, time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, place_point2, joint, time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, "home",      time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, pick_point3, joint, time);
    time += time_inc;
    addTrajPoint(pick_place_trajectory_name, place_point3, joint, time);
    time += time_inc;
    pick_place_trajectory_initialized = true;
  }

  void executeExamplePickPlace()
  {
    if (!pick_place_trajectory_initialized)
      execute(pick_place_trajectory_name);
  }

  std::unique_ptr<moveit_simple::OnlineRobot> robot;
  moveit_simple::InterpolationType cart = moveit_simple::interpolation_type::CARTESIAN;
  moveit_simple::InterpolationType joint = moveit_simple::interpolation_type::JOINT;
  std::string pick_place_trajectory_name = "pick_place_trajectory_example";
  bool pick_place_trajectory_initialized = false;
};
}

int main(int argc, char **argv)
{

  std::string name = "bin_pick_server_main";
  ros::init(argc, argv, name);

  moveit_simple::RobotDemo demo;
  demo.initializeExamplePickPlace();
  demo.executeExamplePickPlace();
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
