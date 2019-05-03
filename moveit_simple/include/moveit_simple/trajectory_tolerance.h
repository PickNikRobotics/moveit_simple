/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019 Plus One Robotics
 * Copyright (c) 2019 PickNik
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

#ifndef TRAJECTORY_TOLERANCE_H
#define TRAJECTORY_TOLERANCE_H

namespace moveit_simple
{

/**@brief TrajectoryTolerances stores the orientation and positional
 *        tolerance information for a waypoint. The Orientation uses the
 *        right hand rule and operates by rotating goal pose in the world
 *        frame (in the order x,y,z when generating pose candidates)
 */
struct TrajectoryTolerance
{
    TrajectoryTolerance(double position_increment = 0, double position_tolerance_x = 0,
                             double position_tolerance_y = 0, double position_tolerance_z = 0,
                             double orientation_increment = 0, double oreintation_tolerance_x = 0,
                             double oreintation_tolerance_y = 0, double oreintation_tolerance_z = 0)
        : position_increment_(position_increment)
        , position_tolerance_x_(position_tolerance_x)
        , position_tolerance_y_(position_tolerance_y)
        , position_tolerance_z_(position_tolerance_z)
        , orientation_increment_(orientation_increment)
        , oreintation_tolerance_x_(oreintation_tolerance_x)
        , oreintation_tolerance_y_(oreintation_tolerance_y)
        , oreintation_tolerance_z_(oreintation_tolerance_z){};

    TrajectoryTolerance(const TrajectoryTolerance& other)
    {
        this->position_increment_ = other.position_increment_;
        this->position_tolerance_x_ = other.position_tolerance_x_;
        this->position_tolerance_y_ = other.position_tolerance_y_;
        this->position_tolerance_z_ = other.position_tolerance_z_;
        this->orientation_increment_ = other.orientation_increment_;
        this->oreintation_tolerance_x_ = other.oreintation_tolerance_x_;
        this->oreintation_tolerance_y_ = other.oreintation_tolerance_y_;
        this->oreintation_tolerance_z_ = other.oreintation_tolerance_z_;
    }

    double position_increment_;
    double position_tolerance_x_;
    double position_tolerance_y_;
    double position_tolerance_z_;
    double orientation_increment_;
    double oreintation_tolerance_x_;
    double oreintation_tolerance_y_;
    double oreintation_tolerance_z_;
};

} // namespace moveit_simple

#endif // TRAJECTORY_TOLERANCE_H
