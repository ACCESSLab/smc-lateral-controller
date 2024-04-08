// Copyright 2023 North Carolina A & T State University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMC_UTILS_HPP_
#define SMC_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tuple>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include <limits>
#include <deque>
#include <utility>
#include <vector>

#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>


namespace smc_utils
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Vector3;
// Define a line segment that connects the three nearest points
std::tuple<Pose, Pose, Pose> calculate_nearest_poses(
  const Trajectory & trajectory, const Odometry & odometry);

// Calculate the curvature of the path given three points
double calculate_curvature(
  const Trajectory & trajectory,
  const Odometry & odometry);

// Normalize the angle to the range [-pi, pi]
double normalize_angle(const double & angle);

// Finds the two nearest points on the trajectory to the current position
double calculate_lateral_error(
  const Trajectory & trajectory,
  const Odometry & odometry);

// Finds the angular error between the vehicle orientation and the nearest line
double calculate_angular_error(
  const Trajectory & trajectory,
  const Odometry & odometry);

// update the odometry with the current state
Odometry update_odometry(
  const Odometry & odometry, const Eigen::VectorXd & x_curr,
  const double & velocity, const double & dt);

// Resample the trajectory to have a constant distance between points
Trajectory resample_traj_by_distance(
  const Trajectory & trajectory,
  const double & ds);

// Smooth the trajectory using a moving average
Trajectory smooth_trajectory(
  const Trajectory & trajectory, const int & path_filter_moving_ave_num);

// Calculate the distance between two points
geometry_msgs::msg::Quaternion get_quaternion_from_yaw(const double yaw);

}  // namespace smc_utils

#endif  // SMC_UTILS_HPP_
