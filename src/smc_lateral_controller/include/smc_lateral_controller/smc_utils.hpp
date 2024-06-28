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

/**
 * @brief Calculate the curvature of the path
 * @param trajectory The trajectory
 * @param odometry The odometry of the vehicle
 * @return The curvature
 */
double calculate_curvature(
  const Trajectory & trajectory,
  const Odometry & odometry);

/**
 * @brief Calculate the lateral error between the vehicle and the trajectory
 * @param trajectory The trajectory
 * @param odometry The odometry of the vehicle
 * @return The lateral error
 */
double calculate_lateral_error(
  const Trajectory & trajectory,
  const Odometry & odometry);

/**
 * @brief Calculate the angular error between the vehicle and the trajectory
 * @param trajectory The trajectory
 * @param odometry The odometry of the vehicle
 * @return The angular error
 */
double calculate_angular_error(
  const Trajectory & trajectory,
  const Odometry & odometry);

/**
 * @brief Update the odometry of the vehicle
 * @param odometry The current odometry
 * @param dt The time step
 * @return The updated odometry
 */
Odometry update_odometry(
  const Odometry & odometry, const double & dt);

/**
 * @brief Resample the trajectory by distance
 * @param traj The trajectory to resample
 * @param ds The distance between points
 */
void resample_traj_by_distance(
  Trajectory & traj, const double & ds);

/**
 * @brief Smooth the trajectory using a moving average filter
 * @param traj The trajectory to smooth
 * @param path_filter_moving_ave_num The number of points to average
 */
void smooth_trajectory(
  Trajectory & traj, const int & path_filter_moving_ave_num);

/**
 * @brief Get the quaternion from the yaw angle
 * @param yaw The yaw angle
 * @return The quaternion
 */
geometry_msgs::msg::Quaternion get_quaternion_from_yaw(const double yaw);

/**
 * @brief Add a lookahead distance to the odometry
 * @param odometry The odometry
 * @param lookahead_distance The lookahead distance
 * @return The updated odometry
 */
Odometry add_lookahead_distance(
  const Odometry & odometry, const double & lookahead_distance);
}  // namespace smc_utils

#endif  // SMC_UTILS_HPP_
