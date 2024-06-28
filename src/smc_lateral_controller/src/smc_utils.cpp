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

#include "smc_lateral_controller/smc_utils.hpp"

namespace smc_utils
{
double calculate_curvature(
  const Trajectory & trajectory, const Odometry & odometry)
{
  // Find the nearest segment on the trajectory to the current position of the vehicle
  size_t nearest_idx = motion_utils::findNearestSegmentIndex(
    trajectory.points, odometry.pose.pose.position);

  // Get the three nearest poses on the trajectory
  const auto & prev_pose = trajectory.points.at(nearest_idx - 1).pose;
  const auto & curr_pose = trajectory.points.at(nearest_idx).pose;
  const auto & next_pose = trajectory.points.at(nearest_idx + 1).pose;

  // Calculate the curvature of the path using the three nearest poses
  double curvature;
  try {
    curvature = tier4_autoware_utils::calcCurvature(
      prev_pose.position, curr_pose.position, next_pose.position);
  } catch (std::exception const & e) {
    // ...code that handles the error...
    RCLCPP_WARN(rclcpp::get_logger("sliding_mode_control"), "%s", e.what());
    curvature = 0.0;
  }

  return curvature;
}

// Finds the two nearest points on the trajectory to the current position
double calculate_lateral_error(
  const Trajectory & trajectory, const Odometry & odometry)
{
  double lateral_error = motion_utils::calcLateralOffset(
    trajectory.points, odometry.pose.pose.position);

  return lateral_error;
}

// Finds the angular error between the vehicle orientation and the nearest line
double calculate_angular_error(
  const Trajectory & trajectory, const Odometry & odometry)
{
  // Find the nearest segment on the trajectory to the current position of the vehicle
  size_t nearest_idx = motion_utils::findNearestSegmentIndex(
    trajectory.points, odometry.pose.pose.position);
  
  // Calculate the angular error between the vehicle orientation and the nearest line
  double angular_error = tier4_autoware_utils::normalizeRadian(
    tf2::getYaw(odometry.pose.pose.orientation) -
    tf2::getYaw(trajectory.points[nearest_idx].pose.orientation));

  return angular_error;
}

// update the odometry with the current control input
Odometry update_odometry(
  const Odometry & odometry, const double & dt)
{
  // initial states kinematic
  double x = odometry.pose.pose.position.x;
  double y = odometry.pose.pose.position.y;
  double yaw = tf2::getYaw(odometry.pose.pose.orientation);
  double vx = odometry.twist.twist.linear.x;

  // Calculate the new position of the vehicle
  x += vx * dt * std::cos(yaw);
  y += vx * dt * std::sin(yaw);

  // update the odometry
  Odometry updated_odometry = odometry;
  updated_odometry.pose.pose.position.x = x;
  updated_odometry.pose.pose.position.y = y;
  updated_odometry.pose.pose.orientation =
    get_quaternion_from_yaw(yaw);

  return updated_odometry;
}

// Resample the trajectory by distance
void resample_traj_by_distance(
  Trajectory & traj, const double & ds)
{
  // Interpolate with constant interval distance.
  std::vector<double> out_arclength;
  const auto input_tp_array = motion_utils::convertToTrajectoryPointArray(traj);
  const auto traj_length = motion_utils::calcArcLength(input_tp_array);
  for (double s = 0; s < traj_length; s += ds) {
    out_arclength.push_back(s);
  }

  traj = motion_utils::resampleTrajectory(
    motion_utils::convertToTrajectory(input_tp_array), out_arclength);
}

// Smooth the trajectory using a moving average filter
void smooth_trajectory(
  Trajectory & traj, const int & path_filter_moving_ave_num)
{
  if (static_cast<int>(traj.points.size()) <= 2 * path_filter_moving_ave_num) {
    // Create a logger instance
    auto logger = rclcpp::get_logger("smooth_trajectory");

    // Use the logger
    RCLCPP_ERROR(logger, "Cannot smooth path! Trajectory size is too low!");
  }

  Trajectory trajectory_smoothed(traj);

  for (int64_t i = 0; i < static_cast<int64_t>(traj.points.size()); ++i) {
    TrajectoryPoint tmp{};
    int64_t num_tmp = path_filter_moving_ave_num;
    int64_t count = 0;
    // double yaw = 0.0;
    if (i - num_tmp < 0) {
      num_tmp = i;
    }
    if (i + num_tmp > static_cast<int64_t>(traj.points.size()) - 1) {
      num_tmp = static_cast<int64_t>(traj.points.size()) - i - 1;
    }
    for (int64_t j = -num_tmp; j <= num_tmp; ++j) {
      const auto & p = traj.points.at(static_cast<size_t>(i + j));

      tmp.pose.position.x += p.pose.position.x;
      tmp.pose.position.y += p.pose.position.y;
      tmp.pose.position.z += p.pose.position.z;
      tmp.longitudinal_velocity_mps += p.longitudinal_velocity_mps;
      tmp.acceleration_mps2 += p.acceleration_mps2;
      tmp.front_wheel_angle_rad += p.front_wheel_angle_rad;
      tmp.heading_rate_rps += p.heading_rate_rps;
      tmp.lateral_velocity_mps += p.lateral_velocity_mps;
      tmp.rear_wheel_angle_rad += p.rear_wheel_angle_rad;
      // yaw += tf2::getYaw(p.pose.orientation);
      ++count;
    }
    auto & p = trajectory_smoothed.points.at(static_cast<size_t>(i));

    p.pose.position.x = tmp.pose.position.x / count;
    p.pose.position.y = tmp.pose.position.y / count;
    p.pose.position.z = tmp.pose.position.z / count;
    p.longitudinal_velocity_mps = tmp.longitudinal_velocity_mps / count;
    p.acceleration_mps2 = tmp.acceleration_mps2 / count;
    p.front_wheel_angle_rad = tmp.front_wheel_angle_rad / count;
    p.heading_rate_rps = tmp.heading_rate_rps / count;
    p.lateral_velocity_mps = tmp.lateral_velocity_mps / count;
    p.rear_wheel_angle_rad = tmp.rear_wheel_angle_rad / count;
    p.pose.orientation = traj.points.at(i).pose.orientation;
    // p.pose.orientation = get_quaternion_from_yaw(yaw / count);
  }

  traj = trajectory_smoothed;
}

// Get the quaternion from the yaw angle
geometry_msgs::msg::Quaternion get_quaternion_from_yaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

// Add lookahead distance to odometry
Odometry add_lookahead_distance(
  const Odometry & odometry, const double & lookahead_distance)
{
  // Get the current position and orientation of the vehicle
  double x = odometry.pose.pose.position.x;
  double y = odometry.pose.pose.position.y;
  double yaw = tf2::getYaw(odometry.pose.pose.orientation);

  // Calculate the new position of the vehicle
  x += lookahead_distance * std::cos(yaw);
  y += lookahead_distance * std::sin(yaw);

  // Update the odometry
  Odometry updated_odometry = odometry;
  updated_odometry.pose.pose.position.x = x;
  updated_odometry.pose.pose.position.y = y;

  return updated_odometry;
}
}  // namespace smc_utils
