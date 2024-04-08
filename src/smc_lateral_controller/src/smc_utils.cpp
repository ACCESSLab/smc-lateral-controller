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
std::tuple<Pose, Pose, Pose> calculate_nearest_poses(
  const Trajectory & trajectory, const Odometry & odometry)
{
  // Find the three nearest points on the trajectory to the current position of the vehicle
  std::array<std::tuple<double, Pose, size_t>, 3> nearest_poses;
  // Initialize the nearest points with the maximum distance
  std::fill(
    nearest_poses.begin(), nearest_poses.end(),
    std::make_tuple(std::numeric_limits<double>::max(), Pose{}, 0));

  double prev_dist = 0.0;  // previous distance
  for (auto & nearest : nearest_poses) {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & point = trajectory.points[i];
      double dx = point.pose.position.x - odometry.pose.pose.position.x;
      double dy = point.pose.position.y - odometry.pose.pose.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < std::get<0>(nearest) && distance > prev_dist) {
        std::get<0>(nearest) = distance;
        std::get<1>(nearest) = point.pose;
        std::get<2>(nearest) = i;
      }
    }
    prev_dist = std::get<0>(nearest);
  }

  // Sort the nearest points by distance
  std::sort(
    nearest_poses.begin(), nearest_poses.end(),
    [](const auto & a, const auto & b) {
      return std::get<2>(a) < std::get<2>(b);
    });

  // size_t prev_idx = std::get<2>(nearest_poses[0]);
  // size_t curr_idx = std::get<2>(nearest_poses[1]);
  // size_t next_idx = std::get<2>(nearest_poses[2]);

  // RCLCPP_WARN(rclcpp::get_logger("num_points:"), "%ld", trajectory.points.size());
  // RCLCPP_WARN(rclcpp::get_logger("index:"), "%ld, %ld, %ld", prev_idx, curr_idx, next_idx);
  // RCLCPP_WARN(rclcpp::get_logger("yaw:"), "%f, %f, %f",
  //   tf2::getYaw(trajectory.points[prev_idx].pose.orientation),
  //   tf2::getYaw(trajectory.points[curr_idx].pose.orientation),
  //   tf2::getYaw(trajectory.points[next_idx].pose.orientation));

  // Return the three nearest points
  const Pose & prev_pose = std::get<1>(nearest_poses[0]);
  const Pose & curr_pose = std::get<1>(nearest_poses[1]);
  const Pose & next_pose = std::get<1>(nearest_poses[2]);

  return std::make_tuple(prev_pose, curr_pose, next_pose);
}

double calculate_curvature(
  const Trajectory & trajectory, const Odometry & odometry)
{
  // Get the three nearest points on the trajectory
  Pose prev_pose, curr_pose, next_pose;
  std::tie(
    prev_pose, curr_pose, next_pose) = calculate_nearest_poses(
    trajectory, odometry);

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

// Normalize the angle to the range [-pi, pi]
double normalize_angle(const double & angle)
{
  double normalized_angle = angle;

  // Normalize the angle to the range [-pi, pi]
  if (normalized_angle > M_PI) {
    normalized_angle -= 2 * M_PI;
  } else if (normalized_angle < -M_PI) {
    normalized_angle += 2 * M_PI;
  }

  return normalized_angle;
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
  // Get the three nearest points on the trajectory
  Pose prev_pose, curr_pose, next_pose;
  std::tie(
    prev_pose, curr_pose, next_pose) = calculate_nearest_poses(
    trajectory, odometry);

  // Calculate the angle between the vehicle orientation vector and
  // the line segment vector using the atan2 function
  double yaw_path = tf2::getYaw(curr_pose.orientation);
  double yaw = tf2::getYaw(odometry.pose.pose.orientation);
  double angular_error = normalize_angle(yaw - yaw_path);

  return angular_error;
}

// update the odometry with the current state
Odometry update_odometry(
  const Odometry & odometry, const Eigen::VectorXd & x_curr,
  const double & velocity, const double & dt)
{
  // initial states kinematic
  double x = odometry.pose.pose.position.x;
  double y = odometry.pose.pose.position.y;
  double yaw = tf2::getYaw(odometry.pose.pose.orientation);

  // angular error
  double e_yaw = x_curr(2);

  // Calculate the new position of the vehicle
  yaw += e_yaw;
  x += std::cos(yaw) * velocity * dt;
  y += std::sin(yaw) * velocity * dt;

  // update the odometry
  Odometry updated_odometry = odometry;
  updated_odometry.pose.pose.position.x = x;
  updated_odometry.pose.pose.position.y = y;
  updated_odometry.pose.pose.orientation = tf2::toMsg(
    tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));

  return updated_odometry;
}

// Resample the trajectory with a constant interval distance
Trajectory resample_traj_by_distance(
  const Trajectory & trajectory, const double & ds)
{
  // Interpolate with constant interval distance.
  std::vector<double> out_arclength;
  const auto input_tp_array = motion_utils::convertToTrajectoryPointArray(trajectory);
  const auto traj_length = motion_utils::calcArcLength(input_tp_array);
  for (double s = 0; s < traj_length; s += ds) {
    out_arclength.push_back(s);
  }

  Trajectory trajectory_resampled =
    motion_utils::resampleTrajectory(
    motion_utils::convertToTrajectory(input_tp_array), out_arclength);
  trajectory_resampled.points.back() = trajectory.points.back();
  trajectory_resampled.header = trajectory.header;

  return trajectory_resampled;
}

// Smooth the trajectory with a moving average filter
Trajectory smooth_trajectory(
  const Trajectory & u, const int & path_filter_moving_ave_num)
{
  if (static_cast<int>(u.points.size()) <= 2 * path_filter_moving_ave_num) {
    // Create a logger instance
    auto logger = rclcpp::get_logger("smooth_trajectory");

    // Use the logger
    RCLCPP_ERROR(logger, "Cannot smooth path! Trajectory size is too low!");

    return u;
  }

  Trajectory trajectory_smoothed(u);

  for (int64_t i = 0; i < static_cast<int64_t>(u.points.size()); ++i) {
    TrajectoryPoint tmp{};
    int64_t num_tmp = path_filter_moving_ave_num;
    int64_t count = 0;
    double yaw = 0.0;
    if (i - num_tmp < 0) {
      num_tmp = i;
    }
    if (i + num_tmp > static_cast<int64_t>(u.points.size()) - 1) {
      num_tmp = static_cast<int64_t>(u.points.size()) - i - 1;
    }
    for (int64_t j = -num_tmp; j <= num_tmp; ++j) {
      const auto & p = u.points.at(static_cast<size_t>(i + j));

      tmp.pose.position.x += p.pose.position.x;
      tmp.pose.position.y += p.pose.position.y;
      tmp.pose.position.z += p.pose.position.z;
      tmp.longitudinal_velocity_mps += p.longitudinal_velocity_mps;
      tmp.acceleration_mps2 += p.acceleration_mps2;
      tmp.front_wheel_angle_rad += p.front_wheel_angle_rad;
      tmp.heading_rate_rps += p.heading_rate_rps;
      yaw += tf2::getYaw(p.pose.orientation);
      tmp.lateral_velocity_mps += p.lateral_velocity_mps;
      tmp.rear_wheel_angle_rad += p.rear_wheel_angle_rad;
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
    p.pose.orientation = get_quaternion_from_yaw(yaw / count);
  }
  return trajectory_smoothed;
}

geometry_msgs::msg::Quaternion get_quaternion_from_yaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

}  // namespace smc_utils
