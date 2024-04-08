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

#include "dummy_lateral_controller/dummy_lateral_controller.hpp"

namespace autoware::motion::control::dummy_lateral_controller
{

DummyLateralController::DummyLateralController(rclcpp::Node & node) : node_{&node}
{
  m_pub_predicted_traj = node_->create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  m_pub_debug_values =
    node_->create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);
}

DummyLateralController::~DummyLateralController()
{
}

trajectory_follower::LateralOutput DummyLateralController::run(
  trajectory_follower::InputData const & input_data)
{
  Trajectory predicted_traj = input_data.current_trajectory;
  Float32MultiArrayStamped debug_values;

  publishPredictedTraj(predicted_traj);
  publishDebugValues(debug_values);

  // TODO: calculate here the control_cmd for path tracking
  trajectory_follower::LateralOutput output;
  output.control_cmd.stamp = node_->now();
  output.control_cmd.steering_tire_angle = 0.0;
  output.sync_data.is_steer_converged = true;

  return output;
}

bool DummyLateralController::isReady(const trajectory_follower::InputData & input_data)
{
  const auto unused_variable = input_data;

  return true;
}

void DummyLateralController::publishPredictedTraj(Trajectory & predicted_traj) const
{
  // TODO: calculate here the predicted trajectory and use it
  predicted_traj.header.stamp = node_->now();
  predicted_traj.header.frame_id = "base_link";
  m_pub_predicted_traj->publish(predicted_traj);
}

void DummyLateralController::publishDebugValues(Float32MultiArrayStamped & debug_values) const
{
  // TODO: calculate here the debug values and use it
  debug_values.stamp = node_->now();
  m_pub_debug_values->publish(debug_values);
}

}  // namespace autoware::motion::control::dummy_lateral_controller
