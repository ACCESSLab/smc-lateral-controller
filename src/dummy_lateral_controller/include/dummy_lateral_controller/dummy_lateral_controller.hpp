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

#ifndef DUMMY_LATERAL_CONTROLLER__DUMMY_LATERAL_CONTROLLER_HPP_
#define DUMMY_LATERAL_CONTROLLER__DUMMY_LATERAL_CONTROLLER_HPP_

#include "trajectory_follower_base/lateral_controller_base.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

namespace autoware::motion::control::dummy_lateral_controller
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
using autoware_auto_planning_msgs::msg::Trajectory;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

class DummyLateralController : public trajectory_follower::LateralControllerBase
{
public:
  explicit DummyLateralController(rclcpp::Node & node);
  virtual ~DummyLateralController();

private:
  rclcpp::Node * node_;

  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_debug_values;

  Trajectory m_current_trajectory;  // Current reference trajectory for path following.

  /**
   * @brief Check if all necessary data is received and ready to run the control.
   * @param input_data Input data required for control calculation.
   * @return True if the data is ready, false otherwise.
   */
  bool isReady(const trajectory_follower::InputData & input_data) override;

  /**
   * @brief Compute the control command for path following with a constant control period.
   * @param input_data Input data required for control calculation.
   * @return Lateral output control command.
   */
  trajectory_follower::LateralOutput run(
    trajectory_follower::InputData const & input_data) override;

  /**
   * @brief Publish the predicted future trajectory.
   * @param predicted_traj Predicted future trajectory to be published.
   */
  void publishPredictedTraj(Trajectory & predicted_traj) const;

  /**
   * @brief Publish diagnostic message.
   * @param diagnostic Diagnostic message to be published.
   */
  void publishDebugValues(Float32MultiArrayStamped & diagnostic) const;
};
}  // namespace autoware::motion::control::dummy_lateral_controller

#endif  // DUMMY_LATERAL_CONTROLLER__DUMMY_LATERAL_CONTROLLER_HPP_
