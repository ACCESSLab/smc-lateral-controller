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

#ifndef SMC_LATERAL_CONTROLLER__SMC_LATERAL_CONTROLLER_HPP_
#define SMC_LATERAL_CONTROLLER__SMC_LATERAL_CONTROLLER_HPP_

#include "smc_lateral_controller/smc_utils.hpp"
#include "smc_lateral_controller/lowpass_filter.hpp"
#include "smc_lateral_controller/vehicle_model/kinematic.hpp"
#include "smc_lateral_controller/vehicle_model/dynamic.hpp"
#include "smc_lateral_controller/vehicle_model/interface.hpp"
#include "smc_lateral_controller/smc.hpp"

#include "rclcpp/rclcpp.hpp"
#include "trajectory_follower_base/lateral_controller_base.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#include <string>
#include <memory>
#include <vector>

namespace autoware::motion::control::smc_lateral_controller
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
using autoware_auto_planning_msgs::msg::Trajectory;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using nav_msgs::msg::Odometry;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_adapi_v1_msgs::msg::OperationModeState;

class SmcLateralController : public trajectory_follower::LateralControllerBase
{
public:
  explicit SmcLateralController(rclcpp::Node & node);
  virtual ~SmcLateralController();

private:
  // ROS2 node
  rclcpp::Node * node_;

  // Current data
  Trajectory m_current_trajectory;
  Odometry m_current_odometry;
  SteeringReport m_current_steering;
  OperationModeState m_current_operation_mode;

  // Lowpass filters
  Butterworth2dFilter m_lpf_cmd;    // Lowpass filter for smoothing the control command.
  Butterworth2dFilter m_lpf_e_lat;  // Lowpass filter for smoothing the lateral error.
  Butterworth2dFilter m_lpf_e_yaw;  // Lowpass filter for smoothing the heading error.
  std::vector<Butterworth2dFilter> m_lpf_k;      // Lowpass filter for smoothing the curvature.

  // Pointers for vehicle model and SMC controller
  std::shared_ptr<Interface> m_vehicle_model_ptr;
  std::unique_ptr<SMC> m_smc_ptr;

  // Publishers
  rclcpp::Publisher<Trajectory>::SharedPtr pub_predicted_traj_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_values_;

  // Callbacks
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // Parameters
  Eigen::VectorXd m_x_prev;
  double m_ctrl_period;
  double m_converged_steer_rad;
  double m_traj_resample_dist;
  bool m_enable_path_smoothing;
  int m_path_filter_moving_ave_num;
  int m_n_pred;
  
  /**
   * @brief Create the vehicle model based on the provided parameters.
   * @param wheelbase Vehicle's wheelbase.
   * @param steer_lim Steering command limit.
   * @return Pointer to the created vehicle model.
   */
  std::shared_ptr<Interface> createVehicleModel(
    const double wheelbase, const double steer_lim);

  /**
   * @brief Initialize the SMC controller.
   * @param steer_lim Steering command limit.
   * @param steer_rate_lim Steering rate limit.
   * @param decay_speed Decay speed near zero speed.
   * @param n_pred Number of prediction steps.
   * @return Pointer to the initialized SMC controller.
   */
  std::unique_ptr<SMC> initializeSMC(
    const double steer_lim, const double steer_rate_lim, 
    const double decay_speed, const int n_pred);

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
   * @param odom_pred Predicted odometry.
   */
  void publishPredictedTraj(const std::vector<Odometry> & odom_pred) const;

  /**
   * @brief Publish diagnostic message.
   * @param diagnostic Diagnostic message to be published.
   */
  void publishDebugValues(Float32MultiArrayStamped & diagnostic) const;

  /**
   * @brief Updates the states of the lateral controller.
   *
   * This function takes in a trajectory and odometry information, along with the previous lateral and yaw errors,
   * and updates the states of the lateral controller. It returns a vector of updated states.
   *
   * @return A vector of updated states.
   */
  Eigen::VectorXd updateStates();

  /**
   * @brief Calculate the state predictions.
   * @param [in] x_curr The initial state vector.
   * @param [in] u_steer The current steering angle.
   * @param [in] velocity The current ego velocity.
   * @return The predicted states. 
   */
  std::tuple<Eigen::MatrixXd, std::vector<Odometry>, std::vector<double>, std::vector<double>> calculatePredictions(
    const Eigen::VectorXd & x_curr, const double & u_steer, const double & velocity);

  /**
   * @brief Initialize the lowpass filters.
   * @param lpf_cutoff_hz Cutoff frequency for lowpass filter.
   * @param cmd_lpf_cutoff_hz Cutoff frequency for lowpass filter of control command.
   */
  inline void initializeFilters(const double lpf_cutoff_hz, const double cmd_lpf_cutoff_hz)
  {
    m_lpf_cmd.initialize(m_ctrl_period, cmd_lpf_cutoff_hz);
    m_lpf_e_lat.initialize(m_ctrl_period, lpf_cutoff_hz);
    m_lpf_e_yaw.initialize(m_ctrl_period, lpf_cutoff_hz);
    m_lpf_k.resize(m_n_pred);
    for (int i = 0; i < m_n_pred; ++i) {
      m_lpf_k[i].initialize(m_ctrl_period, cmd_lpf_cutoff_hz);
    }
  }

  /**
   * @brief Check steering convergence
   * @param steer_cmd The current steering command.
   * @return True if the steering command is converged, false otherwise.
   */
  bool isSteerConverged(const double & steer_cmd);
};
}  // namespace autoware::motion::control::smc_lateral_controller
#endif  // SMC_LATERAL_CONTROLLER__SMC_LATERAL_CONTROLLER_HPP_
