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

#ifndef SMC_LATERAL_CONTROLLER__SMC_HPP_
#define SMC_LATERAL_CONTROLLER__SMC_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace autoware::motion::control::smc_lateral_controller
{
class SMC
{
private:
  struct Params
  {
    double lambda;
    double alpha;
    double beta;
    double phi;
  };
  Params m_params;  // sliding mode control parameters

  // state variables
  double m_velocity;
  double m_curvature;
  double m_steer_lim;
  double m_steer_rate_lim;
  double m_ctrl_period;
  double m_decay_speed;
  int m_n_pred;

  // control variables
  Eigen::VectorXd m_u;
  double m_u1_lat;
  double m_u1_yaw;
  double m_u2_lat;
  double m_u2_yaw;

  // error variables
  Eigen::VectorXd m_e_lat;
  Eigen::VectorXd m_e_yaw;

public:
  /**
   * @brief constructor
   *
   * @param steer_lim The steering limit in radians.
   * @param steer_rate_lim The steering rate limit in radians per second.
   * @param ctrl_period The control period in seconds.
   * @param decay_speed The decay speed near zero speed.
   * @param n_pred The number of prediction steps.
   */
  SMC(
    const double & steer_lim,
    const double & steer_rate_lim,
    const double & ctrl_period,
    const double & decay_speed,
    const int & n_pred)
  : m_steer_lim(steer_lim),
    m_steer_rate_lim(steer_rate_lim),
    m_ctrl_period(ctrl_period),
    m_decay_speed(decay_speed),
    m_n_pred(n_pred - 1),
    m_u(Eigen::VectorXd::Zero(2)),
    m_e_lat(Eigen::VectorXd::Zero(2)),
    m_e_yaw(Eigen::VectorXd::Zero(2))
  {
    // Initialize other member variables if needed
  }

  /**
   * @brief Sets the velocity for the lateral controller.
   *
   * @param velocity The current velocity in meters per second.
   */
  void setVelocity(const double & velocity);

  /**
   * @brief Sets the curvature for the lateral controller.
   *
   * @param curvature The current curvature in radians per meter.
   */
  void setCurvature(const double & curvature);

  /**
   * @brief Sets the gains for the lateral and yaw controllers.
   *
   * @param lambda The gain for error control.
   * @param alpha The gain for error control.
   * @param beta The gain for error derivative control.
   * @param phi The gain for control input.
   */
  void setGains(
    const double & lambda,
    const double & alpha,
    const double & beta,
    const double & phi);

  /**
   * @brief Calculates the control command.
   *
   * @param is_controller_active The controller activation flag.
   * @param u_curr The current steering tire angle
   *
   * @return The control command.
   */
  Eigen::VectorXd calculate(
    const bool & is_controller_active, const double & u_curr);

  /**
   * @brief Sets the prediction vector for the given input matrix.
   *
   * @param x_pred The input matrix for which the prediction vector is calculated.
   */
  void setPrediction(const Eigen::MatrixXd & x_pred);

  /**
   * @brief Applies the Super Twisting Algorithm (STA) to calculate the control input.
   *
   * @param e The error vector.
   * @param u1 The first control input.
   * @param u2 The second control input.
   * @param lambda The gain for error control.
   * @param alpha The gain for error control.
   * @param beta The gain for error derivative control.
   * @param is_controller_active The controller activation flag.
   */
  void superTwistingAlgorithm(
    const Eigen::VectorXd & e, double & u1, double & u2,
    const double & lambda, const double & alpha, const double & beta,
    const bool & is_controller_active);

  /**
   * @brief Limit the control command and rate.
   *
   * @return The control command and rate are updated.
   */
  void limitSteeringAngle(Eigen::VectorXd & u);
};
}  // namespace autoware::motion::control::smc_lateral_controller
#endif  // SMC_LATERAL_CONTROLLER__SMC_HPP_
