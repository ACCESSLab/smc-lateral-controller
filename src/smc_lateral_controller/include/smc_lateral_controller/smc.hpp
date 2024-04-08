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
    double gamma;
  };
  Params m_params;  // sliding mode control parameters

  // state variables
  double m_velocity;
  double m_steer_lim;
  double m_ctrl_period;
  double m_decay_factor;
  double m_u2_lat_prev;
  double m_u2_yaw_prev;
  double m_ctrl_cmd_prev;
  int m_n_pred;

  // error variables
  double m_e_lat;
  double m_e_yaw;
  double m_e_lat_dot;
  double m_e_yaw_dot;

public:
  /**
   * @brief constructor
   *
   * @param steer_lim The steering limit in radians.
   * @param ctrl_period The control period in seconds.
   * @param decay_factor The decay factor for the controller.
   * @param n_pred The number of prediction steps.
   */
  SMC(
    const double & steer_lim,
    const double & ctrl_period,
    const double & decay_factor,
    const int & n_pred)
  : m_steer_lim(steer_lim),
    m_ctrl_period(ctrl_period),
    m_decay_factor(decay_factor),
    m_n_pred(n_pred)
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
   * @brief Sets the gains for the lateral and yaw controllers.
   *
   * @param lambda The gain for lateral error control.
   * @param alpha The gain for error control.
   * @param beta The gain for derivative control.
   * @param gamma The gain for control input.
   */
  void setGains(
    const double & lambda,
    const double & alpha,
    const double & beta,
    const double & gamma);

  /**
   * @brief Calculates the desired control output based on the predicted state.
   *
   * This function takes in a predicted state matrix and calculates the desired control output
   * using a specific algorithm.
   *
   * @param x_pred The predicted state matrix.
   * @return The calculated control output.
   */
  Eigen::VectorXd calculate(const Eigen::MatrixXd & x_pred);

  /**
   * @brief Calculates the prediction vector for the given input matrix.
   *
   * This function takes an input matrix x_pred and returns a prediction vector.
   * The prediction vector is calculated using Eigen::VectorXd.
   *
   * @param x_pred The input matrix for which the prediction vector is calculated.
   */
  void getPrediction(const Eigen::MatrixXd & x_pred);

  /**
   * @brief Applies the Super Twisting Algorithm (STA) to calculate the control input.
   *
   * This function implements the Super Twisting Algorithm (STA) for lateral control.
   * It takes the error (e), error derivative (e_dot), and the previous control
   * input (u2_prev) as inputs and returns the control input as a vector of doubles.
   *
   * @param e The error between the desired and actual lateral position.
   * @param e_dot The derivative of the error.
   * @param u2_prev The previous control input.
   * @param lambda The gain for lateral error control.
   * @param alpha The gain for error control.
   * @param beta The gain for derivative control.
   * @return The control input as a vector of doubles.
   */
  Eigen::VectorXd superTwistingAlgorithm(
    const double & e, const double & e_dot, const double & u2_prev,
    const double & lambda, const double & alpha, const double & beta);
};
}  // namespace autoware::motion::control::smc_lateral_controller
#endif  // SMC_LATERAL_CONTROLLER__SMC_HPP_
