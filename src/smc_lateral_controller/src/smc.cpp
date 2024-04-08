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

#include "smc_lateral_controller/smc.hpp"

namespace autoware::motion::control::smc_lateral_controller
{

// The setGains function updates the SMC parameters with the provided values.
void SMC::setGains(
  const double & lambda,
  const double & alpha,
  const double & beta,
  const double & gamma
)
{
  m_params.lambda = lambda;
  m_params.alpha = alpha;
  m_params.beta = beta;
  m_params.gamma = gamma;
}

// The calculate function computes the control command and its rate from 'x_pred'
// using the Super Twisting Algorithm. It returns a vector containing the control
// command, its rate, and the control vectors 'u_lat_12' and 'u_yaw_12'.
Eigen::VectorXd SMC::calculate(const Eigen::MatrixXd & x_pred)
{
  // Get the lateral and yaw errors and their derivatives from 'x_pred'
  getPrediction(x_pred);

  // Set the gains for the Super Twisting Algorithm
  double gain_e_lat = 1.0;
  double gain_e_yaw = 1.0;

  // Calculate the control vectors 'u_lat_12' and 'u_yaw_12'
  // using the Super Twisting Algorithm
  Eigen::VectorXd u_lat_12, u_yaw_12;
  u_lat_12 = superTwistingAlgorithm(
    m_e_lat, m_e_lat_dot, m_u2_lat_prev,
    gain_e_lat * m_params.lambda,
    gain_e_lat * m_params.alpha,
    gain_e_lat * m_params.beta);
  u_yaw_12 = superTwistingAlgorithm(
    m_e_yaw, m_e_yaw_dot, m_u2_yaw_prev,
    gain_e_yaw * m_params.lambda,
    gain_e_yaw * m_params.alpha,
    gain_e_yaw * m_params.beta);

  // Update the previous control inputs
  m_u2_lat_prev = u_lat_12[1];
  m_u2_yaw_prev = u_yaw_12[1];

  // Calculate the control command
  double ctrl_cmd = u_lat_12[0] + u_yaw_12[0];
  // Limit the control command to the maximum steering angle
  if (std::abs(ctrl_cmd) >= m_steer_lim) {
    ctrl_cmd = m_steer_lim * std::copysign(1.0, ctrl_cmd);
  }

  // Calculate the control command rate
  double ctrl_cmd_rate = (ctrl_cmd - m_ctrl_cmd_prev) / m_ctrl_period;
  m_ctrl_cmd_prev = ctrl_cmd;  // Update the previous control command

  // Create a vector 'result' containing the control command, its rate,
  // and the control vectors 'u_lat_12' and 'u_yaw_12'.
  Eigen::VectorXd result(6);
  result << ctrl_cmd, ctrl_cmd_rate, u_lat_12, u_yaw_12;
  return result;
}

// The getPrediction function extracts the lateral and yaw errors and their
// derivatives from 'x_pred' and returns them as a vector.
void SMC::getPrediction(const Eigen::MatrixXd & x_pred)
{
  m_e_lat = x_pred(0, static_cast<int>(m_n_pred * 1 / 2));
  m_e_lat_dot = x_pred(1, static_cast<int>(m_n_pred * 1 / 2));
  m_e_yaw = x_pred(2, m_n_pred - 1);
  m_e_yaw_dot = x_pred(3, m_n_pred - 1);
}

// The setVelocity function updates the current velocity with the provided value.
void SMC::setVelocity(const double & velocity)
{
  m_velocity = velocity;
}

// The superTwistingAlgorithm function implements the Super Twisting Algorithm,
// a second-order sliding mode control method. It calculates the sliding surface
// 's_curr' and the saturation 's_sat' based on the current error 'e', its derivative
// 'e_dot', and the previous control input 'u2_prev'. It then computes a 2D control
// vector 'u' based on these values and various parameters. The function returns this
// control vector 'u'.
Eigen::VectorXd SMC::superTwistingAlgorithm(
  const double & e, const double & e_dot, const double & u2_prev,
  const double & lambda, const double & alpha, const double & beta)
{
  // set gamma
  const double gamma = m_params.gamma;

  // Calculate the sliding surface
  double s_curr = e_dot + lambda * e;

  // Calculate the boundary layer
  const double phi = std::max(1.0, std::abs(gamma * m_velocity));

  // sliding surface normalized by boundary layer
  // double s_phi = std::max(-1.0, std::min(1.0, s_curr / phi));
  double s_phi = std::tanh(s_curr / phi);

  // Calculate the control input using the Super Twisting Algorithm
  Eigen::VectorXd u(2);
  u << -alpha * std::sqrt(std::abs(s_phi)) * s_phi + u2_prev,
    (std::abs(m_velocity) > 0.1) ?
    -beta * m_velocity * s_phi * m_ctrl_period + u2_prev :
    u2_prev * m_decay_factor;

  return u;
}
}  // namespace autoware::motion::control::smc_lateral_controller
