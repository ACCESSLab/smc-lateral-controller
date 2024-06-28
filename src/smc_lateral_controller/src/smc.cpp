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

// Update the SMC parameters with the provided values.
void SMC::setGains(
  const double & lambda,
  const double & alpha,
  const double & beta,
  const double & phi
)
{
  m_params.lambda = lambda;
  m_params.alpha = alpha;
  m_params.beta = beta;
  m_params.phi = phi;
}

// Compute the control command.
Eigen::VectorXd SMC::calculate(
  const bool & is_controller_active, const double & u_curr)
{
  // Calculate the control inputs using the Super Twisting Algorithm
  superTwistingAlgorithm(m_e_lat, m_u1_lat, m_u2_lat,
    m_params.lambda, m_params.alpha, m_params.beta,
    is_controller_active);
  superTwistingAlgorithm(m_e_yaw, m_u1_yaw, m_u2_yaw,
    m_params.lambda, m_params.alpha, m_params.beta,
    is_controller_active);

  // Calculate the control command and rate
  Eigen::VectorXd cmd(2);
  if (is_controller_active) {
    cmd[0] = m_u1_lat + m_u1_yaw;
  } else {
    cmd[0] = u_curr;
    m_u1_lat = 0.0;
    m_u1_yaw = 0.0;
    m_u2_lat = 0.0;
    m_u2_yaw = 0.0;
  }
  cmd[1] = (cmd[0] - m_u[0]) / m_ctrl_period;
  
  // Limit the steering angle and rate
  limitSteeringAngle(cmd);
  
  // Update the previous control command
  m_u = cmd;

  // Return the output command
  Eigen::VectorXd output(6);
  output << cmd, m_u1_lat, m_u1_yaw, m_u2_lat, m_u2_yaw;

  return output;
}

// Update the current prediction with the provided value.
void SMC::setPrediction(const Eigen::MatrixXd & x_pred)
{
  // Extract the lateral and yaw errors and their derivatives
  m_e_lat[0] = x_pred(0, m_n_pred);
  m_e_lat[1] = x_pred(1, m_n_pred);
  m_e_yaw[0] = x_pred(2, m_n_pred);
  m_e_yaw[1] = x_pred(3, m_n_pred);
}

// Update the current velocity with the provided value.
void SMC::setVelocity(const double & velocity)
{
  m_velocity = velocity;
}

// Update the current curvature with the provided value.
void SMC::setCurvature(const double & curvature)
{
  m_curvature = curvature;
}

// Implement the Super Twisting Algorithm (STA) for lateral control.
void SMC::superTwistingAlgorithm(
  const Eigen::VectorXd & e, double & u1, double & u2,
  const double & lambda, const double & alpha, const double & beta,
  const bool & is_controller_active)
{
  // Calculate the sliding surface
  double s_curr = e[1] + lambda * e[0];

  // Calculate the boundary layer
  // TODO: make the minimum boundary layer configurable using parameters
  double phi = std::max(
    2.0, std::abs(m_params.phi * m_velocity));

  // Normalize the sliding surface
  double s_phi = std::tanh(s_curr / phi);

  // Calulate the second control input
  u2 = ((std::abs(m_velocity) > m_decay_speed) && (is_controller_active))?
    -beta * m_velocity * s_phi * m_ctrl_period + u2 : u2 * m_decay_speed;
  
  // Calculate the first control input
  u1 = -alpha * std::sqrt(std::abs(s_phi)) * s_phi + u2;
  
  // Limit the first and second control input
  if (std::abs(u1) >= m_steer_lim) {
    u1 = m_steer_lim * std::copysign(1.0, u1);
  }
  if (std::abs(u2) >= m_steer_lim) {
    u2 = m_steer_lim * std::copysign(1.0, u2);
  }
}

// Limit the steering angle and rate.
void SMC::limitSteeringAngle(Eigen::VectorXd & cmd)
{
  // Limit the control rate to the maximum steering rate
  if (std::abs(cmd[1]) >= m_steer_rate_lim) {
    cmd[1] = m_steer_rate_lim * std::copysign(1.0, cmd[1]);

    // Calculate the control command based on the limited rate
    cmd[0] = m_u[0] + cmd[1] * m_ctrl_period;
  }
  // Limit the control command to the maximum steering angle
  if (std::abs(cmd[0]) >= m_steer_lim) {
    cmd[0] = m_steer_lim * std::copysign(1.0, cmd[0]);
  }
}
}  // namespace autoware::motion::control::smc_lateral_controller
