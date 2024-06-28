// Copyright 2023 North Carolina A & T State University
// Copyright 2018-2021 The Autoware Foundation
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

#include "smc_lateral_controller/vehicle_model/kinematic.hpp"

namespace autoware::motion::control::smc_lateral_controller
{
Kinematic::Kinematic(
  const double wheelbase, const double steer_lim)
: Interface(/* dim_x */ 4, /* dim_u */ 1, /* dim_y */ 2, wheelbase)
{
  m_wheelbase = wheelbase;
  m_steer_lim = steer_lim;
}

void Kinematic::calculateDiscreteMatrix(
  Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
  const double dt)
{
  auto sign = [](double x) {return (x > 0.0) - (x < 0.0);};

  /* Linearize delta around delta_r (reference delta) */
  double delta_r = atan(m_wheelbase * m_curvature);
  if (std::abs(delta_r) >= m_steer_lim) {
    delta_r = m_steer_lim * static_cast<double>(sign(delta_r));
  }
  double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

  a_d = Eigen::MatrixXd::Zero(m_dim_x, m_dim_x);
  b_d = Eigen::MatrixXd::Zero(m_dim_x, m_dim_u);
  c_d = Eigen::MatrixXd::Zero(m_dim_y, m_dim_x);
  w_d = Eigen::MatrixXd::Zero(m_dim_x, 1);

  a_d(0, 2) = m_velocity;
  a_d(1, 3) = m_velocity;

  b_d(2, 0) = m_velocity / m_wheelbase * cos_delta_r_squared_inv;

  c_d(0, 0) = 1.0;
  c_d(1, 2) = 1.0;

  w_d(2, 0) = -m_velocity / m_wheelbase * cos_delta_r_squared_inv * delta_r;

  // add negative gains in the diagonal of a_d to make the system stable
  // TODO: make the gains configurable using parameters
  a_d(0, 0) = -std::max(2.0, 0.5 * m_velocity);
  a_d(1, 1) = -std::max(2.0, 0.5 * m_velocity);
  a_d(2, 2) = -2.0;
  a_d(3, 3) = -2.0;

  // bilinear discretization for ZOH system
  // no discretization is needed for Cd
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  const Eigen::MatrixXd i_dt2a_inv = (I - dt * 0.5 * a_d).inverse();
  a_d = i_dt2a_inv * (I + dt * 0.5 * a_d);
  b_d = i_dt2a_inv * b_d * dt;
  w_d = i_dt2a_inv * w_d * dt;
}

void Kinematic::calculateReferenceInput(Eigen::MatrixXd & u_ref)
{
  u_ref(0, 0) = std::atan(m_wheelbase * m_curvature);
}
}  // namespace autoware::motion::control::smc_lateral_controller
