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

#include "smc_lateral_controller/vehicle_model/interface.hpp"

namespace autoware::motion::control::smc_lateral_controller
{
Interface::Interface(int dim_x, int dim_u, int dim_y, double wheelbase)
: m_dim_x(dim_x), m_dim_u(dim_u), m_dim_y(dim_y), m_wheelbase(wheelbase)
{
}
int Interface::getDimX()
{
  return m_dim_x;
}
int Interface::getDimU()
{
  return m_dim_u;
}
int Interface::getDimY()
{
  return m_dim_y;
}
double Interface::getWheelbase()
{
  return m_wheelbase;
}
void Interface::setVelocity(const double velocity)
{
  m_velocity = velocity;
}
void Interface::setCurvature(const double curvature)
{
  m_curvature = curvature;
}
}  // namespace autoware::motion::control::smc_lateral_controller
