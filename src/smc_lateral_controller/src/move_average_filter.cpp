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

#include "smc_lateral_controller/move_average_filter.hpp"

#include <vector>

namespace autoware::motion::control::smc_lateral_controller
{
MoveAverageFilter::MoveAverageFilter(int num)
{
  initialize(num);
}

MoveAverageFilter::~MoveAverageFilter()
{
}

void MoveAverageFilter::initialize(const int num)
{
  m_num = num;
  m_u = std::deque<double>();
}

double MoveAverageFilter::filter(const double & u0)
{
  m_u.push_back(u0);
  if (static_cast<int>(m_u.size()) >= m_num) {
    m_u.pop_front();
  }

  double sum = 0.0;
  for (double value : m_u) {
    sum += value;
  }
  return sum / m_u.size();
}
}  // namespace autoware::motion::control::smc_lateral_controller
