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

#ifndef SMC_LATERAL_CONTROLLER__MOVE_AVERAGE_FILTER_HPP_
#define SMC_LATERAL_CONTROLLER__MOVE_AVERAGE_FILTER_HPP_

#include <deque>

namespace autoware::motion::control::smc_lateral_controller
{
class MoveAverageFilter
{
private:
  int m_num;
  std::deque<double> m_u;

public:
  /**
   * @brief filtering deque
   * @param [in] num index distance for moving average filter
   */
  explicit MoveAverageFilter(int num = 10);

  /**
   * @brief destructor
   */
  ~MoveAverageFilter();

  /**
   * @brief filtering deque
   * @param [in] num index distance for moving average filter
   * @param [out] u object deque
   */
  void initialize(const int num);

  /**
   * @brief filtering (call this function at each sampling time with input)
   * @param [in] u scalar input for filter
   * @return filtered scalar value
   */
  double filter(const double & u);
};
}  // namespace autoware::motion::control::smc_lateral_controller
#endif  // SMC_LATERAL_CONTROLLER__MOVE_AVERAGE_FILTER_HPP_
