// Copyright 2023 North Carolina A & T State University
// Copyright 2018-2021 Autoware Foundation
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

// representation:
// s_path:  path length
// e_lat:   lateral error
// e_ang:   angular error
// u_steer: steering angle
// l_wb:    wheelbase length
// k_curv:  path curvature
// v:       velocity
//
// states and input
// x = [e_lat, e_ang]^T
// u = u_steer
//
// nonlinear kinematic model in path coordinates
// [s_path' ] = [                             cos(e_ang) / (1 - (e_lat' * k_curv))]
// [e_lat'  ]   [                                                       sin(e_ang)] * v
// [e_ang'  ]   [tan(u_steer) / l_wb - k_curv * cos(e_ang) / (1 - e_ang' * k_curv)]
// [u_steer']   [                                                                0]
//                  [0]
//                + [0] * u_steer'
//                  [0]
//                  [1]
// nonlinear model for path tracking
// [e_lat'  ] = [                                                       sin(e_ang)] * v
// [e_lat'' ] = [                                                         e_lat'/v]
// [e_ang'  ]   [tan(u_steer) / l_wb - k_curv * cos(e_ang) / (1 - e_ang' * k_curv)]
// [e_ang'' ] = [                                                         e_ang'/v]
//
// Linearized model around reference point (v = v_r, th = th_r, steer = steer_r)
// dx/dt = [0, 0, vr, 0] * x + [                  0] * u + [                           0]
//         [0, 1,  0, 0]       [                  0]       [                           0]
//         [0, 0,  0, 0]       [vr/W/cos(steer_r)^2]       [-vr*steer_r/W/cos(steer_r)^2]
//         [0, 0,  0, 1]       [                  0]       [                           0]

#ifndef SMC_LATERAL_CONTROLLER__VEHICLE_MODEL__KINEMATIC_HPP_
#define SMC_LATERAL_CONTROLLER__VEHICLE_MODEL__KINEMATIC_HPP_

#include "smc_lateral_controller/vehicle_model/interface.hpp"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <string>

namespace autoware::motion::control::smc_lateral_controller
{
/**
 * Vehicle model class of bicycle kinematics
 * @brief calculate model-related values
 */
class Kinematic : public Interface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] steer_lim steering angle limit [rad]
   */
  Kinematic(const double wheelbase, const double steer_lim);

  /**
   * @brief destructor
   */
  ~Kinematic() = default;

  /**
   * @brief calculate discrete model matrix of x_k+1 = a_d * xk + b_d * uk + w_d, yk = c_d * xk
   * @param [out] a_d coefficient matrix
   * @param [out] b_d coefficient matrix
   * @param [out] c_d coefficient matrix
   * @param [out] w_d coefficient matrix
   * @param [in] dt Discretization time [s]
   */
  void calculateDiscreteMatrix(
    Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
    const double dt) override;

  /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
  void calculateReferenceInput(Eigen::MatrixXd & u_ref) override;

  std::string modelName() override {return "kinematics";}

private:
  double m_steer_lim;  //!< @brief steering angle limit [rad]
};
}  // namespace autoware::motion::control::smc_lateral_controller
#endif  // SMC_LATERAL_CONTROLLER__VEHICLE_MODEL__KINEMATIC_HPP_
