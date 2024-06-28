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

#include "smc_lateral_controller/smc_lateral_controller.hpp"

namespace autoware::motion::control::smc_lateral_controller
{
SmcLateralController::SmcLateralController(rclcpp::Node & node)
: node_{&node}
{
  using std::placeholders::_1;

  // vehicle parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*node_).getVehicleInfo();
  const double wheelbase = vehicle_info.wheel_base_m;
  const double steer_lim = vehicle_info.max_steer_angle_rad;
  const double steer_rate_lim = node_->declare_parameter<double>("steer_rate_lim");

  // algorithm parameters
  m_traj_resample_dist = node_->declare_parameter<double>("traj_resample_dist");
  m_path_filter_moving_ave_num = node_->declare_parameter<int>("path_filter_moving_ave_num");
  m_enable_path_smoothing = node_->declare_parameter<bool>("enable_path_smoothing");
  m_ctrl_period = node_->get_parameter("ctrl_period").as_double();
  m_n_pred = node_->declare_parameter<int>("n_pred");
  m_converged_steer_rad = node_->declare_parameter<double>("converged_steer_rad");
  const double decay_speed = node_->declare_parameter<double>("decay_speed");

  // smc parameters
  const double lambda = node_->declare_parameter<double>("lambda");
  const double alpha = node_->declare_parameter<double>("alpha");
  const double beta = node_->declare_parameter<double>("beta");
  const double phi = node_->declare_parameter<double>("phi");

  // initialize smc
  m_smc_ptr = initializeSMC(steer_lim, steer_rate_lim, decay_speed, m_n_pred);
  m_smc_ptr->setGains(lambda, alpha, beta, phi);

  // initialize vehicle states
  m_x_prev = Eigen::VectorXd::Zero(4);

  // set parameter callback
  m_set_param_res = node_->add_on_set_parameters_callback(
    std::bind(&SmcLateralController::paramCallback, this, _1));

  pub_predicted_traj_ = node_->create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  pub_debug_values_ =
    node_->create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);

  // initialize lowpass filters
  {
    const double lpf_cutoff_hz = node_->declare_parameter<double>("lpf_cutoff_hz");
    const double cmd_lpf_cutoff_hz = node_->declare_parameter<double>("cmd_lpf_cutoff_hz");
    initializeFilters(lpf_cutoff_hz, cmd_lpf_cutoff_hz);
  }

  // vehicle model pointer
  m_vehicle_model_ptr = createVehicleModel(wheelbase, steer_lim);
}

SmcLateralController::~SmcLateralController()
{
}

// Callback function for setting parameters
rcl_interfaces::msg::SetParametersResult SmcLateralController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // helper lambda for updating parameters
  auto update_param = [&](const std::string & name, auto & v) {
      auto it = std::find_if(
        parameters.cbegin(), parameters.cend(),
        [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
      if (it != parameters.cend()) {
        if constexpr (std::is_same_v<decltype(v), int>) {
          v = it->as_int();
        } else if constexpr (std::is_same_v<decltype(v), double>) {
          v = it->as_double();
        }
        return true;
      }
      return false;
    };

  // update parameters
  {
    double lambda = node_->get_parameter("lambda").as_double();
    double alpha = node_->get_parameter("alpha").as_double();
    double beta = node_->get_parameter("beta").as_double();
    double phi = node_->get_parameter("phi").as_double();

    update_param("lambda", lambda);
    update_param("alpha", alpha);
    update_param("beta", beta);
    update_param("phi", phi);
    m_smc_ptr->setGains(lambda, alpha, beta, phi);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

// Create the vehicle model based on the provided parameters
std::shared_ptr<Interface> SmcLateralController::createVehicleModel(
  const double wheelbase, const double steer_lim)
{
  std::shared_ptr<Interface> vehicle_model_ptr;

  const std::string vehicle_model_type =
    node_->declare_parameter<std::string>("vehicle_model_type");

  if (vehicle_model_type == "kinematic") {
    vehicle_model_ptr = std::make_shared<Kinematic>(wheelbase, steer_lim);
    return vehicle_model_ptr;
  }

  if (vehicle_model_type == "dynamic") {
    const double mass_fl = node_->declare_parameter<double>("mass_fl");
    const double mass_fr = node_->declare_parameter<double>("mass_fr");
    const double mass_rl = node_->declare_parameter<double>("mass_rl");
    const double mass_rr = node_->declare_parameter<double>("mass_rr");
    const double cf = node_->declare_parameter<double>("cf");
    const double cr = node_->declare_parameter<double>("cr");

    // vehicle_model_ptr is only assigned in ctor, so parameter value have to be passed at init time
    vehicle_model_ptr =
      std::make_shared<Dynamic>(wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    return vehicle_model_ptr;
  }

  RCLCPP_ERROR(node_->get_logger(), "vehicle_model_type is undefined");
  return vehicle_model_ptr;
}

std::unique_ptr<SMC> SmcLateralController::initializeSMC(
  const double steer_lim, const double steer_rate_lim,
  const double decay_speed, const int n_pred)
{
  std::unique_ptr<SMC> smc_controller_ptr;

  smc_controller_ptr = std::make_unique<SMC>(
    steer_lim, steer_rate_lim, m_ctrl_period,
    decay_speed, n_pred);

  return smc_controller_ptr;
}

// Run the lateral controller
trajectory_follower::LateralOutput SmcLateralController::run(
  trajectory_follower::InputData const & input_data)
{
  // update the current states
  m_current_trajectory = input_data.current_trajectory;
  m_current_odometry = input_data.current_odometry;
  m_current_steering = input_data.current_steering;
  m_current_operation_mode = input_data.current_operation_mode;

  // add lookahead distance
  // NOTE: The lookahead distance does not improve the e_lat accuracy
  // m_current_odometry = smc_utils::add_lookahead_distance(
  //   m_current_odometry, 1.0);

  // check if autoware mode and controller mode are enabled
  bool is_controller_active = false;
  if (m_current_operation_mode.is_autoware_control_enabled == true &&
      m_current_operation_mode.mode == 2) {
    is_controller_active = true;
  }

  // resample the trajectory
  smc_utils::resample_traj_by_distance(
    m_current_trajectory, m_traj_resample_dist);
  if (m_enable_path_smoothing) {
    // smooth the trajectory
    smc_utils::smooth_trajectory(
      m_current_trajectory, m_path_filter_moving_ave_num);
  }

  // get the current control command, velocity and curvature
  double u_curr = m_current_steering.steering_tire_angle;
  double velocity = m_current_odometry.twist.twist.linear.x;

  // update the states
  auto x_curr = updateStates();

  // calculate the predictions
  auto [x_pred, odom_pred, k_pred, u_pred] = calculatePredictions(
    x_curr, u_curr, velocity);  

  // set the states for the controller
  m_smc_ptr->setVelocity(velocity);
  m_smc_ptr->setPrediction(x_pred);

  // calculate the control command
  auto ctrl_cmd = m_smc_ptr->calculate(
    is_controller_active, u_curr);

  // filter cmd for smoothness
  ctrl_cmd[0] = m_lpf_cmd.filter(ctrl_cmd[0]);

  // debug_values for path tracking
  // TODO: Publish x_pred, odom_pred, k_pred, u_pred, x_curr, and ctrl_cmd
  Float32MultiArrayStamped debug_values;

  debug_values.data.resize(10);
  debug_values.data[0] = static_cast<float>(x_curr[0]);
  debug_values.data[1] = static_cast<float>(x_curr[1]);
  debug_values.data[2] = static_cast<float>(x_curr[2]);
  debug_values.data[3] = static_cast<float>(x_curr[3]);
  debug_values.data[4] = static_cast<float>(ctrl_cmd[0]);
  debug_values.data[5] = static_cast<float>(ctrl_cmd[1]);
  debug_values.data[6] = static_cast<float>(ctrl_cmd[2]);
  debug_values.data[7] = static_cast<float>(ctrl_cmd[3]);
  debug_values.data[8] = static_cast<float>(ctrl_cmd[4]);
  debug_values.data[9] = static_cast<float>(ctrl_cmd[5]);
  debug_values.stamp = node_->now();

  publishDebugValues(debug_values);

  // predicted trajectory for path tracking
  publishPredictedTraj(odom_pred);

  trajectory_follower::LateralOutput output;
  output.control_cmd.stamp = node_->now();
  output.control_cmd.steering_tire_angle = ctrl_cmd[0];
  output.control_cmd.steering_tire_rotation_rate = ctrl_cmd[1];

  output.sync_data.is_steer_converged =
    isSteerConverged(output.control_cmd.steering_tire_angle);

  return output;
}

// Check if the controller is ready
bool SmcLateralController::isReady(const trajectory_follower::InputData & input_data)
{
  const auto unused_variable = input_data;

  return true;
}

// Publish the predicted trajectory
// TODO: Change the current input by a vector containing the predicted trajectory
void SmcLateralController::publishPredictedTraj(
  const std::vector<Odometry> & odom_pred) const
{
  Trajectory predicted_traj;
  predicted_traj.header.stamp = node_->now();
  predicted_traj.header.frame_id = "map";

  TrajectoryPoint p;
  for (int i = 0; i < m_n_pred; ++i) {
    p.pose = odom_pred[i].pose.pose;
    p.longitudinal_velocity_mps = odom_pred[i].twist.twist.linear.x;
    p.lateral_velocity_mps = odom_pred[i].twist.twist.linear.y;
    p.heading_rate_rps = odom_pred[i].twist.twist.angular.z;

    predicted_traj.points.push_back(p);
  }
  pub_predicted_traj_->publish(predicted_traj);
}

// Publish the debug values
void SmcLateralController::publishDebugValues(Float32MultiArrayStamped & debug_values) const
{
  // debug values
  debug_values.stamp = node_->now();
  pub_debug_values_->publish(debug_values);
}

// Update the states of the lateral controller
Eigen::VectorXd SmcLateralController::updateStates()
{
  Eigen::VectorXd x_curr(4);

  // Calculate the errors
  x_curr[0] = smc_utils::calculate_lateral_error(
    m_current_trajectory, m_current_odometry);
  x_curr[2] = smc_utils::calculate_angular_error(
    m_current_trajectory, m_current_odometry);

  // Filter the errors
  // NOTE: No improvement is observed when smoothing the error derivatives.
  x_curr[0] = m_lpf_e_lat.filter(x_curr[0]);
  x_curr[2] = m_lpf_e_yaw.filter(x_curr[2]);

  // Calculate the derivatives
  x_curr[1] = (x_curr[0] - m_x_prev[0]) / m_ctrl_period;
  x_curr[3] = (x_curr[2] - m_x_prev[2]) / m_ctrl_period;

  // Update the previous values
  m_x_prev = x_curr;

  return x_curr;
}

// Calculate the predictions
std::tuple<Eigen::MatrixXd, std::vector<Odometry>, std::vector<double>, std::vector<double>> SmcLateralController::calculatePredictions(
  const Eigen::VectorXd & x0, const double & u_steer, const double & velocity)
{
  Eigen::VectorXd x_curr(4);
  Eigen::MatrixXd x_pred(4, m_n_pred);
  std::vector<Odometry> odom_pred(m_n_pred);
  std::vector<double> k_pred(m_n_pred);
  std::vector<double> u_pred(m_n_pred);

  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  const double dt = m_ctrl_period;

  // Perform n_pred predictions and store the results
  x_curr = x0;
  x_pred = Eigen::MatrixXd::Zero(DIM_X, m_n_pred);
  odom_pred[0] = m_current_odometry;
  Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
  for (int i = 0; i < m_n_pred; ++i) {
    // predict odometry
    // TODO: The odometry prediction is not accurate
    // this can be improved by using the vehicle model to predict the odometry
    if (i > 0) {
      odom_pred[i] = smc_utils::update_odometry(
        odom_pred[i - 1], dt);
    }

    // predict curvature
    // NOTE: The curvature prediction improves the e_lat accuracy
    k_pred[i] = smc_utils::calculate_curvature(
      m_current_trajectory, odom_pred[i]);
    
    // filter curvature
    // NOTE: The curvature filter smooths the cmd derivative but worsen the e_lat accuracy
    k_pred[i] = m_lpf_k[i].filter(k_pred[i]);

    m_vehicle_model_ptr->setVelocity(velocity);
    m_vehicle_model_ptr->setCurvature(k_pred[i]);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, dt);
    m_vehicle_model_ptr->calculateReferenceInput(Uref);
    u_pred[i] = Uref(0, 0);
    ud(0, 0) = u_steer;

    x_curr = Ad * x_curr + Bd * ud + Wd;
    x_pred.col(i) = x_curr;
  }

  return std::make_tuple(x_pred, odom_pred, k_pred, u_pred);
}

bool SmcLateralController::isSteerConverged(const double & steer_cmd)
{
  return std::abs(steer_cmd - m_current_steering.steering_tire_angle) <
         static_cast<float>(m_converged_steer_rad);
}
}  // namespace autoware::motion::control::smc_lateral_controller
