#include "failure_injection/failure_injection.hpp"

FailureInjection::FailureInjection() 
  : Node("failure_injection")
{
  // Initialize the subscribers
  sub_odometry_ = this->create_subscription<Odometry>(
    "localization/kinematic_state_", rclcpp::QoS{1}, std::bind(
      &FailureInjection::onOdometry, this, _1));
  sub_steering_ = this->create_subscription<SteeringReport>(
    "vehicle/status/steering_status_", rclcpp::QoS{1}, std::bind(
      &FailureInjection::onSteering, this, _1));

  // Initialize the publishers
  pub_odometry_ = this->create_publisher<Odometry>(
    "localization/kinematic_state", rclcpp::QoS{1});
  pub_steering_ = this->create_publisher<SteeringReport>(
    "vehicle/status/steering_status", rclcpp::QoS{1});
}

FailureInjection::~FailureInjection()
{
}

void FailureInjection::onOdometry(
  const Odometry::SharedPtr msg_odometry)
{
  double x = msg_odometry->pose.pose.position.x; // Initial x position
  double y = msg_odometry->pose.pose.position.y; // Initial y position
  double yaw = tf2::getYaw(msg_odometry->pose.pose.orientation); // Initial yaw angle
  double velocity = msg_odometry->twist.twist.linear.x; // Initial velocity

  double mean = 0.0; // Mean of the Gaussian distribution
  double stddev = 0.02; // Standard deviation of the Gaussian distribution

  // select the test case:
  // 1: Add noise to x and y
  // 2: Add noise to yaw
  // 3: Add noise to velocity
  int test_case = 0;

  // Add Gaussian noise and update the odometry message
  if (test_case == 1) {
    addGaussianNoise(x, mean, stddev);
    addGaussianNoise(y, mean, stddev);
    msg_odometry->pose.pose.position.x = x;
    msg_odometry->pose.pose.position.y = y;
  } else if (test_case == 2) {
    addGaussianNoise(yaw, mean, stddev);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg_odometry->pose.pose.orientation = tf2::toMsg(q);
  } else if (test_case == 3) {
    addGaussianNoise(velocity, mean, stddev);
    msg_odometry->twist.twist.linear.x = velocity;
  } else {
    // Do nothing
  }

  pub_odometry_->publish(*msg_odometry);
}

void FailureInjection::onSteering(
  const SteeringReport::SharedPtr msg_steering)
{
  double steering = msg_steering->steering_tire_angle; // Initial steering angle

  double mean = 0.0; // Mean of the Gaussian distribution
  double stddev = 0.005; // Standard deviation of the Gaussian distribution

  // select the test case:
  // 1: Add noise to steering
  int test_case = 0;

  // Add Gaussian noise and update the odometry message
  if (test_case == 1) {
    addGaussianNoise(steering, mean, stddev);
    msg_steering->steering_tire_angle = steering;
  } else {
    // Do nothing
  }

  pub_steering_->publish(*msg_steering);
}

void FailureInjection::addGaussianNoise(
  double & value, double mean, double stddev)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> distribution(mean, stddev);

  value += distribution(gen);
}
