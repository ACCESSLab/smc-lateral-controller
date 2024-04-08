#ifndef FAILURE_INJECTION_HPP_
#define FAILURE_INJECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <random>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using nav_msgs::msg::Odometry;
using autoware_auto_vehicle_msgs::msg::SteeringReport;

class FailureInjection : public rclcpp::Node
{
  public:
    FailureInjection();
    ~FailureInjection();

  private:
    void onOdometry(
      const Odometry::SharedPtr msg);
    void onSteering(
      const SteeringReport::SharedPtr msg_steering);
    void addGaussianNoise(double & value, double mean, double stddev);

    rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;

    rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
};

#endif  // FAILURE_INJECTION_HPP_