#include "failure_injection/failure_injection.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FailureInjection>());
  rclcpp::shutdown();
  return 0;
}
