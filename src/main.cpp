#include <rclcpp/rclcpp.hpp>

#include "robot_config/robot_config.h"

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotConfig>());
  rclcpp::shutdown();
  return 0;
}
