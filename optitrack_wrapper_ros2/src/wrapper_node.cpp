#include "optitrack_wrapper.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optitrack_wrapper::OptitrackWrapper>());
  rclcpp::shutdown();
}
