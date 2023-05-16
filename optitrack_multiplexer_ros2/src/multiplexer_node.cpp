#include "optitrack_multiplexer.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optitrack_multiplexer::OptitrackMultiplexer>());
  rclcpp::shutdown();
}
