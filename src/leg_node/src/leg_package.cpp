#include <memory>
#include <leg_node/leg_node.hpp>


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto nodePtr = std::make_shared<LegNode>();
  rclcpp::spin(nodePtr);
  rclcpp::shutdown();
  return 0;
}
