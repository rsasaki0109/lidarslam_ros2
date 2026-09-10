#include "glim_clean_room/phase3d_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<glim_clean_room::phase3d::GlimPhase3dNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
