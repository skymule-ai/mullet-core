#include <rclcpp/rclcpp.hpp>

#include "mullet_core/drone_state_machine.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mullet_core::DroneStateMachine>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
