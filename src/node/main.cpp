#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mullet_core/flight/drone_state_machine.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<mullet_core::DroneStateMachine>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mullet_core"), "Startup failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
