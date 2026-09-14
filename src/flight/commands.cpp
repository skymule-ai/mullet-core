#include "mullet_core/flight/drone_state_machine.hpp"

#include <cmath>
#include <cstddef>

namespace mullet_core
{

namespace
{
constexpr std::size_t kCommandVxIndex = 0U;
constexpr std::size_t kCommandVyIndex = 1U;
constexpr std::size_t kCommandModeIndex = 2U;
constexpr std::size_t kMinVelocityCommandSize = 3U;
constexpr float kVelocityCommandMode = 1.0F;
constexpr float kCommandModeTolerance = 1e-3F;
}  // namespace

// Latches every valid command regardless of state; only MISSION ever streams it to PX4.
void DroneStateMachine::onCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < kMinVelocityCommandSize) {
    RCLCPP_WARN(get_logger(), "Invalid command: expected [vx_ned_mps, vy_ned_mps, mode]");
    return;
  }

  const auto vx_ned_mps = msg->data[kCommandVxIndex];
  const auto vy_ned_mps = msg->data[kCommandVyIndex];
  const auto mode = msg->data[kCommandModeIndex];
  if (!std::isfinite(vx_ned_mps) || !std::isfinite(vy_ned_mps) || !std::isfinite(mode)) {
    RCLCPP_WARN(get_logger(), "Invalid command: non-finite value rejected");
    return;
  }
  if (std::fabs(mode - kVelocityCommandMode) > kCommandModeTolerance) {
    RCLCPP_WARN(get_logger(), "Invalid command mode %.3f: expected mode=1", mode);
    return;
  }

  latest_command_.vx_ned_mps = vx_ned_mps;
  latest_command_.vy_ned_mps = vy_ned_mps;
  latest_command_.stamp = now();
  latest_command_.valid = true;
}

bool DroneStateMachine::commandFresh(const rclcpp::Time & current_time) const
{
  return latest_command_.valid &&
         (current_time - latest_command_.stamp).seconds() < config_.command_timeout_s;
}

}  // namespace mullet_core
