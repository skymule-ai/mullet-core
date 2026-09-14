#ifndef MULLET_CORE_PX4_OFFBOARD_HEARTBEAT_PUBLISHER_HPP_
#define MULLET_CORE_PX4_OFFBOARD_HEARTBEAT_PUBLISHER_HPP_

#include <functional>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

// Publishes the PX4 OFFBOARD control-mode heartbeat and setpoints PX4 requires to remain in
// OFFBOARD mode. PX4 expects an OffboardControlMode + setpoint pair at >= 2 Hz. This class owns
// its own timer and pulls what to stream from a caller-supplied builder on every tick, so the
// stream never depends on every FSM branch remembering to publish.
class OffboardHeartbeatPublisher
{
public:
  // What to stream on a given tick. `active=false` publishes nothing this tick (e.g. the vehicle
  // is not under offboard control). `x`/`y`/`z` hold NED meters when `position_mode` is true, or
  // NED m/s when false.
  struct Setpoint
  {
    bool active{false};
    bool position_mode{true};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    float yaw_rad{0.0F};
  };
  using SetpointBuilder = std::function<Setpoint()>;

  // Advertises the OffboardControlMode/TrajectorySetpoint topics under `fmu_prefix` and starts a
  // timer publishing at `rate_hz`.
  //
  // Args:
  //   node: Node used to create the publishers, the timer, and to source message timestamps.
  //   fmu_prefix: Vehicle FMU namespace prefix, e.g. "/px4_1", or empty for the default vehicle.
  //   rate_hz: Publish rate in Hz. Clamped to a minimum of 2 Hz, the rate PX4 requires to stay in
  //     OFFBOARD mode.
  //   setpoint_builder: Callback invoked once per tick to build the setpoint to stream.
  OffboardHeartbeatPublisher(
    rclcpp::Node & node, std::string fmu_prefix, double rate_hz, SetpointBuilder setpoint_builder);

private:
  // Builds this tick's Setpoint via `setpoint_builder_` and streams it to PX4 (control-mode
  // heartbeat + trajectory setpoint), skipping the publish entirely when `active` is false.
  void tick();
  // Publishes which control fields this tick's TrajectorySetpoint carries: position xor
  // velocity, never both or neither (acceleration/attitude/body_rate are unused by this
  // supervisor).
  void publishControlMode(bool position_enabled, bool velocity_enabled);

  rclcpp::Node & node_;
  SetpointBuilder setpoint_builder_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_PX4_OFFBOARD_HEARTBEAT_PUBLISHER_HPP_
