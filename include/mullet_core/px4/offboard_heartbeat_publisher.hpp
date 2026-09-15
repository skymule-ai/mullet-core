#ifndef MULLET_CORE_PX4_OFFBOARD_HEARTBEAT_PUBLISHER_HPP_
#define MULLET_CORE_PX4_OFFBOARD_HEARTBEAT_PUBLISHER_HPP_

#include <array>
#include <functional>
#include <limits>
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
  // PX4's TrajectorySetpoint "not controlled" marker.
  static constexpr float kUnset = std::numeric_limits<float>::quiet_NaN();
  using Vector3 = std::array<float, 3>;

  // What to stream on a given tick, in PX4 NED (m, m/s, m/s^2, rad). As in TrajectorySetpoint,
  // a kUnset component is left to PX4, so axes can mix (e.g. position z with velocity x/y).
  // OffboardControlMode enables position, velocity and acceleration control for each field
  // with any set component. `active=false` publishes nothing this tick (e.g. the vehicle is not
  // under offboard control).
  struct Setpoint
  {
    bool active{false};
    Vector3 position{kUnset, kUnset, kUnset};
    Vector3 velocity{kUnset, kUnset, kUnset};
    Vector3 acceleration{kUnset, kUnset, kUnset};
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
  // Publishes which control fields this tick's TrajectorySetpoint carries (attitude and
  // body_rate are unused by this supervisor).
  void publishControlMode(bool position_enabled, bool velocity_enabled, bool acceleration_enabled);

  rclcpp::Node & node_;
  SetpointBuilder setpoint_builder_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_PX4_OFFBOARD_HEARTBEAT_PUBLISHER_HPP_
