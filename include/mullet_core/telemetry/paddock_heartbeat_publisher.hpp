#ifndef MULLET_CORE_TELEMETRY_PADDOCK_HEARTBEAT_PUBLISHER_HPP_
#define MULLET_CORE_TELEMETRY_PADDOCK_HEARTBEAT_PUBLISHER_HPP_

#include <functional>
#include <string>

#include <mullet_interfaces/msg/drone_heartbeat.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

// Publishes the paddock-facing DroneHeartbeat message on a fixed-rate timer. Owns the heartbeat
// publisher and its timer; on each tick it calls the supplied snapshot builder and publishes the
// returned message unchanged. Message content is the caller's responsibility - this class only
// owns when and where the heartbeat is sent.
class PaddockHeartbeatPublisher
{
public:
  using SnapshotBuilder = std::function<mullet_interfaces::msg::DroneHeartbeat()>;

  // Advertises `topic` and starts a timer publishing at `rate_hz`.
  //
  // Args:
  //   node: Node used to create the publisher and the timer.
  //   topic: Topic to publish DroneHeartbeat messages on.
  //   rate_hz: Publish rate in Hz. Clamped to a minimum of 1 Hz.
  //   snapshot_builder: Callback invoked once per tick to build the message to publish.
  PaddockHeartbeatPublisher(
    rclcpp::Node & node, const std::string & topic, double rate_hz,
    SnapshotBuilder snapshot_builder);

private:
  // Builds one snapshot via `snapshot_builder_` and publishes it.
  void tick();

  SnapshotBuilder snapshot_builder_;
  rclcpp::Publisher<mullet_interfaces::msg::DroneHeartbeat>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_TELEMETRY_PADDOCK_HEARTBEAT_PUBLISHER_HPP_
