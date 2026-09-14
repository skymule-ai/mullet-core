#include "mullet_core/telemetry/paddock_heartbeat_publisher.hpp"

#include <utility>

#include "mullet_core/px4/topic_utils.hpp"

namespace mullet_core
{

namespace
{
// Reliable/volatile/keep-last(5): unlike the PX4 uXRCE-DDS link (px4Qos()), the ground-station
// heartbeat has no transient-local requirement and benefits from delivery guarantees over a
// possibly lossy ground link.
rclcpp::QoS heartbeatQos()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}
}  // namespace

PaddockHeartbeatPublisher::PaddockHeartbeatPublisher(
  rclcpp::Node & node, const std::string & topic, double rate_hz,
  SnapshotBuilder snapshot_builder)
: snapshot_builder_(std::move(snapshot_builder))
{
  publisher_ = node.create_publisher<mullet_interfaces::msg::DroneHeartbeat>(
    topic, heartbeatQos());
  timer_ = node.create_wall_timer(ratePeriod(rate_hz, 1.0), [this]() {tick();});
}

void PaddockHeartbeatPublisher::tick()
{
  publisher_->publish(snapshot_builder_());
}

}  // namespace mullet_core
