#include "mullet_core/px4/offboard_heartbeat_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "mullet_core/px4/topic_utils.hpp"

namespace mullet_core
{

namespace
{
// Whether any component is set; PX4 leaves NaN components uncontrolled.
bool anySet(const OffboardHeartbeatPublisher::Vector3 & values)
{
  return std::any_of(values.begin(), values.end(), [](float value) {return std::isfinite(value);});
}
}  // namespace

OffboardHeartbeatPublisher::OffboardHeartbeatPublisher(
  rclcpp::Node & node, std::string fmu_prefix, double rate_hz, SetpointBuilder setpoint_builder)
: node_(node), setpoint_builder_(std::move(setpoint_builder))
{
  const auto qos = px4Qos();
  offboard_mode_pub_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(
    px4Topic(fmu_prefix, "/fmu/in/offboard_control_mode"), qos);
  trajectory_setpoint_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    px4Topic(
      fmu_prefix, "/fmu/in/trajectory_setpoint",
      px4_msgs::msg::TrajectorySetpoint::MESSAGE_VERSION),
    qos);
  timer_ = node_.create_wall_timer(ratePeriod(rate_hz, 2.0), [this]() {tick();});
}

void OffboardHeartbeatPublisher::publishControlMode(
  bool position_enabled, bool velocity_enabled, bool acceleration_enabled)
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = position_enabled;
  msg.velocity = velocity_enabled;
  msg.acceleration = acceleration_enabled;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = px4NowUs(*node_.get_clock());
  offboard_mode_pub_->publish(msg);
}

void OffboardHeartbeatPublisher::tick()
{
  const auto setpoint = setpoint_builder_();
  if (!setpoint.active) {
    return;
  }

  publishControlMode(
    anySet(setpoint.position), anySet(setpoint.velocity), anySet(setpoint.acceleration));
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position = setpoint.position;
  msg.velocity = setpoint.velocity;
  msg.acceleration = setpoint.acceleration;
  msg.jerk = {kUnset, kUnset, kUnset};
  msg.yaw = setpoint.yaw_rad;
  msg.yawspeed = 0.0F;
  msg.timestamp = px4NowUs(*node_.get_clock());
  trajectory_setpoint_pub_->publish(msg);
}

}  // namespace mullet_core
