#include "mullet_core/px4/offboard_heartbeat_publisher.hpp"

#include <array>
#include <limits>
#include <utility>

#include "mullet_core/px4/topic_utils.hpp"

namespace mullet_core
{

namespace
{
// PX4's TrajectorySetpoint uses NaN to mean "field not set"; whichever of position/velocity isn't
// being streamed this tick must be filled with NaN rather than left as zero.
std::array<float, 3> nanVector()
{
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  return {nan, nan, nan};
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
    px4Topic(fmu_prefix, "/fmu/in/trajectory_setpoint"), qos);
  timer_ = node_.create_wall_timer(ratePeriod(rate_hz, 2.0), [this]() {tick();});
}

void OffboardHeartbeatPublisher::publishControlMode(bool position_enabled, bool velocity_enabled)
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = position_enabled;
  msg.velocity = velocity_enabled;
  msg.acceleration = false;
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

  publishControlMode(setpoint.position_mode, !setpoint.position_mode);
  px4_msgs::msg::TrajectorySetpoint msg{};
  if (setpoint.position_mode) {
    msg.position = {setpoint.x, setpoint.y, setpoint.z};
    msg.velocity = nanVector();
  } else {
    msg.position = nanVector();
    msg.velocity = {setpoint.x, setpoint.y, setpoint.z};
  }
  msg.acceleration = nanVector();
  msg.jerk = nanVector();
  msg.yaw = setpoint.yaw_rad;
  msg.yawspeed = 0.0F;
  msg.timestamp = px4NowUs(*node_.get_clock());
  trajectory_setpoint_pub_->publish(msg);
}

}  // namespace mullet_core
