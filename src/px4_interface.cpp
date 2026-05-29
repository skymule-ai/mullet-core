#include "mullet_core/px4_interface.hpp"

#include <array>
#include <limits>
#include <utility>

namespace mullet_core
{

namespace
{
constexpr float kPx4CustomMainMode = 1.0F;
constexpr float kPx4OffboardMode = 6.0F;
constexpr float kPx4PositionMode = 3.0F;
constexpr float kArmParam = 1.0F;

rclcpp::QoS px4QoS()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.transient_local();
  return qos;
}

std::array<float, 3> nanVector()
{
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  return {nan, nan, nan};
}
}  // namespace

Px4Interface::Px4Interface(rclcpp::Node & node, std::string fmu_prefix, std::uint8_t target_system)
: node_(node), fmu_prefix_(std::move(fmu_prefix)), target_system_(target_system)
{
  const auto qos = px4QoS();

  offboard_mode_pub_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(
    topic("/fmu/in/offboard_control_mode"), qos);
  trajectory_setpoint_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    topic("/fmu/in/trajectory_setpoint"), qos);
  vehicle_command_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
    topic("/fmu/in/vehicle_command"), qos);

  local_position_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    topic("/fmu/out/vehicle_local_position"), qos,
    [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {onLocalPosition(msg);});
  vehicle_status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic("/fmu/out/vehicle_status_v1"), qos,
    [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {onVehicleStatus(msg);});
}

void Px4Interface::publishOffboardControlMode(bool position_enabled, bool velocity_enabled)
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = position_enabled;
  msg.velocity = velocity_enabled;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = nowUs();
  offboard_mode_pub_->publish(msg);
}

void Px4Interface::publishPositionSetpoint(
  float x_ned_m, float y_ned_m, float z_ned_m, float yaw_rad)
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position = {x_ned_m, y_ned_m, z_ned_m};
  msg.velocity = nanVector();
  msg.acceleration = nanVector();
  msg.jerk = nanVector();
  msg.yaw = yaw_rad;
  msg.yawspeed = 0.0F;
  msg.timestamp = nowUs();
  trajectory_setpoint_pub_->publish(msg);
}

void Px4Interface::publishVelocitySetpoint(
  float vx_ned_mps, float vy_ned_mps, float vz_ned_mps, float yaw_rad)
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position = nanVector();
  msg.velocity = {vx_ned_mps, vy_ned_mps, vz_ned_mps};
  msg.acceleration = nanVector();
  msg.jerk = nanVector();
  msg.yaw = yaw_rad;
  msg.yawspeed = 0.0F;
  msg.timestamp = nowUs();
  trajectory_setpoint_pub_->publish(msg);
}

void Px4Interface::arm()
{
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, kArmParam);
}

void Px4Interface::setOffboardMode()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, kPx4CustomMainMode,
    kPx4OffboardMode);
}

void Px4Interface::setPositionMode()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, kPx4CustomMainMode,
    kPx4PositionMode);
}

void Px4Interface::land()
{
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

bool Px4Interface::hasLocalPosition() const
{
  return has_local_position_;
}

bool Px4Interface::hasVehicleStatus() const
{
  return has_vehicle_status_;
}

bool Px4Interface::localPositionValid() const
{
  return has_local_position_ && latest_local_position_.xy_valid && latest_local_position_.z_valid;
}

bool Px4Interface::isArmed() const
{
  return has_vehicle_status_ &&
         latest_vehicle_status_.arming_state ==
           px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

bool Px4Interface::isOffboard() const
{
  return has_vehicle_status_ &&
         latest_vehicle_status_.nav_state ==
           px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool Px4Interface::isDisarmed() const
{
  return has_vehicle_status_ &&
         latest_vehicle_status_.arming_state ==
           px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
}

const px4_msgs::msg::VehicleLocalPosition & Px4Interface::localPosition() const
{
  return latest_local_position_;
}

const px4_msgs::msg::VehicleStatus & Px4Interface::vehicleStatus() const
{
  return latest_vehicle_status_;
}

std::string Px4Interface::topic(const std::string & suffix) const
{
  if (fmu_prefix_.empty()) {
    return suffix;
  }
  if (fmu_prefix_.front() == '/') {
    return fmu_prefix_ + suffix;
  }
  return "/" + fmu_prefix_ + suffix;
}

std::uint64_t Px4Interface::nowUs() const
{
  return static_cast<std::uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);
}

void Px4Interface::publishVehicleCommand(
  std::uint16_t command, float param1, float param2, float param3, float param4,
  float param5, float param6, float param7)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.command = command;
  msg.target_system = target_system_;
  msg.target_component = target_component_;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = nowUs();
  vehicle_command_pub_->publish(msg);
}

void Px4Interface::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  latest_local_position_ = *msg;
  has_local_position_ = true;
}

void Px4Interface::onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  latest_vehicle_status_ = *msg;
  has_vehicle_status_ = true;
}

}  // namespace mullet_core
