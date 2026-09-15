#include "mullet_core/px4/px4_interface.hpp"

#include <utility>

#include "mullet_core/px4/topic_utils.hpp"

namespace mullet_core
{

namespace
{
constexpr float kPx4CustomMainMode = 1.0F;
constexpr float kPx4OffboardMode = 6.0F;
constexpr float kPx4PositionMode = 3.0F;
constexpr float kArmParam = 1.0F;
// Identifies mullet_core as the command source to PX4; PX4 does not otherwise use these IDs.
constexpr std::uint8_t kSourceSystemId = 1;
constexpr std::uint8_t kSourceComponentId = 1;
}  // namespace

Px4Interface::Px4Interface(rclcpp::Node & node, std::string fmu_prefix, std::uint8_t target_system)
: node_(node), target_system_(target_system)
{
  const auto qos = px4Qos();

  vehicle_command_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
    px4Topic(
      fmu_prefix, "/fmu/in/vehicle_command", px4_msgs::msg::VehicleCommand::MESSAGE_VERSION),
    qos);

  local_position_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    px4Topic(
      fmu_prefix, "/fmu/out/vehicle_local_position",
      px4_msgs::msg::VehicleLocalPosition::MESSAGE_VERSION),
    qos,
    [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {onLocalPosition(msg);});
  vehicle_status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
    px4Topic(
      fmu_prefix, "/fmu/out/vehicle_status", px4_msgs::msg::VehicleStatus::MESSAGE_VERSION),
    qos,
    [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {onVehicleStatus(msg);});
  land_detected_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLandDetected>(
    px4Topic(
      fmu_prefix, "/fmu/out/vehicle_land_detected",
      px4_msgs::msg::VehicleLandDetected::MESSAGE_VERSION),
    qos,
    [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {onLandDetected(msg);});
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

bool Px4Interface::hasLandDetected() const
{
  return has_land_detected_;
}

bool Px4Interface::isLanded() const
{
  return has_land_detected_ && latest_land_detected_.landed;
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

bool Px4Interface::isFailsafe() const
{
  return has_vehicle_status_ && latest_vehicle_status_.failsafe;
}

const px4_msgs::msg::VehicleLocalPosition & Px4Interface::localPosition() const
{
  return latest_local_position_;
}

const px4_msgs::msg::VehicleStatus & Px4Interface::vehicleStatus() const
{
  return latest_vehicle_status_;
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
  msg.source_system = kSourceSystemId;
  msg.source_component = kSourceComponentId;
  msg.from_external = true;
  msg.timestamp = px4NowUs(*node_.get_clock());
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

void Px4Interface::onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  latest_land_detected_ = *msg;
  has_land_detected_ = true;
}

}  // namespace mullet_core
