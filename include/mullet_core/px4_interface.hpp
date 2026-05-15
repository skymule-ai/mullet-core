#ifndef MULLET_CORE_PX4_INTERFACE_HPP_
#define MULLET_CORE_PX4_INTERFACE_HPP_

#include <cstdint>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

class Px4Interface
{
public:
  Px4Interface(rclcpp::Node & node, std::string fmu_prefix, std::uint8_t target_system);

  void publishOffboardControlMode(bool position_enabled, bool velocity_enabled);
  void publishPositionSetpoint(float x_ned_m, float y_ned_m, float z_ned_m, float yaw_rad);
  void publishVelocitySetpoint(
    float vx_ned_mps, float vy_ned_mps, float vz_ned_mps, float yaw_rad);

  void arm();
  void setOffboardMode();
  void setPositionMode();
  void land();

  bool hasLocalPosition() const;
  bool hasVehicleStatus() const;
  bool localPositionValid() const;
  bool isArmed() const;
  bool isOffboard() const;
  bool isDisarmed() const;

  const px4_msgs::msg::VehicleLocalPosition & localPosition() const;
  const px4_msgs::msg::VehicleStatus & vehicleStatus() const;

private:
  std::string topic(const std::string & suffix) const;
  std::uint64_t nowUs() const;
  void publishVehicleCommand(
    std::uint16_t command, float param1 = 0.0F, float param2 = 0.0F,
    float param3 = 0.0F, float param4 = 0.0F, float param5 = 0.0F,
    float param6 = 0.0F, float param7 = 0.0F);
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  rclcpp::Node & node_;
  std::string fmu_prefix_;
  std::uint8_t target_system_;
  std::uint8_t target_component_{1};

  px4_msgs::msg::VehicleLocalPosition latest_local_position_{};
  px4_msgs::msg::VehicleStatus latest_vehicle_status_{};
  bool has_local_position_{false};
  bool has_vehicle_status_{false};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_PX4_INTERFACE_HPP_
