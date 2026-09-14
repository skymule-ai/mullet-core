#ifndef MULLET_CORE_PX4_PX4_INTERFACE_HPP_
#define MULLET_CORE_PX4_PX4_INTERFACE_HPP_

#include <cstdint>
#include <string>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

// Publishes PX4 vehicle commands and retains the telemetry used by the flight state machine.
class Px4Interface
{
public:
  // Creates PX4 publishers and telemetry subscriptions for one vehicle.
  //
  // Args:
  //   node: Node that owns the ROS interfaces.
  //   fmu_prefix: PX4 FMU namespace prefix.
  //   target_system: PX4 target system ID.
  Px4Interface(rclcpp::Node & node, std::string fmu_prefix, std::uint8_t target_system);

  // Requests PX4 to arm the vehicle.
  void arm();
  // Requests PX4 OFFBOARD navigation mode.
  void setOffboardMode();
  // Requests PX4 position-control navigation mode.
  void setPositionMode();
  // Requests PX4 to land the vehicle.
  void land();

  // Returns whether a local-position message has been received.
  bool hasLocalPosition() const;
  // Returns whether a VehicleStatus message has been received yet.
  bool hasVehicleStatus() const;
  // Returns whether the latest local-position message reports valid xy/z fields.
  bool localPositionValid() const;
  // Returns whether a VehicleLandDetected message has been received yet.
  bool hasLandDetected() const;
  // Returns whether the latest land-detected message reports the vehicle as landed.
  bool isLanded() const;
  // Returns whether the latest vehicle-status message reports the vehicle as armed.
  bool isArmed() const;
  // Returns whether the latest vehicle-status message reports OFFBOARD navigation mode.
  bool isOffboard() const;
  // Returns whether the latest vehicle-status message reports the vehicle as disarmed.
  bool isDisarmed() const;
  // Returns whether the latest vehicle-status message reports an active failsafe.
  bool isFailsafe() const;

  // Returns the latest received local-position message.
  const px4_msgs::msg::VehicleLocalPosition & localPosition() const;
  // Returns the latest received vehicle-status message.
  const px4_msgs::msg::VehicleStatus & vehicleStatus() const;

private:
  // Publishes a PX4 VehicleCommand addressed to this vehicle's target system/component, filling
  // in the timestamp and source IDs shared by every command type.
  void publishVehicleCommand(
    std::uint16_t command, float param1 = 0.0F, float param2 = 0.0F,
    float param3 = 0.0F, float param4 = 0.0F, float param5 = 0.0F,
    float param6 = 0.0F, float param7 = 0.0F);
  // Updates the retained VehicleLocalPosition snapshot.
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  // Updates the retained VehicleStatus snapshot.
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  // Updates the retained VehicleLandDetected snapshot.
  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

  rclcpp::Node & node_;
  std::uint8_t target_system_;
  std::uint8_t target_component_{1};

  px4_msgs::msg::VehicleLocalPosition latest_local_position_{};
  px4_msgs::msg::VehicleStatus latest_vehicle_status_{};
  px4_msgs::msg::VehicleLandDetected latest_land_detected_{};
  bool has_local_position_{false};
  bool has_vehicle_status_{false};
  bool has_land_detected_{false};

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_PX4_PX4_INTERFACE_HPP_
