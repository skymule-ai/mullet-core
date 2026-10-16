#ifndef MULLET_CORE_TELEMETRY_LOCALIZATION_STATUS_PUBLISHER_HPP_
#define MULLET_CORE_TELEMETRY_LOCALIZATION_STATUS_PUBLISHER_HPP_

#include <optional>
#include <string>

#include <mullet_interfaces/msg/localization_status.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mullet_core
{

// Maps PX4's EKF2 control-status flags to the operator-facing LocalizationStatus (stamp unset).
//
// Args:
//   flags: The latest EstimatorStatusFlags from PX4.
//   drone_id: This drone's id, the key consumers use.
//
// Returns:
//   The message, one field per fused source.
mullet_interfaces::msg::LocalizationStatus toLocalizationStatus(
  const px4_msgs::msg::EstimatorStatusFlags & flags, const std::string & drone_id);

// Republishes PX4's estimator_status_flags as LocalizationStatus on /mullet/localization_status
// at 1 Hz, so the Paddock can show which localization profile is active. Read-only: it never
// writes to PX4. Silent until the first flags arrive, so a consumer never sees made-up falses.
class LocalizationStatusPublisher
{
public:
  // Args:
  //   node: Node used for the subscription, publisher and timer.
  //   fmu_prefix: Vehicle FMU namespace, as for px4Topic().
  //   drone_id: This drone's id.
  LocalizationStatusPublisher(
    rclcpp::Node & node, const std::string & fmu_prefix, std::string drone_id);

private:
  void tick();

  rclcpp::Node & node_;
  std::string drone_id_;
  std::optional<px4_msgs::msg::EstimatorStatusFlags> latest_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr flags_sub_;
  rclcpp::Publisher<mullet_interfaces::msg::LocalizationStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mullet_core

#endif  // MULLET_CORE_TELEMETRY_LOCALIZATION_STATUS_PUBLISHER_HPP_
