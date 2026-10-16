#include "mullet_core/telemetry/localization_status_publisher.hpp"

#include <utility>

#include "mullet_core/px4/topic_utils.hpp"

namespace mullet_core
{

mullet_interfaces::msg::LocalizationStatus toLocalizationStatus(
  const px4_msgs::msg::EstimatorStatusFlags & flags, const std::string & drone_id)
{
  mullet_interfaces::msg::LocalizationStatus msg;
  msg.drone_id = drone_id;
  msg.gnss_pos = flags.cs_gnss_pos;
  msg.gnss_vel = flags.cs_gnss_vel;
  msg.ev_pos = flags.cs_ev_pos;
  msg.ev_vel = flags.cs_ev_vel;
  msg.ev_yaw = flags.cs_ev_yaw;
  // EKF2_MAG_TYPE 0 (automatic) fuses heading on the ground and the 3-axis field in flight.
  msg.mag_hdg = flags.cs_mag_hdg || flags.cs_mag_3d;
  msg.baro_hgt = flags.cs_baro_hgt;
  msg.rng_hgt = flags.cs_rng_hgt;
  msg.yaw_aligned = flags.cs_yaw_align;
  msg.dead_reckoning = flags.cs_inertial_dead_reckoning;
  return msg;
}

LocalizationStatusPublisher::LocalizationStatusPublisher(
  rclcpp::Node & node, const std::string & fmu_prefix, std::string drone_id)
: node_(node), drone_id_(std::move(drone_id))
{
  flags_sub_ = node_.create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
    px4Topic(fmu_prefix, "/fmu/out/estimator_status_flags"),  // unversioned in px4_msgs 1.17
    px4Qos(),
    [this](const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg) {latest_ = *msg;});
  // Same QoS as the heartbeat: reliable, volatile, over a possibly lossy ground link.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
  qos.reliable();
  qos.durability_volatile();
  publisher_ = node_.create_publisher<mullet_interfaces::msg::LocalizationStatus>(
    "/mullet/localization_status", qos);
  timer_ = node_.create_wall_timer(ratePeriod(1.0, 1.0), [this]() {tick();});
}

void LocalizationStatusPublisher::tick()
{
  if (!latest_) {
    return;
  }
  auto msg = toLocalizationStatus(*latest_, drone_id_);
  msg.stamp = node_.now();
  publisher_->publish(msg);
}

}  // namespace mullet_core
