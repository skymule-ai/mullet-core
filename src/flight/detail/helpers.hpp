#ifndef MULLET_CORE_SRC_DRONE_STATE_MACHINE_HELPERS_HPP_
#define MULLET_CORE_SRC_DRONE_STATE_MACHINE_HELPERS_HPP_

#include <rclcpp/time.hpp>

namespace mullet_core::state_machine_detail
{

// Returns whether a retriable PX4 command is due: no previous attempt (`last` is zero), or at
// least `interval_s` has elapsed since it.
bool retryDue(const rclcpp::Time & now, const rclcpp::Time & last, double interval_s);

// Returns whether `value` is finite and within [min_value, max_value].
bool finiteInRange(double value, double min_value, double max_value);

// Wraps an angle to (-pi, pi].
float wrapPi(float angle_rad);

// WGS84 position derived from PX4's local frame.
struct GlobalPosition
{
  double lat_deg;
  double lon_deg;
  double alt_m;
};

// Converts a PX4 local NED position to WGS84 around the EKF origin (the `ref_lat`, `ref_lon`,
// `ref_alt` fields of VehicleLocalPosition).
GlobalPosition localToGlobal(
  double ref_lat_deg, double ref_lon_deg, double ref_alt_m,
  double x_north_m, double y_east_m, double z_down_m);

}  // namespace mullet_core::state_machine_detail

#endif  // MULLET_CORE_SRC_DRONE_STATE_MACHINE_HELPERS_HPP_
