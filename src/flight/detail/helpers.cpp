#include "flight/detail/helpers.hpp"

#include <cmath>

namespace mullet_core::state_machine_detail
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

bool retryDue(const rclcpp::Time & now, const rclcpp::Time & last, double interval_s)
{
  return last.nanoseconds() == 0 || (now - last).seconds() >= interval_s;
}

bool finiteInRange(double value, double min_value, double max_value)
{
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

float wrapPi(float angle_rad)
{
  constexpr double kTwoPi = 2.0 * kPi;
  constexpr double kBoundaryTolerance = 1e-6;
  const auto wrapped = std::remainder(static_cast<double>(angle_rad), kTwoPi);
  if (wrapped <= -kPi || std::fabs(wrapped + kPi) <= kBoundaryTolerance) {
    return static_cast<float>(kPi);
  }
  return static_cast<float>(wrapped);
}

GlobalPosition localToGlobal(
  double ref_lat_deg, double ref_lon_deg, double ref_alt_m,
  double x_north_m, double y_east_m, double z_down_m)
{
  // ponytail: equirectangular projection around the EKF origin, sub-metre within ~1 km of it.
  // Subscribe to /fmu/out/vehicle_global_position instead if missions range further.
  constexpr double kEarthRadiusM = 6371000.0;
  constexpr double kDegToRad = kPi / 180.0;
  return {
    ref_lat_deg + x_north_m / kEarthRadiusM / kDegToRad,
    ref_lon_deg + y_east_m / (kEarthRadiusM * std::cos(ref_lat_deg * kDegToRad)) / kDegToRad,
    ref_alt_m - z_down_m,
  };
}

}  // namespace mullet_core::state_machine_detail
