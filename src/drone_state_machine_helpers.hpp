#ifndef MULLET_CORE_SRC_DRONE_STATE_MACHINE_HELPERS_HPP_
#define MULLET_CORE_SRC_DRONE_STATE_MACHINE_HELPERS_HPP_

#include <rclcpp/time.hpp>

#include "mullet_core/types.hpp"

namespace mullet_core::state_machine_detail
{

bool retryDue(const rclcpp::Time & now, const rclcpp::Time & last, double interval_s);

bool finiteInRange(double value, double min_value, double max_value);

float wrapPi(float angle_rad);

bool isActiveTakeoffState(DroneState state);

bool isLandActionAllowedFromState(DroneState state);

}  // namespace mullet_core::state_machine_detail

#endif  // MULLET_CORE_SRC_DRONE_STATE_MACHINE_HELPERS_HPP_
