#include "mullet_core/flight/drone_state_machine.hpp"

#include "flight/detail/action_lifecycle_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mullet_core
{

void DroneStateMachine::publishTakeoffFeedback(float progress)
{
  if (!action_lifecycle_->takeoff().active()) {
    return;
  }
  auto feedback = std::make_shared<Takeoff::Feedback>();
  feedback->state = stateName(state_);
  feedback->target_altitude_m = takeoff_origin_z_ned_m_ - takeoff_z_ned_m_;
  if (px4_->hasLocalPosition() && px4_->localPosition().z_valid) {
    feedback->current_altitude_m = takeoff_origin_z_ned_m_ - px4_->localPosition().z;
    feedback->altitude_error_m = std::fabs(
      feedback->target_altitude_m - feedback->current_altitude_m);
  } else {
    feedback->current_altitude_m = 0.0F;
    feedback->altitude_error_m = std::numeric_limits<float>::infinity();
  }
  feedback->progress = progress;
  action_lifecycle_->takeoff().publish(feedback);
}

void DroneStateMachine::publishLandFeedback(const rclcpp::Time & current_time)
{
  if (!action_lifecycle_->land().active()) {
    return;
  }

  auto feedback = std::make_shared<Land::Feedback>();
  feedback->state = px4_->isArmed() ? "WAITING_FOR_DISARM" : stateName(state_);
  feedback->armed = px4_->isArmed();
  feedback->elapsed_s = static_cast<float>(
    std::max(0.0, (current_time - landing_start_time_).seconds()));
  action_lifecycle_->land().publish(feedback);
}

void DroneStateMachine::publishReturnHomeFeedback(float distance_m)
{
  if (!action_lifecycle_->returnHome().active()) {
    return;
  }

  auto feedback = std::make_shared<ReturnHome::Feedback>();
  feedback->distance_m = distance_m;
  action_lifecycle_->returnHome().publish(feedback);
}

}  // namespace mullet_core
