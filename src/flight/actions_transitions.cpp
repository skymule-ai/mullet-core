#include "mullet_core/flight/drone_state_machine.hpp"

#include "flight/detail/action_lifecycle_coordinator.hpp"

namespace mullet_core
{

void DroneStateMachine::startLanding(const char * reason)
{
  position_mode_requested_for_landing_ = true;
  landing_start_time_ = now();
  landing_timeout_reported_ = false;
  enterPhase(Phase::kLanding, reason);
}

void DroneStateMachine::failTakeoff(const char * reason)
{
  action_lifecycle_->takeoff().abort(reason);
  endTakeoff(reason);
}

void DroneStateMachine::endTakeoff(const char * reason)
{
  if (shouldRecoverThroughLanding()) {
    startLanding(reason);
  } else {
    transitionTo(DroneState::kIdle, reason);
  }
}

void DroneStateMachine::resetVehicleCommandRetryTimer()
{
  last_vehicle_command_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

}  // namespace mullet_core
