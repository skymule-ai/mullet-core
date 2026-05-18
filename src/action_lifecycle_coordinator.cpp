#include "action_lifecycle_coordinator.hpp"

#include <utility>

namespace mullet_core::state_machine_detail
{

bool ActionLifecycleCoordinator::acceptTakeoffGoal(std::shared_ptr<GoalHandleTakeoff> goal)
{
  if (active_takeoff_goal_) {
    return false;
  }
  active_takeoff_goal_ = std::move(goal);
  takeoff_cancel_requested_ = false;
  return true;
}

bool ActionLifecycleCoordinator::acceptLandGoal(std::shared_ptr<GoalHandleLand> goal)
{
  if (active_land_goal_) {
    return false;
  }
  active_land_goal_ = std::move(goal);
  return true;
}

bool ActionLifecycleCoordinator::hasActiveTakeoffGoal() const
{
  return active_takeoff_goal_ != nullptr;
}

bool ActionLifecycleCoordinator::hasActiveLandGoal() const
{
  return active_land_goal_ != nullptr;
}

bool ActionLifecycleCoordinator::isActiveTakeoffGoal(
  const std::shared_ptr<GoalHandleTakeoff> & goal) const
{
  return goal == active_takeoff_goal_;
}

bool ActionLifecycleCoordinator::isActiveLandGoal(
  const std::shared_ptr<GoalHandleLand> & goal) const
{
  return goal == active_land_goal_;
}

void ActionLifecycleCoordinator::requestTakeoffCancel()
{
  takeoff_cancel_requested_ = true;
}

bool ActionLifecycleCoordinator::takeoffCancelRequested() const
{
  return takeoff_cancel_requested_;
}

void ActionLifecycleCoordinator::publishTakeoffFeedback(
  const std::shared_ptr<Takeoff::Feedback> & feedback) const
{
  if (active_takeoff_goal_) {
    active_takeoff_goal_->publish_feedback(feedback);
  }
}

void ActionLifecycleCoordinator::publishLandFeedback(
  const std::shared_ptr<Land::Feedback> & feedback) const
{
  if (active_land_goal_) {
    active_land_goal_->publish_feedback(feedback);
  }
}

bool ActionLifecycleCoordinator::succeedTakeoff(const std::string & message)
{
  if (!active_takeoff_goal_) {
    return false;
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = true;
  result->message = message;
  active_takeoff_goal_->succeed(result);
  active_takeoff_goal_.reset();
  takeoff_cancel_requested_ = false;
  return true;
}

bool ActionLifecycleCoordinator::abortTakeoff(const std::string & message)
{
  if (!active_takeoff_goal_) {
    return false;
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = false;
  result->message = message;
  active_takeoff_goal_->abort(result);
  active_takeoff_goal_.reset();
  takeoff_cancel_requested_ = false;
  return true;
}

bool ActionLifecycleCoordinator::cancelTakeoff(const std::string & message)
{
  if (!active_takeoff_goal_) {
    return false;
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = false;
  result->message = message;
  active_takeoff_goal_->canceled(result);
  active_takeoff_goal_.reset();
  takeoff_cancel_requested_ = false;
  return true;
}

bool ActionLifecycleCoordinator::succeedLand(const std::string & message)
{
  if (!active_land_goal_) {
    return false;
  }
  auto result = std::make_shared<Land::Result>();
  result->success = true;
  result->message = message;
  active_land_goal_->succeed(result);
  active_land_goal_.reset();
  return true;
}

bool ActionLifecycleCoordinator::abortLand(const std::string & message)
{
  if (!active_land_goal_) {
    return false;
  }
  auto result = std::make_shared<Land::Result>();
  result->success = false;
  result->message = message;
  active_land_goal_->abort(result);
  active_land_goal_.reset();
  return true;
}

bool ActionLifecycleCoordinator::abortRejectedTakeoff(
  const std::shared_ptr<GoalHandleTakeoff> & goal, const std::string & message) const
{
  if (!goal) {
    return false;
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = false;
  result->message = message;
  goal->abort(result);
  return true;
}

bool ActionLifecycleCoordinator::abortRejectedLand(
  const std::shared_ptr<GoalHandleLand> & goal, const std::string & message) const
{
  if (!goal) {
    return false;
  }
  auto result = std::make_shared<Land::Result>();
  result->success = false;
  result->message = message;
  goal->abort(result);
  return true;
}

}  // namespace mullet_core::state_machine_detail
