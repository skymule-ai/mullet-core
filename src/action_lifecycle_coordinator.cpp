#include "action_lifecycle_coordinator.hpp"

#include <memory>
#include <utility>

namespace mullet_core::state_machine_detail
{

namespace
{
enum class ActionTerminal
{
  kSucceeded,
  kAborted,
  kCanceled,
};

template<typename GoalHandleT>
bool acceptGoal(std::shared_ptr<GoalHandleT> & active_goal, std::shared_ptr<GoalHandleT> goal,
  bool * cancel_requested = nullptr)
{
  if (active_goal) {
    return false;
  }
  active_goal = std::move(goal);
  if (cancel_requested != nullptr) {
    *cancel_requested = false;
  }
  return true;
}

template<typename GoalHandleT>
bool hasActiveGoal(const std::shared_ptr<GoalHandleT> & active_goal)
{
  return active_goal != nullptr;
}

template<typename GoalHandleT>
bool isActiveGoal(
  const std::shared_ptr<GoalHandleT> & active_goal,
  const std::shared_ptr<GoalHandleT> & goal)
{
  return goal == active_goal;
}

template<typename ActionT, typename GoalHandleT>
void publishFeedback(
  const std::shared_ptr<GoalHandleT> & active_goal,
  const std::shared_ptr<typename ActionT::Feedback> & feedback)
{
  if (active_goal) {
    active_goal->publish_feedback(feedback);
  }
}

template<typename ActionT, typename GoalHandleT>
bool finishGoal(
  std::shared_ptr<GoalHandleT> & active_goal, ActionTerminal terminal,
  const std::string & message, bool * cancel_requested = nullptr)
{
  if (!active_goal) {
    return false;
  }

  auto result = std::make_shared<typename ActionT::Result>();
  result->success = terminal == ActionTerminal::kSucceeded;
  result->message = message;

  switch (terminal) {
    case ActionTerminal::kSucceeded:
      active_goal->succeed(result);
      break;
    case ActionTerminal::kAborted:
      active_goal->abort(result);
      break;
    case ActionTerminal::kCanceled:
      active_goal->canceled(result);
      break;
  }

  active_goal.reset();
  if (cancel_requested != nullptr) {
    *cancel_requested = false;
  }
  return true;
}

template<typename ActionT, typename GoalHandleT>
bool abortRejectedGoal(
  const std::shared_ptr<GoalHandleT> & goal, const std::string & message)
{
  if (!goal) {
    return false;
  }

  auto result = std::make_shared<typename ActionT::Result>();
  result->success = false;
  result->message = message;
  goal->abort(result);
  return true;
}
}  // namespace

bool ActionLifecycleCoordinator::acceptTakeoffGoal(std::shared_ptr<GoalHandleTakeoff> goal)
{
  return acceptGoal(active_takeoff_goal_, std::move(goal), &takeoff_cancel_requested_);
}

bool ActionLifecycleCoordinator::acceptLandGoal(std::shared_ptr<GoalHandleLand> goal)
{
  return acceptGoal(active_land_goal_, std::move(goal));
}

bool ActionLifecycleCoordinator::acceptEnableOffboardGoal(
  std::shared_ptr<GoalHandleEnableOffboard> goal)
{
  return acceptGoal(active_enable_offboard_goal_, std::move(goal), &enable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::acceptDisableOffboardGoal(
  std::shared_ptr<GoalHandleDisableOffboard> goal)
{
  return acceptGoal(
    active_disable_offboard_goal_, std::move(goal), &disable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::acceptArmGoal(std::shared_ptr<GoalHandleArm> goal)
{
  return acceptGoal(active_arm_goal_, std::move(goal), &arm_cancel_requested_);
}

bool ActionLifecycleCoordinator::acceptDisarmGoal(std::shared_ptr<GoalHandleDisarm> goal)
{
  return acceptGoal(active_disarm_goal_, std::move(goal), &disarm_cancel_requested_);
}

bool ActionLifecycleCoordinator::hasActiveTakeoffGoal() const
{
  return hasActiveGoal(active_takeoff_goal_);
}

bool ActionLifecycleCoordinator::hasActiveLandGoal() const
{
  return hasActiveGoal(active_land_goal_);
}

bool ActionLifecycleCoordinator::hasActiveEnableOffboardGoal() const
{
  return hasActiveGoal(active_enable_offboard_goal_);
}

bool ActionLifecycleCoordinator::hasActiveDisableOffboardGoal() const
{
  return hasActiveGoal(active_disable_offboard_goal_);
}

bool ActionLifecycleCoordinator::hasActiveArmGoal() const
{
  return hasActiveGoal(active_arm_goal_);
}

bool ActionLifecycleCoordinator::hasActiveDisarmGoal() const
{
  return hasActiveGoal(active_disarm_goal_);
}

bool ActionLifecycleCoordinator::isActiveTakeoffGoal(
  const std::shared_ptr<GoalHandleTakeoff> & goal) const
{
  return isActiveGoal(active_takeoff_goal_, goal);
}

bool ActionLifecycleCoordinator::isActiveLandGoal(
  const std::shared_ptr<GoalHandleLand> & goal) const
{
  return isActiveGoal(active_land_goal_, goal);
}

bool ActionLifecycleCoordinator::isActiveEnableOffboardGoal(
  const std::shared_ptr<GoalHandleEnableOffboard> & goal) const
{
  return isActiveGoal(active_enable_offboard_goal_, goal);
}

bool ActionLifecycleCoordinator::isActiveDisableOffboardGoal(
  const std::shared_ptr<GoalHandleDisableOffboard> & goal) const
{
  return isActiveGoal(active_disable_offboard_goal_, goal);
}

bool ActionLifecycleCoordinator::isActiveArmGoal(
  const std::shared_ptr<GoalHandleArm> & goal) const
{
  return isActiveGoal(active_arm_goal_, goal);
}

bool ActionLifecycleCoordinator::isActiveDisarmGoal(
  const std::shared_ptr<GoalHandleDisarm> & goal) const
{
  return isActiveGoal(active_disarm_goal_, goal);
}

void ActionLifecycleCoordinator::requestTakeoffCancel()
{
  takeoff_cancel_requested_ = true;
}

bool ActionLifecycleCoordinator::takeoffCancelRequested() const
{
  return takeoff_cancel_requested_;
}

void ActionLifecycleCoordinator::requestEnableOffboardCancel()
{
  enable_offboard_cancel_requested_ = true;
}

bool ActionLifecycleCoordinator::enableOffboardCancelRequested() const
{
  return enable_offboard_cancel_requested_;
}

void ActionLifecycleCoordinator::requestDisableOffboardCancel()
{
  disable_offboard_cancel_requested_ = true;
}

bool ActionLifecycleCoordinator::disableOffboardCancelRequested() const
{
  return disable_offboard_cancel_requested_;
}

void ActionLifecycleCoordinator::requestArmCancel()
{
  arm_cancel_requested_ = true;
}

bool ActionLifecycleCoordinator::armCancelRequested() const
{
  return arm_cancel_requested_;
}

void ActionLifecycleCoordinator::requestDisarmCancel()
{
  disarm_cancel_requested_ = true;
}

bool ActionLifecycleCoordinator::disarmCancelRequested() const
{
  return disarm_cancel_requested_;
}

void ActionLifecycleCoordinator::publishTakeoffFeedback(
  const std::shared_ptr<Takeoff::Feedback> & feedback) const
{
  publishFeedback<Takeoff>(active_takeoff_goal_, feedback);
}

void ActionLifecycleCoordinator::publishLandFeedback(
  const std::shared_ptr<Land::Feedback> & feedback) const
{
  publishFeedback<Land>(active_land_goal_, feedback);
}

void ActionLifecycleCoordinator::publishEnableOffboardFeedback(
  const std::shared_ptr<EnableOffboard::Feedback> & feedback) const
{
  publishFeedback<EnableOffboard>(active_enable_offboard_goal_, feedback);
}

void ActionLifecycleCoordinator::publishDisableOffboardFeedback(
  const std::shared_ptr<DisableOffboard::Feedback> & feedback) const
{
  publishFeedback<DisableOffboard>(active_disable_offboard_goal_, feedback);
}

void ActionLifecycleCoordinator::publishArmFeedback(
  const std::shared_ptr<Arm::Feedback> & feedback) const
{
  publishFeedback<Arm>(active_arm_goal_, feedback);
}

void ActionLifecycleCoordinator::publishDisarmFeedback(
  const std::shared_ptr<Disarm::Feedback> & feedback) const
{
  publishFeedback<Disarm>(active_disarm_goal_, feedback);
}

bool ActionLifecycleCoordinator::succeedTakeoff(const std::string & message)
{
  return finishGoal<Takeoff>(
    active_takeoff_goal_, ActionTerminal::kSucceeded, message, &takeoff_cancel_requested_);
}

bool ActionLifecycleCoordinator::abortTakeoff(const std::string & message)
{
  return finishGoal<Takeoff>(
    active_takeoff_goal_, ActionTerminal::kAborted, message, &takeoff_cancel_requested_);
}

bool ActionLifecycleCoordinator::cancelTakeoff(const std::string & message)
{
  return finishGoal<Takeoff>(
    active_takeoff_goal_, ActionTerminal::kCanceled, message, &takeoff_cancel_requested_);
}

bool ActionLifecycleCoordinator::succeedLand(const std::string & message)
{
  return finishGoal<Land>(active_land_goal_, ActionTerminal::kSucceeded, message);
}

bool ActionLifecycleCoordinator::abortLand(const std::string & message)
{
  return finishGoal<Land>(active_land_goal_, ActionTerminal::kAborted, message);
}

bool ActionLifecycleCoordinator::succeedEnableOffboard(const std::string & message)
{
  return finishGoal<EnableOffboard>(
    active_enable_offboard_goal_, ActionTerminal::kSucceeded, message,
    &enable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::abortEnableOffboard(const std::string & message)
{
  return finishGoal<EnableOffboard>(
    active_enable_offboard_goal_, ActionTerminal::kAborted, message,
    &enable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::cancelEnableOffboard(const std::string & message)
{
  return finishGoal<EnableOffboard>(
    active_enable_offboard_goal_, ActionTerminal::kCanceled, message,
    &enable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::succeedDisableOffboard(const std::string & message)
{
  return finishGoal<DisableOffboard>(
    active_disable_offboard_goal_, ActionTerminal::kSucceeded, message,
    &disable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::abortDisableOffboard(const std::string & message)
{
  return finishGoal<DisableOffboard>(
    active_disable_offboard_goal_, ActionTerminal::kAborted, message,
    &disable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::cancelDisableOffboard(const std::string & message)
{
  return finishGoal<DisableOffboard>(
    active_disable_offboard_goal_, ActionTerminal::kCanceled, message,
    &disable_offboard_cancel_requested_);
}

bool ActionLifecycleCoordinator::succeedArm(const std::string & message)
{
  return finishGoal<Arm>(
    active_arm_goal_, ActionTerminal::kSucceeded, message, &arm_cancel_requested_);
}

bool ActionLifecycleCoordinator::abortArm(const std::string & message)
{
  return finishGoal<Arm>(
    active_arm_goal_, ActionTerminal::kAborted, message, &arm_cancel_requested_);
}

bool ActionLifecycleCoordinator::cancelArm(const std::string & message)
{
  return finishGoal<Arm>(
    active_arm_goal_, ActionTerminal::kCanceled, message, &arm_cancel_requested_);
}

bool ActionLifecycleCoordinator::succeedDisarm(const std::string & message)
{
  return finishGoal<Disarm>(
    active_disarm_goal_, ActionTerminal::kSucceeded, message, &disarm_cancel_requested_);
}

bool ActionLifecycleCoordinator::abortDisarm(const std::string & message)
{
  return finishGoal<Disarm>(
    active_disarm_goal_, ActionTerminal::kAborted, message, &disarm_cancel_requested_);
}

bool ActionLifecycleCoordinator::cancelDisarm(const std::string & message)
{
  return finishGoal<Disarm>(
    active_disarm_goal_, ActionTerminal::kCanceled, message, &disarm_cancel_requested_);
}

bool ActionLifecycleCoordinator::abortRejectedTakeoff(
  const std::shared_ptr<GoalHandleTakeoff> & goal, const std::string & message) const
{
  return abortRejectedGoal<Takeoff>(goal, message);
}

bool ActionLifecycleCoordinator::abortRejectedLand(
  const std::shared_ptr<GoalHandleLand> & goal, const std::string & message) const
{
  return abortRejectedGoal<Land>(goal, message);
}

bool ActionLifecycleCoordinator::abortRejectedEnableOffboard(
  const std::shared_ptr<GoalHandleEnableOffboard> & goal, const std::string & message) const
{
  return abortRejectedGoal<EnableOffboard>(goal, message);
}

bool ActionLifecycleCoordinator::abortRejectedDisableOffboard(
  const std::shared_ptr<GoalHandleDisableOffboard> & goal, const std::string & message) const
{
  return abortRejectedGoal<DisableOffboard>(goal, message);
}

bool ActionLifecycleCoordinator::abortRejectedArm(
  const std::shared_ptr<GoalHandleArm> & goal, const std::string & message) const
{
  return abortRejectedGoal<Arm>(goal, message);
}

bool ActionLifecycleCoordinator::abortRejectedDisarm(
  const std::shared_ptr<GoalHandleDisarm> & goal, const std::string & message) const
{
  return abortRejectedGoal<Disarm>(goal, message);
}

}  // namespace mullet_core::state_machine_detail
