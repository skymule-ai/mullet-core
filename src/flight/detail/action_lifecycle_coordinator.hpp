#ifndef MULLET_CORE_SRC_FLIGHT_DETAIL_ACTION_LIFECYCLE_COORDINATOR_HPP_
#define MULLET_CORE_SRC_FLIGHT_DETAIL_ACTION_LIFECYCLE_COORDINATOR_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp_action/rclcpp_action.hpp>

#include "mullet_interfaces/action/land.hpp"
#include "mullet_interfaces/action/return_home.hpp"
#include "mullet_interfaces/action/takeoff.hpp"

namespace mullet_core::state_machine_detail
{

enum class ActionTerminal
{
  kSucceeded,
  kAborted,
  kCanceled,
};

// Owns the single active goal and cancellation state for one ROS action type.
template<typename ActionT>
class ActionSlot
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  bool accept(std::shared_ptr<GoalHandle> goal)
  {
    if (active_goal_) {
      return false;
    }
    active_goal_ = std::move(goal);
    cancel_requested_ = false;
    return true;
  }

  bool active() const {return active_goal_ != nullptr;}

  bool isActive(const std::shared_ptr<GoalHandle> & goal) const
  {
    return goal == active_goal_;
  }

  void requestCancel() {cancel_requested_ = true;}
  bool cancelRequested() const {return cancel_requested_;}

  void publish(const std::shared_ptr<typename ActionT::Feedback> & feedback) const
  {
    if (active_goal_) {
      active_goal_->publish_feedback(feedback);
    }
  }

  bool succeed(const std::string & message)
  {
    return finish(ActionTerminal::kSucceeded, message);
  }

  bool abort(const std::string & message)
  {
    return finish(ActionTerminal::kAborted, message);
  }

  bool cancel(const std::string & message)
  {
    return finish(ActionTerminal::kCanceled, message);
  }

  static bool abortRejected(
    const std::shared_ptr<GoalHandle> & goal, const std::string & message)
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

private:
  bool finish(ActionTerminal terminal, const std::string & message)
  {
    if (!active_goal_) {
      return false;
    }

    auto result = std::make_shared<typename ActionT::Result>();
    result->success = terminal == ActionTerminal::kSucceeded;
    result->message = message;
    switch (terminal) {
      case ActionTerminal::kSucceeded:
        active_goal_->succeed(result);
        break;
      case ActionTerminal::kAborted:
        active_goal_->abort(result);
        break;
      case ActionTerminal::kCanceled:
        active_goal_->canceled(result);
        break;
    }
    active_goal_.reset();
    cancel_requested_ = false;
    return true;
  }

  std::shared_ptr<GoalHandle> active_goal_{};
  bool cancel_requested_{false};
};

// Keeps the three action slots explicit so lifecycle state remains easy to audit.
class ActionLifecycleCoordinator
{
public:
  ActionSlot<mullet_interfaces::action::Takeoff> & takeoff() {return takeoff_;}
  const ActionSlot<mullet_interfaces::action::Takeoff> & takeoff() const {return takeoff_;}
  ActionSlot<mullet_interfaces::action::Land> & land() {return land_;}
  const ActionSlot<mullet_interfaces::action::Land> & land() const {return land_;}
  ActionSlot<mullet_interfaces::action::ReturnHome> & returnHome() {return return_home_;}
  const ActionSlot<mullet_interfaces::action::ReturnHome> & returnHome() const
  {
    return return_home_;
  }

private:
  ActionSlot<mullet_interfaces::action::Takeoff> takeoff_{};
  ActionSlot<mullet_interfaces::action::Land> land_{};
  ActionSlot<mullet_interfaces::action::ReturnHome> return_home_{};
};

}  // namespace mullet_core::state_machine_detail

#endif  // MULLET_CORE_SRC_FLIGHT_DETAIL_ACTION_LIFECYCLE_COORDINATOR_HPP_
