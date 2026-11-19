#pragma once

#include <memory>
#include <string>
#include <utility>
#include <functional>

#include <rclcpp_action/rclcpp_action.hpp>

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
class ActionServer
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using Goal = typename ActionT::Goal;
  using RejectFn = std::function<const char*(const Goal&)>;
  using AcceptFn = std::function<void(const Goal&)>;

  explicit ActionServer(rclcpp::Node & node, const char * name, RejectFn reject_fn, AcceptFn accept_fn,
                        bool cancellable) :
                        name_(name),
                        logger_(node.get_logger()),
                        reject_fn_(std::move(reject_fn)),
                        accept_fn_(std::move(accept_fn)),
                        cancellable_(cancellable)
  { 
    auto handle_goal = [this](
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Goal> goal) -> rclcpp_action::GoalResponse
    {
      if (const char * reason = reject_fn_(*goal)) {
        RCLCPP_WARN(logger_, "[GOAL REJECT] %s", reason);
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](const std::shared_ptr<GoalHandle> goal_handle) -> rclcpp_action::CancelResponse
    {
      if (goal_handle != active_goal_ || !cancellable_) {
        if (!cancellable_) {
          RCLCPP_WARN(logger_, "[CANCEL REJECT] %s action is not cancellable", name_.c_str());
        } else {
          RCLCPP_WARN(logger_, "[CANCEL REJECT] Goal is not active");
        }
        return rclcpp_action::CancelResponse::REJECT;
      }
      cancel_requested_ = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle) -> void
    {
      if (!accept(goal_handle)) {
        abortRejected(goal_handle, "[ACCEPT REJECT] " + std::string(name_) + " action is already active");
        return;
      }
      accept_fn_(*goal_handle->get_goal());
    };

    action_server_ = rclcpp_action::create_server<ActionT>(
      &node, name_,
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

  ActionServer(const ActionServer &) = delete;
  ActionServer & operator=(const ActionServer &) = delete;

  ActionServer(ActionServer &&) = delete;
  ActionServer & operator=(ActionServer &&) = delete;

  void publish(const std::shared_ptr<typename ActionT::Feedback> & feedback) const
  {
    if (active_goal_) {
      active_goal_->publish_feedback(feedback);
    }
  }

  bool active() const { return active_goal_ != nullptr; }

  bool cancelRequested() const { return cancel_requested_; }

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

  bool accept(std::shared_ptr<GoalHandle> goal)
  {
    if (active_goal_) {
      return false;
    }
    active_goal_ = std::move(goal);
    cancel_requested_ = false;
    return true;
  }

  bool abortRejected(
    const std::shared_ptr<GoalHandle> & goal_handle, const std::string & message)
  {
    if (!goal_handle) {
      return false;
    }
    auto result = std::make_shared<typename ActionT::Result>();
    result->success = false;
    result->message = message;
    goal_handle->abort(result);
    return true;
  }

  std::shared_ptr<GoalHandle> active_goal_{};
  std::string name_;
  rclcpp::Logger logger_;
  RejectFn reject_fn_{};
  AcceptFn accept_fn_{};
  bool cancellable_{false};
  bool cancel_requested_{false};

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_{};
};

}  // namespace mullet_core::state_machine_detail
