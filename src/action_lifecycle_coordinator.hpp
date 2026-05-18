#ifndef MULLET_CORE_SRC_ACTION_LIFECYCLE_COORDINATOR_HPP_
#define MULLET_CORE_SRC_ACTION_LIFECYCLE_COORDINATOR_HPP_

#include <memory>
#include <string>

#include <rclcpp_action/rclcpp_action.hpp>

#include "mullet_core/action/land.hpp"
#include "mullet_core/action/takeoff.hpp"

namespace mullet_core::state_machine_detail
{

class ActionLifecycleCoordinator
{
public:
  using Land = mullet_core::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  using Takeoff = mullet_core::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

  bool acceptTakeoffGoal(std::shared_ptr<GoalHandleTakeoff> goal);
  bool acceptLandGoal(std::shared_ptr<GoalHandleLand> goal);

  bool hasActiveTakeoffGoal() const;
  bool hasActiveLandGoal() const;
  bool isActiveTakeoffGoal(const std::shared_ptr<GoalHandleTakeoff> & goal) const;
  bool isActiveLandGoal(const std::shared_ptr<GoalHandleLand> & goal) const;

  void requestTakeoffCancel();
  bool takeoffCancelRequested() const;

  void publishTakeoffFeedback(const std::shared_ptr<Takeoff::Feedback> & feedback) const;
  void publishLandFeedback(const std::shared_ptr<Land::Feedback> & feedback) const;

  bool succeedTakeoff(const std::string & message);
  bool abortTakeoff(const std::string & message);
  bool cancelTakeoff(const std::string & message);
  bool succeedLand(const std::string & message);
  bool abortLand(const std::string & message);
  bool abortRejectedTakeoff(
    const std::shared_ptr<GoalHandleTakeoff> & goal, const std::string & message) const;
  bool abortRejectedLand(
    const std::shared_ptr<GoalHandleLand> & goal, const std::string & message) const;

private:
  std::shared_ptr<GoalHandleTakeoff> active_takeoff_goal_{};
  std::shared_ptr<GoalHandleLand> active_land_goal_{};
  bool takeoff_cancel_requested_{false};
};

}  // namespace mullet_core::state_machine_detail

#endif  // MULLET_CORE_SRC_ACTION_LIFECYCLE_COORDINATOR_HPP_
