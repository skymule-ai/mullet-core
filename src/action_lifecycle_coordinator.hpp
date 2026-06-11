#ifndef MULLET_CORE_SRC_ACTION_LIFECYCLE_COORDINATOR_HPP_
#define MULLET_CORE_SRC_ACTION_LIFECYCLE_COORDINATOR_HPP_

#include <memory>
#include <string>

#include <rclcpp_action/rclcpp_action.hpp>

#include "mullet_interfaces/action/arm.hpp"
#include "mullet_interfaces/action/disable_offboard.hpp"
#include "mullet_interfaces/action/disarm.hpp"
#include "mullet_interfaces/action/enable_offboard.hpp"
#include "mullet_interfaces/action/land.hpp"
#include "mullet_interfaces/action/takeoff.hpp"

namespace mullet_core::state_machine_detail
{

class ActionLifecycleCoordinator
{
public:
  using DisableOffboard = mullet_interfaces::action::DisableOffboard;
  using GoalHandleDisableOffboard = rclcpp_action::ServerGoalHandle<DisableOffboard>;
  using Arm = mullet_interfaces::action::Arm;
  using GoalHandleArm = rclcpp_action::ServerGoalHandle<Arm>;
  using Disarm = mullet_interfaces::action::Disarm;
  using GoalHandleDisarm = rclcpp_action::ServerGoalHandle<Disarm>;
  using EnableOffboard = mullet_interfaces::action::EnableOffboard;
  using GoalHandleEnableOffboard = rclcpp_action::ServerGoalHandle<EnableOffboard>;
  using Land = mullet_interfaces::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  using Takeoff = mullet_interfaces::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;

  bool acceptTakeoffGoal(std::shared_ptr<GoalHandleTakeoff> goal);
  bool acceptLandGoal(std::shared_ptr<GoalHandleLand> goal);
  bool acceptEnableOffboardGoal(std::shared_ptr<GoalHandleEnableOffboard> goal);
  bool acceptDisableOffboardGoal(std::shared_ptr<GoalHandleDisableOffboard> goal);
  bool acceptArmGoal(std::shared_ptr<GoalHandleArm> goal);
  bool acceptDisarmGoal(std::shared_ptr<GoalHandleDisarm> goal);

  bool hasActiveTakeoffGoal() const;
  bool hasActiveLandGoal() const;
  bool hasActiveEnableOffboardGoal() const;
  bool hasActiveDisableOffboardGoal() const;
  bool hasActiveArmGoal() const;
  bool hasActiveDisarmGoal() const;
  bool isActiveTakeoffGoal(const std::shared_ptr<GoalHandleTakeoff> & goal) const;
  bool isActiveLandGoal(const std::shared_ptr<GoalHandleLand> & goal) const;
  bool isActiveEnableOffboardGoal(
    const std::shared_ptr<GoalHandleEnableOffboard> & goal) const;
  bool isActiveDisableOffboardGoal(
    const std::shared_ptr<GoalHandleDisableOffboard> & goal) const;
  bool isActiveArmGoal(const std::shared_ptr<GoalHandleArm> & goal) const;
  bool isActiveDisarmGoal(const std::shared_ptr<GoalHandleDisarm> & goal) const;

  void requestTakeoffCancel();
  bool takeoffCancelRequested() const;
  void requestEnableOffboardCancel();
  bool enableOffboardCancelRequested() const;
  void requestDisableOffboardCancel();
  bool disableOffboardCancelRequested() const;
  void requestArmCancel();
  bool armCancelRequested() const;
  void requestDisarmCancel();
  bool disarmCancelRequested() const;

  void publishTakeoffFeedback(const std::shared_ptr<Takeoff::Feedback> & feedback) const;
  void publishLandFeedback(const std::shared_ptr<Land::Feedback> & feedback) const;
  void publishEnableOffboardFeedback(
    const std::shared_ptr<EnableOffboard::Feedback> & feedback) const;
  void publishDisableOffboardFeedback(
    const std::shared_ptr<DisableOffboard::Feedback> & feedback) const;
  void publishArmFeedback(const std::shared_ptr<Arm::Feedback> & feedback) const;
  void publishDisarmFeedback(const std::shared_ptr<Disarm::Feedback> & feedback) const;

  bool succeedTakeoff(const std::string & message);
  bool abortTakeoff(const std::string & message);
  bool cancelTakeoff(const std::string & message);
  bool succeedLand(const std::string & message);
  bool abortLand(const std::string & message);
  bool succeedEnableOffboard(const std::string & message);
  bool abortEnableOffboard(const std::string & message);
  bool cancelEnableOffboard(const std::string & message);
  bool succeedDisableOffboard(const std::string & message);
  bool abortDisableOffboard(const std::string & message);
  bool cancelDisableOffboard(const std::string & message);
  bool succeedArm(const std::string & message);
  bool abortArm(const std::string & message);
  bool cancelArm(const std::string & message);
  bool succeedDisarm(const std::string & message);
  bool abortDisarm(const std::string & message);
  bool cancelDisarm(const std::string & message);
  bool abortRejectedTakeoff(
    const std::shared_ptr<GoalHandleTakeoff> & goal, const std::string & message) const;
  bool abortRejectedLand(
    const std::shared_ptr<GoalHandleLand> & goal, const std::string & message) const;
  bool abortRejectedEnableOffboard(
    const std::shared_ptr<GoalHandleEnableOffboard> & goal, const std::string & message) const;
  bool abortRejectedDisableOffboard(
    const std::shared_ptr<GoalHandleDisableOffboard> & goal, const std::string & message) const;
  bool abortRejectedArm(
    const std::shared_ptr<GoalHandleArm> & goal, const std::string & message) const;
  bool abortRejectedDisarm(
    const std::shared_ptr<GoalHandleDisarm> & goal, const std::string & message) const;

private:
  std::shared_ptr<GoalHandleTakeoff> active_takeoff_goal_{};
  std::shared_ptr<GoalHandleLand> active_land_goal_{};
  std::shared_ptr<GoalHandleEnableOffboard> active_enable_offboard_goal_{};
  std::shared_ptr<GoalHandleDisableOffboard> active_disable_offboard_goal_{};
  std::shared_ptr<GoalHandleArm> active_arm_goal_{};
  std::shared_ptr<GoalHandleDisarm> active_disarm_goal_{};
  bool takeoff_cancel_requested_{false};
  bool enable_offboard_cancel_requested_{false};
  bool disable_offboard_cancel_requested_{false};
  bool arm_cancel_requested_{false};
  bool disarm_cancel_requested_{false};
};

}  // namespace mullet_core::state_machine_detail

#endif  // MULLET_CORE_SRC_ACTION_LIFECYCLE_COORDINATOR_HPP_
