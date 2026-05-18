#include "drone_state_machine_helpers.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace mullet_core::state_machine_detail
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTolerance = 1e-6F;

const std::vector<DroneState> & allStates()
{
  static const std::vector<DroneState> states{
    DroneState::kInit,
    DroneState::kReady,
    DroneState::kTakeoffRequested,
    DroneState::kWarmupSetpoints,
    DroneState::kRequestOffboard,
    DroneState::kRequestArm,
    DroneState::kTakeoff,
    DroneState::kHover,
    DroneState::kCommand,
    DroneState::kCancelingToLand,
    DroneState::kLanding,
    DroneState::kTakeoffFailed,
    DroneState::kTakeoffCanceled,
    DroneState::kFailsafeHold,
    DroneState::kDisarmed,
  };
  return states;
}

TEST(StateMachineHelpers, RetryDueAcceptsZeroLastTime)
{
  const rclcpp::Time now(10, 0, RCL_ROS_TIME);
  const rclcpp::Time last(0, 0, RCL_ROS_TIME);

  EXPECT_TRUE(retryDue(now, last, 1.0));
}

TEST(StateMachineHelpers, RetryDueChecksIntervalBoundary)
{
  const rclcpp::Time last(10, 0, RCL_ROS_TIME);

  EXPECT_FALSE(retryDue(rclcpp::Time(10, 999999999, RCL_ROS_TIME), last, 1.0));
  EXPECT_TRUE(retryDue(rclcpp::Time(11, 0, RCL_ROS_TIME), last, 1.0));
  EXPECT_TRUE(retryDue(rclcpp::Time(11, 1, RCL_ROS_TIME), last, 1.0));
}

TEST(StateMachineHelpers, FiniteInRangeAcceptsInclusiveBounds)
{
  EXPECT_TRUE(finiteInRange(1.0, 1.0, 2.0));
  EXPECT_TRUE(finiteInRange(1.5, 1.0, 2.0));
  EXPECT_TRUE(finiteInRange(2.0, 1.0, 2.0));
}

TEST(StateMachineHelpers, FiniteInRangeRejectsInvalidValues)
{
  EXPECT_FALSE(finiteInRange(0.99, 1.0, 2.0));
  EXPECT_FALSE(finiteInRange(2.01, 1.0, 2.0));
  EXPECT_FALSE(finiteInRange(std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0));
  EXPECT_FALSE(finiteInRange(std::numeric_limits<double>::infinity(), 1.0, 2.0));
  EXPECT_FALSE(finiteInRange(-std::numeric_limits<double>::infinity(), 1.0, 2.0));
}

TEST(StateMachineHelpers, WrapPiPreservesBoundaryConvention)
{
  EXPECT_NEAR(wrapPi(0.0F), 0.0F, kTolerance);
  EXPECT_NEAR(wrapPi(kPi), kPi, kTolerance);
  EXPECT_NEAR(wrapPi(-kPi), kPi, kTolerance);
  EXPECT_NEAR(wrapPi(3.0F * kPi), kPi, kTolerance);
  EXPECT_NEAR(wrapPi(-3.0F * kPi), kPi, kTolerance);
}

TEST(StateMachineHelpers, WrapPiOutputUsesNegativePiExclusivePositivePiInclusiveRange)
{
  for (const float angle : {-10.0F, -2.0F * kPi, -0.1F, 0.0F, 0.1F, 2.0F * kPi, 10.0F}) {
    const auto wrapped = wrapPi(angle);
    EXPECT_GT(wrapped, -kPi);
    EXPECT_LE(wrapped, kPi);
  }
}

TEST(StateMachineHelpers, ActiveTakeoffStateMatchesExpectedStates)
{
  for (const auto state : allStates()) {
    const bool expected =
      state == DroneState::kTakeoffRequested ||
      state == DroneState::kWarmupSetpoints ||
      state == DroneState::kRequestOffboard ||
      state == DroneState::kRequestArm ||
      state == DroneState::kTakeoff;

    EXPECT_EQ(isActiveTakeoffState(state), expected);
  }
}

TEST(StateMachineHelpers, LandActionAllowedMatchesExpectedStates)
{
  for (const auto state : allStates()) {
    const bool expected =
      state == DroneState::kHover ||
      state == DroneState::kCommand ||
      isActiveTakeoffState(state);

    EXPECT_EQ(isLandActionAllowedFromState(state), expected);
  }
}

TEST(StateMachineHelpers, LandActionAllowedRelationshipDoesNotDrift)
{
  for (const auto state : allStates()) {
    EXPECT_EQ(
      isLandActionAllowedFromState(state),
      state == DroneState::kHover || state == DroneState::kCommand ||
      isActiveTakeoffState(state));
  }
}

}  // namespace
}  // namespace mullet_core::state_machine_detail
