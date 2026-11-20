#include "flight/detail/helpers.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <optional>

#include <gtest/gtest.h>

#include "mullet_core/flight/flight_states.hpp"
#include "mullet_core/px4/topic_utils.hpp"

namespace mullet_core::state_machine_detail
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTolerance = 1e-6F;

TEST(StateMachineHelpers, RetryDueWhenNeverSent)
{
  EXPECT_TRUE(retryDue(rclcpp::Time(10, 0, RCL_ROS_TIME), std::nullopt, 1.0));
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

TEST(StateMachineHelpers, LocalToGlobalAtOriginIsTheEkfOrigin)
{
  const auto global = localToGlobal(47.0, 8.0, 400.0, 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(global.lat_deg, 47.0);
  EXPECT_DOUBLE_EQ(global.lon_deg, 8.0);
  EXPECT_DOUBLE_EQ(global.alt_m, 400.0);
}

TEST(StateMachineHelpers, LocalToGlobalMovesNorthEastAndUp)
{
  // 111.19492664 m is 0.001 degrees of arc on the 6371 km sphere; z is down, so -10 m is up.
  const auto north = localToGlobal(47.0, 8.0, 400.0, 111.19492664, 0.0, -10.0);
  EXPECT_NEAR(north.lat_deg, 47.001, 1e-9);
  EXPECT_NEAR(north.lon_deg, 8.0, 1e-12);
  EXPECT_NEAR(north.alt_m, 410.0, 1e-9);

  // The same eastward distance spans twice the longitude at 60 degrees as at the equator.
  const auto east_equator = localToGlobal(0.0, 0.0, 0.0, 0.0, 1000.0, 0.0);
  const auto east_60 = localToGlobal(60.0, 0.0, 0.0, 0.0, 1000.0, 0.0);
  EXPECT_NEAR(east_60.lon_deg, 2.0 * east_equator.lon_deg, 1e-9);
  EXPECT_NEAR(east_60.lat_deg, 60.0, 1e-12);
}

// rclcpp::Time's default constructor is explicit, so aggregates must name their Time fields.
const rclcpp::Time kT0;

TEST(FlightStates, WireStateAndActiveActionPerAlternative)
{
  EXPECT_EQ(wireState(Idle{}), DroneState::kIdle);
  EXPECT_EQ(wireState(TakingOff{.step_enter = kT0}), DroneState::kTransition);
  EXPECT_EQ(wireState(Landing{.start = kT0}), DroneState::kTransition);
  EXPECT_EQ(wireState(ReturningHome{}), DroneState::kTransition);
  EXPECT_EQ(wireState(Hold{}), DroneState::kHold);
  EXPECT_EQ(wireState(Mission{.enter = kT0}), DroneState::kMission);

  EXPECT_STREQ(activeActionName(Idle{}), "");
  EXPECT_STREQ(activeActionName(TakingOff{.step_enter = kT0}), "takeoff");
  EXPECT_STREQ(activeActionName(Landing{.start = kT0}), "land");
  EXPECT_STREQ(activeActionName(ReturningHome{}), "return_home");
  EXPECT_STREQ(activeActionName(Hold{}), "");
  EXPECT_STREQ(activeActionName(Mission{.enter = kT0}), "");
}

TEST(FlightStates, FlownPoseIsTheStreamedTarget)
{
  const Pose p{.x = 1.0F, .y = 2.0F, .z = -3.0F, .yaw = 0.5F};
  const auto fields = [](const Pose & q) { return std::array{q.x, q.y, q.z, q.yaw}; };

  EXPECT_EQ(fields(flownPose(TakingOff{.step_enter = kT0, .target = p})), fields(p));
  EXPECT_EQ(fields(flownPose(ReturningHome{.carrot = p})), fields(p));
  EXPECT_EQ(fields(flownPose(Hold{.target = p})), fields(p));
  EXPECT_EQ(fields(flownPose(Mission{.hold = p, .enter = kT0})), fields(p));
  EXPECT_EQ(fields(flownPose(Idle{})), fields(Pose{}));
  EXPECT_EQ(fields(flownPose(Landing{.start = kT0})), fields(Pose{}));
}

TEST(Px4Topic, VersionedMessagesGetTheVersionSuffix)
{
  EXPECT_EQ(
    px4Topic("/uav_1", "/fmu/out/vehicle_local_position", 1),
    "/uav_1/fmu/out/vehicle_local_position_v1");
  EXPECT_EQ(px4Topic("", "/fmu/out/vehicle_status", 1), "/fmu/out/vehicle_status_v1");
  EXPECT_EQ(px4Topic("uav_2", "/fmu/in/vehicle_command", 0), "/uav_2/fmu/in/vehicle_command");
}

}  // namespace
}  // namespace mullet_core::state_machine_detail
