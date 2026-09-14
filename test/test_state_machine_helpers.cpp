#include "flight/detail/helpers.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace mullet_core::state_machine_detail
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTolerance = 1e-6F;

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

}  // namespace
}  // namespace mullet_core::state_machine_detail
