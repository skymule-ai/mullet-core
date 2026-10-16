#include <gtest/gtest.h>

#include "mullet_core/telemetry/localization_status_publisher.hpp"

namespace
{

TEST(LocalizationStatus, OutdoorFlagsMapToGnssAndCompass)
{
  px4_msgs::msg::EstimatorStatusFlags flags;
  flags.cs_gnss_pos = true;
  flags.cs_gnss_vel = true;
  flags.cs_mag_hdg = true;
  flags.cs_baro_hgt = true;
  flags.cs_yaw_align = true;

  const auto msg = mullet_core::toLocalizationStatus(flags, "mullet1");

  EXPECT_EQ(msg.drone_id, "mullet1");
  EXPECT_TRUE(msg.gnss_pos);
  EXPECT_TRUE(msg.gnss_vel);
  EXPECT_TRUE(msg.mag_hdg);
  EXPECT_TRUE(msg.baro_hgt);
  EXPECT_TRUE(msg.yaw_aligned);
  EXPECT_FALSE(msg.ev_pos);
  EXPECT_FALSE(msg.ev_vel);
  EXPECT_FALSE(msg.ev_yaw);
  EXPECT_FALSE(msg.rng_hgt);
  EXPECT_FALSE(msg.dead_reckoning);
}

TEST(LocalizationStatus, IndoorFlagsMapToVisionOnly)
{
  px4_msgs::msg::EstimatorStatusFlags flags;
  flags.cs_ev_pos = true;
  flags.cs_ev_vel = true;
  flags.cs_ev_yaw = true;
  flags.cs_rng_hgt = true;
  flags.cs_inertial_dead_reckoning = true;

  const auto msg = mullet_core::toLocalizationStatus(flags, "mullet1");

  EXPECT_TRUE(msg.ev_pos);
  EXPECT_TRUE(msg.ev_vel);
  EXPECT_TRUE(msg.ev_yaw);
  EXPECT_TRUE(msg.rng_hgt);
  EXPECT_TRUE(msg.dead_reckoning);
  EXPECT_FALSE(msg.gnss_pos);
  EXPECT_FALSE(msg.mag_hdg);
  EXPECT_FALSE(msg.yaw_aligned);
}

TEST(LocalizationStatus, In3dMagFusionCountsAsCompass)
{
  px4_msgs::msg::EstimatorStatusFlags flags;
  flags.cs_mag_3d = true;  // EKF2_MAG_TYPE 0 in flight

  EXPECT_TRUE(mullet_core::toLocalizationStatus(flags, "mullet1").mag_hdg);
}

}  // namespace
