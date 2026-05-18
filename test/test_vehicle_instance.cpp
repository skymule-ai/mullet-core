#include "mullet_core/vehicle_instance.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace mullet_core
{
namespace
{

TEST(VehicleInstance, MapsValidBoundsToTargetSystem)
{
  EXPECT_EQ(static_cast<int>(targetSystemFromVehicleInstance(0)), 1);
  EXPECT_EQ(static_cast<int>(targetSystemFromVehicleInstance(254)), 255);
}

TEST(VehicleInstance, RejectsValuesOutsideSafeRange)
{
  EXPECT_THROW(validateVehicleInstance(-1), std::out_of_range);
  EXPECT_THROW(validateVehicleInstance(255), std::out_of_range);
  EXPECT_THROW(
    validateVehicleInstance(std::numeric_limits<int>::max()), std::out_of_range);
}

}  // namespace
}  // namespace mullet_core
