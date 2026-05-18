#ifndef MULLET_CORE_VEHICLE_INSTANCE_HPP_
#define MULLET_CORE_VEHICLE_INSTANCE_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

namespace mullet_core
{

constexpr int kMinVehicleInstance = 0;
constexpr int kMaxVehicleInstance = 254;

inline void validateVehicleInstance(int vehicle_instance)
{
  if (vehicle_instance < kMinVehicleInstance || vehicle_instance > kMaxVehicleInstance) {
    throw std::out_of_range(
      "vehicle_instance must be in [" + std::to_string(kMinVehicleInstance) + ", " +
      std::to_string(kMaxVehicleInstance) +
      "] so target_system maps to PX4 system IDs [1, 255]");
  }
}

inline std::uint8_t targetSystemFromVehicleInstance(int vehicle_instance)
{
  validateVehicleInstance(vehicle_instance);
  return static_cast<std::uint8_t>(vehicle_instance + 1);
}

}  // namespace mullet_core

#endif  // MULLET_CORE_VEHICLE_INSTANCE_HPP_
