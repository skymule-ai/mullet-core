#ifndef MULLET_CORE_PX4_VEHICLE_INSTANCE_HPP_
#define MULLET_CORE_PX4_VEHICLE_INSTANCE_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

namespace mullet_core
{

// Inclusive bounds for PX4 vehicle instance IDs.
constexpr int kMinVehicleInstance = 0;
constexpr int kMaxVehicleInstance = 254;

// Validates that an instance can map to a PX4 system ID.
//
// Args:
//   vehicle_instance: Zero-based PX4 vehicle instance.
//
// Throws:
//   std::out_of_range: The instance is outside [kMinVehicleInstance, kMaxVehicleInstance].
inline void validateVehicleInstance(int vehicle_instance)
{
  if (vehicle_instance < kMinVehicleInstance || vehicle_instance > kMaxVehicleInstance) {
    throw std::out_of_range(
      "vehicle_instance must be in [" + std::to_string(kMinVehicleInstance) + ", " +
      std::to_string(kMaxVehicleInstance) +
      "] so target_system maps to PX4 system IDs [1, 255]");
  }
}

// Maps a valid vehicle instance to its PX4 target system ID.
//
// Args:
//   vehicle_instance: Zero-based PX4 vehicle instance.
//
// Returns:
//   The target system ID in [1, 255].
//
// Throws:
//   std::out_of_range: The instance is outside [kMinVehicleInstance, kMaxVehicleInstance].
inline std::uint8_t targetSystemFromVehicleInstance(int vehicle_instance)
{
  validateVehicleInstance(vehicle_instance);
  return static_cast<std::uint8_t>(vehicle_instance + 1);
}

}  // namespace mullet_core

#endif  // MULLET_CORE_PX4_VEHICLE_INSTANCE_HPP_
