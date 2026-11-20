#pragma once

#include <cstdint>
#include <optional>
#include <variant>
#include <rclcpp/time.hpp>

namespace mullet_core
{

template<typename... Ts> struct overloaded : Ts... { using Ts::operator()...; };

enum class DroneState : std::uint8_t
{
  kIdle,        // On the ground, or otherwise not under supervisor control.
  kTransition,  // Takeoff, land, or return home in progress.
  kHold,        // Airborne, holding a latched position.
  kMission,     // Airborne, following the external mission command stream.
};

struct Pose { float x{}, y{}, z{}, yaw{}; };   // NED
struct Idle {};
struct TakingOff {
  enum class Step : std::uint8_t { kWarmup, kRequestOffboard, kRequestArm, kAscent };
  Step step{};
  rclcpp::Time step_enter;
  std::optional<rclcpp::Time> last_command{};
  Pose target{};
  float origin_z{};
  bool use_current_yaw{false};
  bool arm_requested{false};
  std::optional<rclcpp::Time> settle_start{};
};
struct Landing {
  rclcpp::Time start;
  std::optional<rclcpp::Time> last_command{};
  bool position_mode_sent{}, timeout_reported{};
};
struct ReturningHome { Pose carrot{}; };
struct Hold { Pose target{}; };
struct Mission { Pose hold{}; rclcpp::Time enter; };
using FlightState = std::variant<Idle, TakingOff, Landing, ReturningHome, Hold, Mission>;

DroneState wireState(const FlightState &);         // the 4-value wire enum
// The position target the current state streams to PX4; fallback for currentPose().
Pose flownPose(const FlightState & state);
const char * activeActionName(const FlightState &); // "takeoff" / "land" / "return_home" / ""

// Log and heartbeat labels (src/flight/labels.cpp); each returns "UNKNOWN" for an unknown value.
const char * stateName(DroneState state);
const char * stepName(TakingOff::Step step);
const char * navStateName(std::uint8_t nav_state);
const char * armingStateName(std::uint8_t arming_state);

}  // namespace mullet_core