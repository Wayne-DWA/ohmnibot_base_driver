#pragma once

#include <ohmnibot_base_driver/Mode.h>
#include <cstdint>

namespace eduart {
namespace robot {

enum class Mode {
  UNCONFIGURED              = 0,
  INACTIVE                  = ohmnibot_base_driver::Mode::INACTIVE,
  REMOTE_CONTROLLED         = ohmnibot_base_driver::Mode::REMOTE_CONTROLLED,
  FLEET                     = ohmnibot_base_driver::Mode::FLEET,
  MASK_UNSET_DRIVING_MODE   = ohmnibot_base_driver::Mode::MASK_UNSET_DRIVING_MODE,

  SKID_DRIVE                = ohmnibot_base_driver::Mode::SKID_DRIVE,
  MECANUM_DRIVE             = ohmnibot_base_driver::Mode::MECANUM_DRIVE,
  MASK_UNSET_KINEMATIC_MODE = ohmnibot_base_driver::Mode::MASK_UNSET_KINEMATIC_MODE,

  COLLISION_AVOIDANCE_OVERRIDE_ENABLED    = ohmnibot_base_driver::Mode::COLLISION_AVOIDANCE_OVERRIDE_ENABLED,
  COLLISION_AVOIDANCE_OVERRIDE_DISABLED   = ohmnibot_base_driver::Mode::COLLISION_AVOIDANCE_OVERRIDE_DISABLED,
  MASK_UNSET_COLLISION_AVOIDANCE_OVERRIDE = ohmnibot_base_driver::Mode::MASK_UNSET_COLLISION_AVOIDANCE_OVERRIDE,
};

inline Mode& operator|=(Mode& lhs, const Mode rhs)
{
  lhs = static_cast<Mode>(static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
  return lhs;
}

inline Mode& operator&=(Mode& lhs, const Mode rhs)
{
  lhs = static_cast<Mode>(static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs));
  return lhs;
}

inline bool operator&(const Mode lhs, const Mode rhs)
{
  return static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs);
}

} // end namespace robot
} // end namespace eduart
