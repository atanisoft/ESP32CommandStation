/*
 * SPDX-FileCopyrightText: 2016 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _LOCODB_DEFS_HXX_
#define _LOCODB_DEFS_HXX_

#include <utils/logging.h>
#include <dcc/Address.hxx>
#include <dcc/Defs.hxx>

namespace locodb
{

/// Maximum number of supported functions to expose in CDI configuration
/// elements.
static constexpr size_t MAX_LOCO_FUNCTIONS = 29;

/// Constant identifiers for locomotive functions.
enum Function : uint8_t
{
  /// Function purpose is not known.
  UNKNOWN = 127,

  /// Function is treated as momentary.
  MOMENTARY = 128,

  /// Empty eeprom will have these bytes.
  UNINITIALIZED = 255,

  /// Function does not exist.
  NONEXISTANT = 0,

  /// Function is used for the headlight.
  HEADLIGHT = 1,

  /// Function is used for engine sound (?).
  ENGINE = 4,

  /// Function is used tor an announcement.
  ANNOUNCEMENT = 6 | MOMENTARY,

  /// Function is used to enable / disable shunting mode.
  SHUNTING_MODE = 7,

  /// Function is used to enable / disable momentum.
  MOMENTUM = 8,

  /// Function is used to trigger the uncoupler sound.
  UNCOUPLE = 9 | MOMENTARY,

  /// Function is used to enable / disable smoke generator.
  SMOKE = 10,

  /// Function is used to raise / lower pantograph.
  PANTOGRAPH = 11,

  /// Function is used for a rear headlight.
  FAR_LIGHT = 12,

  /// Function is used to enable / disable the bell sound.
  BELL = 13,

  /// Function is used to trigger the horn sound.
  HORN = 14 | MOMENTARY,

  /// Function is used to trigger the whistle sound.
  WHISTLE = 15 | MOMENTARY,

  /// Function is used as a generic light.
  LIGHT = 16,

  /// Function is used to mute / unmute all sounds.
  MUTE = 17
};

/// Constant identifiers used to determine how to drive the locomotive.
enum DriveMode : uint8_t
{
  DEFAULT = 0,
  FAKE_DRIVE = 1,
  OLCBUSER = 1,

  /// Value for testing whether the protocol is a Markin-Motorola protocol
  /// variant.
  MARKLIN_ANY = 0b00100,

  /// value for testing whether the protocol is a DCC variant
  DCC_ANY = 0b01000,

  /// Acquisition for a Marklin locomotive with default setting.
  MARKLIN_DEFAULT = MARKLIN_ANY,
  /// Force MM protocol version 1 (F0 only).
  MARKLIN_OLD = MARKLIN_ANY | 1,
  /// Force MM protocol version 2 (F0-F4).
  MARKLIN_NEW = MARKLIN_ANY | 2,
  /// Force MM protocol version 2 with subsequent address for more functions
  /// (F0-F8).
  MARKLIN_TWOADDR = MARKLIN_ANY | 3,
  /// Alias for MFX locmotives (to be driven with Marklin v2 protocol for now).
  MFX = MARKLIN_NEW,

  /// Acquisition for DCC locomotive with default settings.
  DCC_DEFAULT = DCC_ANY,
  /// Force long address for DCC. If clear, uses default address type by number.
  DCC_LONG_ADDRESS = 0b00100,
  /// Force long address for DCC with DEFAULT/ANY speed step mode.
  DCC_DEFAULT_LONG_ADDRESS = DCC_DEFAULT | DCC_LONG_ADDRESS,
  /// Unpecified / default speed step setting.
  DCC_DEFAULT_SS = DCC_DEFAULT,
  /// Force 14 SS mode
  DCC_14 = DCC_ANY | 1,
  /// Force 28 SS mode
  DCC_28 = DCC_ANY | 2,
  /// Force 128 SS mode
  DCC_128 = DCC_ANY | 3,
  /// Force 14 SS mode & long address (this is meaningless).
  DCC_14_LONG_ADDRESS = DCC_14 | DCC_LONG_ADDRESS,
  /// Force 28 SS mode & long address.
  DCC_28_LONG_ADDRESS = DCC_28 | DCC_LONG_ADDRESS,
  /// Force 128 SS mode & long address.
  DCC_128_LONG_ADDRESS = DCC_128 | DCC_LONG_ADDRESS,
  
  /// Mask for testing whether the protocol is a Markin-Motorola protocol
  /// variant.
  MARKLIN_ANY_MASK = 0b11100,

  /// mask for testing whether the protocol is a DCC variant
  DCC_ANY_MASK = 0b11000,

  /// Mask for the Marklin protocol version speed step setting.
  MARKLIN_VERSION_MASK = 0b00011,

  /// Bit mask for the protocol field only.
  PROTOCOL_MASK = 0b11111,

  /// Mask for the DCC speed step setting.
  DCC_SS_MASK = 0b00011,

};

/// Converts a DriveMode bit mask and a legacy address into a
/// @ref TrainAddressType enum.
/// @param mode the legacy drive mode (e.g. from a TrainDb entry or from a
/// search query)
/// @param address is the legacy address.
/// @return an enum value which together with the address uniquely represents
/// an addressable entity on the track. May return UNSPECIFIED if DriveMode ==
/// DEFAULT (usually a query did not specify any restriction) or UNSUPPORTED if
/// we did not recognize the code in the DriveMode bitfield.
static inline dcc::TrainAddressType drive_mode_to_address_type(DriveMode mode,
                                                               uint32_t address)
{
  if (mode == DriveMode::DEFAULT)
  {
    return dcc::TrainAddressType::UNSPECIFIED;
  }
  if ((mode & DriveMode::MARKLIN_ANY_MASK) == DriveMode::MARKLIN_ANY)
  {
    return dcc::TrainAddressType::MM;
  }
  if ((mode & DriveMode::DCC_ANY_MASK) == DriveMode::DCC_ANY)
  {
    if ((mode & DriveMode::DCC_LONG_ADDRESS) ||
        (address > dcc::DccShortAddress::ADDRESS_MAX))
    {
      return dcc::TrainAddressType::DCC_LONG_ADDRESS;
    }
    return dcc::TrainAddressType::DCC_SHORT_ADDRESS;
  }
  LOG_ERROR("Unsupported drive mode %d (0x%02x)", mode, mode);
  return dcc::TrainAddressType::UNSUPPORTED;
}

/// Converts a DriveMode bit mask down to a protocol enumeration, i.e. DCC,
/// Marklin or OpenLCB.
/// @param mode the detailed mode bit field.
/// @return a stripped down mode bit field which does not specify any details
/// about the protocol variant.
static inline DriveMode drive_mode_to_protocol(DriveMode mode)
{
  if (mode == DriveMode::DEFAULT ||
      mode == DriveMode::OLCBUSER)
  {
    return mode;
  }
  if ((mode & MARKLIN_ANY_MASK) == DriveMode::MARKLIN_ANY)
  {
    return DriveMode::MARKLIN_ANY;
  }
  if ((mode & DCC_ANY_MASK) == DriveMode::DCC_ANY)
  {
    return DriveMode::DCC_ANY;
  }
  LOG_ERROR("Unknown DCC Mode %d", (int)mode);
  // We return the value unchanged.
  return mode;
}

/// Convert a function label to a string value.
///
/// @param function_type Type value for the function.
/// @return String representation of the value or null if no known value.
static inline const char* function_to_string(uint8_t function_type)
{
  struct Label
  {
    uint8_t fn;
    const char* label;
  };

  static const Label labels[] =
  {
    { Function::MUTE, "Mute" },
    { Function::UNKNOWN, "Unknown" },
    { Function::HEADLIGHT, "Headlight" },
    { Function::ENGINE, "Engine Sound" },
    { Function::ANNOUNCEMENT, "Announce" },
    { Function::SHUNTING_MODE, "Shunting Mode" },
    { Function::MOMENTUM, "Momentum" },
    { Function::UNCOUPLE, "Uncouple" },
    { Function::SMOKE, "Smoke" },
    { Function::PANTOGRAPH, "Pantograph" },
    { Function::FAR_LIGHT, "Far Light" },
    { Function::BELL, "Bell" },
    { Function::HORN, "Horn" },
    { Function::WHISTLE, "Whistle" },
    { Function::LIGHT, "Light" },
    { Function::UNINITIALIZED, "Uninitialized" },
    { Function::NONEXISTANT, nullptr }
  };

  const Label* r = labels;
  while (r->fn)
  {
    if ((r->fn & ~Function::MOMENTARY) == (function_type & ~Function::MOMENTARY))
    {
      return r->label;
    }
    ++r;
  }
  return nullptr;
}

/// Convert a function label to a string value.
///
/// @param mode Type value for the function.
/// @return String representation of the value or null if no known value.
static inline const char* drive_mode_to_string(uint8_t mode)
{
  struct Label
  {
    uint8_t fn;
    const char* label;
  };

  static const Label labels[] =
  {
    { DriveMode::DCC_DEFAULT, "DCC (auto speed step)" },
    { DriveMode::DCC_DEFAULT_LONG_ADDRESS, "DCC (auto speed step, long address)" },
    { DriveMode::DCC_ANY, "DCC (auto speed step)" },
    { DriveMode::DCC_14, "DCC (14 speed step)" },
    { DriveMode::DCC_14_LONG_ADDRESS, "DCC (14 speed step, long address)" },
    { DriveMode::DCC_28, "DCC (28 speed step)" },
    { DriveMode::DCC_28_LONG_ADDRESS, "DCC (28 speed step, long address)" },
    { DriveMode::DCC_128, "DCC (128 speed step)" },
    { DriveMode::DCC_128_LONG_ADDRESS, "DCC (128 speed step, long address)" },
    { DriveMode::MARKLIN_DEFAULT, "Marklin (default)" },
    { DriveMode::MARKLIN_OLD, "Marklin v1" },
    { DriveMode::MARKLIN_NEW, "Marklin v2" },
    { DriveMode::MARKLIN_TWOADDR, "Marklin v2 (two address)" },
    { PROTOCOL_MASK, nullptr }
  };

  const Label* r = labels;
  while (r->fn)
  {
    if (r->fn == mode)
    {
      return r->label;
    }
    ++r;
  }
  return nullptr;
}

} // namespace locodb

#endif // _LOCODB_DEFS_HXX_

