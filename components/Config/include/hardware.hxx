/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef HARDWARE_HXX_
#define HARDWARE_HXX_

#include "sdkconfig.h"
#include <dcc/DccOutput.hxx>
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <utils/GpioInitializer.hxx>
#include <hal/gpio_types.h>
#include <hal/rmt_types.h>

#if CONFIG_OLED_RESET_PIN
/// OLED Reset signal pin.
GPIO_PIN(OLED_RESET, GpioOutputSafeHigh, CONFIG_OLED_RESET_PIN);
#else
/// OLED Reset signal pin.
typedef DummyPin OLED_RESET_Pin;
#endif

/// DCC Signal output pin
GPIO_PIN(DCC_SIGNAL, GpioOutputSafeLow, CONFIG_DCC_TRACK_SIGNAL_PIN);
#if CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG || CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY
/// Enables the OPS track output h-bridge.
GPIO_PIN(OPS_ENABLE, GpioOutputSafeLow, CONFIG_OPS_TRACK_ENABLE_PIN);
#else // OPS DISABLED
/// Enables the OPS track output h-bridge.
typedef DummyPin OPS_ENABLE_Pin;
#endif // DCC_TRACK_OUTPUTS_OPS_AND_PROG || DCC_TRACK_OUTPUTS_OPS_ONLY

#if CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG || CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY
/// Enables the PROG track output h-bridge.
GPIO_PIN(PROG_ENABLE, GpioOutputSafeLow, CONFIG_PROG_TRACK_ENABLE_PIN);
#else // PROG DISABLED
/// Enables the PROG track output h-bridge.
typedef DummyPin PROG_ENABLE_Pin;
#endif // DCC_TRACK_OUTPUTS_OPS_AND_PROG || DCC_TRACK_OUTPUTS_PROG_ONLY

#if CONFIG_RAILCOM_DISABLED
/// RailCom detector enable pin.
typedef DummyPin RAILCOM_TRIGGER_Pin;
#else // !CONFIG_RAILCOM_DISABLED
/// RailCom detector enable pin.
GPIO_PIN(RAILCOM_TRIGGER, GpioOutputSafeHigh, CONFIG_RAILCOM_TRIGGER_PIN);
#endif // CONFIG_RAILCOM_DISABLED

#if CONFIG_FACTORY_RESET_PIN != -1
/// Factory reset button pin.
GPIO_PIN(FACTORY_RESET_BUTTON, GpioInputPU, CONFIG_FACTORY_RESET_PIN);
#else
/// Virtual factory reset button pin.
typedef DummyPinWithReadHigh FACTORY_RESET_BUTTON_Pin;
#endif

#if CONFIG_BOOTLOADER_PIN != -1
/// Bootloader Request button pin.
GPIO_PIN(BOOTLOADER_BUTTON, GpioInputPU, CONFIG_BOOTLOADER_PIN);
#else
/// Virtual bootloader request button pin.
typedef DummyPinWithReadHigh BOOTLOADER_BUTTON_Pin;
#endif

/// GPIO Pin initializer.
typedef GpioInitializer<OLED_RESET_Pin, RAILCOM_TRIGGER_Pin,
                        DCC_SIGNAL_Pin, OPS_ENABLE_Pin, PROG_ENABLE_Pin,
                        FACTORY_RESET_BUTTON_Pin, BOOTLOADER_BUTTON_Pin> GpioInit;

// Sanity check that the preamble bits are within the supported range.
#ifndef CONFIG_OPS_DCC_PREAMBLE_BITS
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is not defined and has been set to 11.
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS < 11
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is set too low and has been reset to 11.
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS > 20
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is set too high and has been reset to 20.
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 20
#endif

// Sanity check that the preamble bits are within range.
#ifndef CONFIG_PROG_DCC_PREAMBLE_BITS
#define CONFIG_PROG_DCC_PREAMBLE_BITS 22
#elif CONFIG_PROG_DCC_PREAMBLE_BITS < 22 || CONFIG_PROG_DCC_PREAMBLE_BITS > 75
#undef CONFIG_PROG_DCC_PREAMBLE_BITS
#define CONFIG_PROG_DCC_PREAMBLE_BITS 22
#endif

#ifndef CONFIG_RAILCOM_DATA_PIN
#define CONFIG_RAILCOM_DATA_PIN -1
#endif

#ifndef CONFIG_RAILCOM_DIRECTION_PIN
#define CONFIG_RAILCOM_DIRECTION_PIN -1
#endif

struct DccHwDefs
{
  /// DCC signal output pin.
  using DCC_SIGNAL_Pin = ::DCC_SIGNAL_Pin;
  static constexpr gpio_num_t DCC_SIGNAL_PIN_NUM =
    (gpio_num_t)CONFIG_DCC_TRACK_SIGNAL_PIN;

  /// The number of preamble bits to send exclusive of end of packet '1' bit
  /// for DCC packets that are not service mode.
  static constexpr uint32_t DCC_PREAMBLE_BITS = CONFIG_OPS_DCC_PREAMBLE_BITS;

  /// The number of preamble bits to send exclusive of end of packet '1' bit
  /// for service mode DCC packets.
  static constexpr uint32_t DCC_SERVICE_MODE_PREAMBLE_BITS =
    CONFIG_PROG_DCC_PREAMBLE_BITS;

  /// RMT Channel to use for the DCC Signal output
  static const rmt_channel_t RMT_CHANNEL = RMT_CHANNEL_0;

  /// number of outgoing messages we can queue
  static const size_t PACKET_Q_SIZE = CONFIG_PACKET_QUEUE_SIZE;

  /// RailCom detector enable pin
  using RAILCOM_TRIGGER_Pin = InvertedGpio<::RAILCOM_TRIGGER_Pin>;
  typedef DummyPin PROG_RAILCOM_TRIGGER_Pin;
  /// RailCom data pin
  static constexpr gpio_num_t RAILCOM_DATA_PIN =
    (gpio_num_t)CONFIG_RAILCOM_DATA_PIN;
  /// RailCom direction pin
  static constexpr gpio_num_t RAILCOM_DIRECTION_PIN =
    (gpio_num_t)CONFIG_RAILCOM_DIRECTION_PIN;

  /// OPS track output
  using Output1 =
    DccOutputHwReal<DccOutput::TRACK, OPS_ENABLE_Pin, RAILCOM_TRIGGER_Pin, 0,
                    0, 0>;
  /// PROG track output
  using Output2 =
    DccOutputHwReal<DccOutput::PGM, PROG_ENABLE_Pin, PROG_RAILCOM_TRIGGER_Pin,
                    0, 0, 0>;
  using Output3 = DccOutputHwDummy<DccOutput::LCC>;
}; // DccHwDefs

#endif // HARDWARE_HXX_
