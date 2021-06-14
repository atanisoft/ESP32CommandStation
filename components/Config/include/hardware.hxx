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
#include <driver/periph_ctrl.h>
#include <esp_rom_gpio.h>
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <utils/GpioInitializer.hxx>
#include <hal/gpio_types.h>
#include <hal/rmt_types.h>
#include <hal/timer_types.h>
#include <hal/uart_types.h>
#include <soc/dport_reg.h>
#include <soc/periph_defs.h>
#include <soc/uart_struct.h>
#include <soc/uart_reg.h>
#include <soc/timer_periph.h>

// Validate configured pins are defined and if not disable the feature.

#ifndef CONFIG_RAILCOM_TRIGGER_PIN
#define CONFIG_RAILCOM_TRIGGER_PIN -1
#undef CONFIG_RAILCOM_DISABLED
#define CONFIG_RAILCOM_DISABLED 1
#endif

#ifndef CONFIG_RAILCOM_DATA_PIN
#define CONFIG_RAILCOM_DATA_PIN -1
#endif

#ifndef CONFIG_RAILCOM_DIRECTION_PIN
#define CONFIG_RAILCOM_DIRECTION_PIN -1
#endif

#ifndef CONFIG_OLED_RESET_PIN
#define CONFIG_OLED_RESET_PIN -1
#endif

#ifndef TEMPSENSOR_ADC_CHANNEL
#define TEMPSENSOR_ADC_CHANNEL -1
#endif

// Sanity check that the preamble bits are within the supported range.
#ifndef CONFIG_OPS_DCC_PREAMBLE_BITS
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is not defined and has been set to 11.
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS < 11
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is set too low and has been reset to 11.
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS < 16 && !defined(CONFIG_RAILCOM_DISABLED)
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is set too low and has been reset to 16.
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 16
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

#if CONFIG_OLED_RESET_PIN != -1
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

#if CONFIG_RAILCOM_DISABLED || CONFIG_RAILCOM_TRIGGER_PIN == -1
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


/// RailCom hardware definition
struct RailComHwDefs
{
#if CONFIG_RAILCOM_FULL
#if CONFIG_RAILCOM_UART1
  static constexpr uart_port_t UART = 1;
  static constexpr uart_dev_t *UART_BASE = &UART1;
  static constexpr periph_module_t UART_PERIPH = PERIPH_UART1_MODULE;
  static constexpr int UART_ISR_SOURCE = ETS_UART1_INTR_SOURCE;
  static constexpr uint32_t UART_MATRIX_IDX = U1RXD_IN_IDX;
  static constexpr uint32_t UART_CLOCK_EN_BIT = DPORT_UART1_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = DPORT_UART1_RST;
#elif CONFIG_RAILCOM_UART2
  static constexpr uart_port_t UART = 2;
  static constexpr uart_dev_t *UART_BASE = &UART2;
  static constexpr periph_module_t UART_PERIPH = PERIPH_UART2_MODULE;
  static constexpr int UART_ISR_SOURCE = ETS_UART2_INTR_SOURCE;
  static constexpr uint32_t UART_MATRIX_IDX = U2RXD_IN_IDX;
  static constexpr uint32_t UART_CLOCK_EN_BIT = DPORT_UART2_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = DPORT_UART2_RST;
#else
  #error Unsupported UART selected for OPS RailCom!
#endif
#endif // CONFIG_RAILCOM_FULL

  /// RailCom detector enable pin
  using RAILCOM_TRIGGER_Pin = ::RAILCOM_TRIGGER_Pin;
  /// RailCom data pin
  static constexpr gpio_num_t RAILCOM_DATA_PIN =
    (gpio_num_t)CONFIG_RAILCOM_DATA_PIN;
  /// RailCom direction pin
  static constexpr gpio_num_t RAILCOM_DIRECTION_PIN =
    (gpio_num_t)CONFIG_RAILCOM_DIRECTION_PIN;

  /// Number of RailCom packets to queue
  static constexpr size_t PACKET_Q_SIZE = CONFIG_RAILCOM_FEEDBACK_QUEUE;

  static void hw_init()
  {
#if CONFIG_RAILCOM_FULL
    // initialize the UART
    periph_module_enable(UART_PERIPH);
    gpio_pad_select_gpio(RAILCOM_DATA_PIN);
    esp_rom_gpio_connect_in_signal(RAILCOM_DATA_PIN, UART_MATRIX_IDX, false);
#endif // CONFIG_RAILCOM_FULL
  }

  static constexpr timg_dev_t *TIMER_BASE = &TIMERG0;
  static constexpr timer_idx_t TIMER_IDX = TIMER_0;
  static constexpr timer_group_t TIMER_GRP = TIMER_GROUP_0;
  static constexpr periph_module_t TIMER_PERIPH = PERIPH_TIMG0_MODULE;
  static constexpr int TIMER_ISR_SOURCE = ETS_TG0_T0_LEVEL_INTR_SOURCE + TIMER_IDX;

  /// Number of microseconds to wait after the final packet bit completes
  /// before disabling the ENABLE pin on the h-bridge.
  static constexpr uint32_t RAILCOM_START_PHASE1_DELAY_USEC = 1;

  /// Number of microseconds to wait after RAILCOM_PHASE1_DELAY_USEC before
  /// starting the cut-out period.
  static constexpr uint32_t RAILCOM_START_PHASE2_DELAY_USEC = 1;

  /// Number of microseconds to wait at the end of the cut-out period.
  static constexpr uint32_t RAILCOM_STOP_DELAY_USEC = 1;

  /// Number of microseconds to wait for railcom data on channel 1.
  static constexpr uint32_t RAILCOM_MAX_READ_DELAY_CH_1 =
    177 - RAILCOM_START_PHASE1_DELAY_USEC - RAILCOM_START_PHASE2_DELAY_USEC;

  /// Number of microseconds to wait for railcom data on channel 2.
  static constexpr uint32_t RAILCOM_MAX_READ_DELAY_CH_2 =
    454 - RAILCOM_MAX_READ_DELAY_CH_1 - RAILCOM_STOP_DELAY_USEC;
}; // RailComHwDefs

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

  /// RMT Channel to use for the DCC Signal output.
  static const rmt_channel_t RMT_CHANNEL = RMT_CHANNEL_0;

  /// Number of outgoing DCC packets to allow in the queue.
  static const size_t PACKET_Q_SIZE = CONFIG_PACKET_QUEUE_SIZE;

  /// Track Booster output.
  using InternalBoosterOutput =
    DccOutputHwReal<DccOutput::TRACK, OPS_ENABLE_Pin, RAILCOM_TRIGGER_Pin,
                    RailComHwDefs::RAILCOM_START_PHASE1_DELAY_USEC,
                    RailComHwDefs::RAILCOM_START_PHASE2_DELAY_USEC,
                    RailComHwDefs::RAILCOM_STOP_DELAY_USEC>;
  /// Fake output hardware for program track.
  using Output2 = DccOutputHwDummy<DccOutput::PGM>;
  /// Fake output hardware for LCC track.
  using Output3 = DccOutputHwDummy<DccOutput::LCC>;
}; // DccHwDefs

#endif // HARDWARE_HXX_
