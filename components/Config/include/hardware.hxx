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

#if CONFIG_ESP32CS_PCB
#include "pinmap-pcb.hxx"
#elif CONFIG_ESP32CS_L298
#include "pinmap-l298.hxx"
#elif CONFIG_ESP32CS_LMD18200
#include "pinmap-lmd18200.hxx"
#elif CONFIG_ESP32CS_BTS7960B || CONFIG_ESP32CS_BTS7960B_X2
#include "pinmap-bts7960.hxx"
#endif

#include <esp_idf_version.h>

#include <dcc/DccOutput.hxx>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
#include <esp_private/periph_ctrl.h>
#else // IDF v4.x (or earlier)
#include <driver/periph_ctrl.h>
#endif // IDF v5+
#include <driver/timer.h>
#include <esp_rom_gpio.h>
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <utils/GpioInitializer.hxx>
#include <hal/gpio_types.h>
#include <hal/rmt_types.h>
#include <hal/timer_types.h>
#include <hal/uart_types.h>
#include <soc/dport_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/periph_defs.h>
#include <soc/uart_struct.h>
#include <soc/uart_reg.h>
#include <soc/timer_periph.h>
#include <soc/soc_caps.h>

#if CONFIG_IDF_TARGET_ESP32
#define ADC_WIDTH ADC_WIDTH_BIT_12
#elif SOC_ADC_RTC_MAX_BITWIDTH == 12 || SOC_ADC_MAX_BITWIDTH == 12
#define ADC_WIDTH ADC_WIDTH_BIT_12
#elif SOC_ADC_RTC_MAX_BITWIDTH == 13 || SOC_ADC_MAX_BITWIDTH == 13
#define ADC_WIDTH ADC_WIDTH_BIT_13
#endif

// Validate configured pins are defined and if not disable the feature.
#ifndef CONFIG_DCC_TRACK_BRAKE_PIN
  #define CONFIG_DCC_TRACK_BRAKE_PIN -1
#endif

#ifndef CONFIG_RAILCOM_TRIGGER_PIN
  #define CONFIG_RAILCOM_TRIGGER_PIN -1
#endif

#ifndef CONFIG_RAILCOM_DATA_PIN
  #define CONFIG_RAILCOM_DATA_PIN -1
#endif

#ifndef CONFIG_RAILCOM_DIRECTION_PIN
  #define CONFIG_RAILCOM_DIRECTION_PIN -1
#endif

#ifndef CONFIG_RAILCOM_SHORT_PIN
  #define CONFIG_RAILCOM_SHORT_PIN -1
#endif

#if CONFIG_RAILCOM_TRIGGER_PIN == -1
  #undef CONFIG_RAILCOM_CUT_OUT_ENABLED
#endif

#if CONFIG_RAILCOM_DATA_PIN == -1
  #undef CONFIG_RAILCOM_DATA_ENABLED
#endif

#ifndef CONFIG_OLED_RESET_PIN
  #define CONFIG_OLED_RESET_PIN -1
#endif

#ifndef TEMPSENSOR_ADC_CHANNEL
  #define TEMPSENSOR_ADC_CHANNEL -1
#endif

#ifndef CONFIG_DCC_OLCB_ENABLE_PIN
  #define CONFIG_DCC_OLCB_ENABLE_PIN -1
#endif

#ifndef CONFIG_I2C_ADC_ALERT_PIN
  #define CONFIG_I2C_ADC_ALERT_PIN -1
#endif

#ifndef CONFIG_RAILCOM_FEEDBACK_QUEUE
  #define CONFIG_RAILCOM_FEEDBACK_QUEUE 10
#endif

#if CONFIG_OPS_HBRIDGE_L298
  #define CONFIG_OPS_HBRIDGE_TYPE_NAME "L298"
  #define CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS 2000
  #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 2000
#elif CONFIG_OPS_HBRIDGE_LMD18200
  #define CONFIG_OPS_HBRIDGE_TYPE_NAME "LMD18200"
  #define CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS 3000
  #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 3000
#elif CONFIG_OPS_HBRIDGE_DRV880X
  #define CONFIG_OPS_HBRIDGE_TYPE_NAME "DRV880x"
  #define CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS 2800
  #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 2800
#elif CONFIG_OPS_HBRIDGE_DRV8873 || CONFIG_OPS_HBRIDGE_DRV8873_5A
  #define CONFIG_OPS_HBRIDGE_TYPE_NAME "DRV8873"
  #define CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS 10000

  #if CONFIG_OPS_HBRIDGE_DRV8873
    #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 10000
  #else
    #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 5000
  #endif
#elif CONFIG_OPS_HBRIDGE_POLOLU
  #define CONFIG_OPS_HBRIDGE_TYPE_NAME "MC33926"
  #define CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS 2500
  #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 2500
#elif CONFIG_OPS_HBRIDGE_BTS7960B_5A || CONFIG_OPS_HBRIDGE_BTS7960B_10A
#define CONFIG_OPS_HBRIDGE_TYPE_NAME "BTS7960B"
#define CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS 43000
  #if CONFIG_OPS_HBRIDGE_BTS7960B_10A
    #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 10000
  #else
    #define CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS 5000
  #endif
#elif CONFIG_OPS_TRACK_ENABLED
#error Unknown OPS h-bridge
#endif

#if CONFIG_PROG_HBRIDGE_L298
#define CONFIG_PROG_HBRIDGE_TYPE_NAME "L298"
#define CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS 2000
#elif CONFIG_PROG_HBRIDGE_LMD18200
#define CONFIG_PROG_HBRIDGE_TYPE_NAME "LMD18200"
#define CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS 3000
#elif CONFIG_PROG_HBRIDGE_DRV880X
#define CONFIG_PROG_HBRIDGE_TYPE_NAME "DRV880x"
#define CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS 2800
#elif CONFIG_PROG_HBRIDGE_DRV8873 || CONFIG_PROG_HBRIDGE_DRV8873_5A
#define CONFIG_PROG_HBRIDGE_TYPE_NAME "DRV8873"
#define CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS 10000
#elif CONFIG_PROG_HBRIDGE_POLOLU
#define CONFIG_PROG_HBRIDGE_TYPE_NAME "MC33926"
#define CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS 2500
#elif CONFIG_PROG_HBRIDGE_BTS7960B_5A || CONFIG_PROG_HBRIDGE_BTS7960B_10A
#define CONFIG_PROG_HBRIDGE_TYPE_NAME "BTS7960B"
#define CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS 43000
#elif CONFIG_PROG_TRACK_ENABLED
#error Unknown PROG h-bridge
#endif

#ifndef CONFIG_DISPLAY_LINE_COUNT
#if CONFIG_DISPLAY_OLED_128x64
#define CONFIG_DISPLAY_LINE_COUNT 8
#elif CONFIG_DISPLAY_OLED_128x32 || CONFIG_DISPLAY_LCD_20x4 || \
      CONFIG_DISPLAY_LCD_16x4
#define CONFIG_DISPLAY_LINE_COUNT 4
#else
#define CONFIG_DISPLAY_LINE_COUNT 2
#endif
#endif // CONFIG_DISPLAY_LINE_COUNT

#ifndef CONFIG_DISPLAY_COLUMN_COUNT
#if CONFIG_DISPLAY_LCD_20x4
#define CONFIG_DISPLAY_COLUMN_COUNT 20
#elif CONFIG_DISPLAY_OLED_96x16
#define CONFIG_DISPLAY_COLUMN_COUNT 12
#else
#define CONFIG_DISPLAY_COLUMN_COUNT 16
#endif
#endif // CONFIG_DISPLAY_COLUMN_COUNT

#ifndef CONFIG_DISPLAY_OLED_WIDTH
#if CONFIG_DISPLAY_OLED_96x16
#define CONFIG_DISPLAY_OLED_WIDTH 96
#else
#define CONFIG_DISPLAY_OLED_WIDTH 128
#endif
#endif // CONFIG_DISPLAY_OLED_WIDTH

#ifndef CONFIG_DISPLAY_OLED_HEIGHT
#if CONFIG_DISPLAY_OLED_128x64
#define CONFIG_DISPLAY_OLED_HEIGHT 64
#elif CONFIG_DISPLAY_OLED_128x32
#define CONFIG_DISPLAY_OLED_HEIGHT 32
#else
#define CONFIG_DISPLAY_OLED_HEIGHT 16
#endif
#endif // CONFIG_DISPLAY_OLED_HEIGHT

// Sanity check that the preamble bits are within the supported range.
#ifndef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS < 11
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS < 16 && CONFIG_RAILCOM_CUT_OUT_ENABLED
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 16
#elif CONFIG_OPS_DCC_PREAMBLE_BITS > 20
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 20
#endif

// Sanity check that the preamble bits are within range.
#ifndef CONFIG_PROG_DCC_PREAMBLE_BITS
#define CONFIG_PROG_DCC_PREAMBLE_BITS 22
#elif CONFIG_PROG_DCC_PREAMBLE_BITS < 22 || CONFIG_PROG_DCC_PREAMBLE_BITS > 50
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
#if CONFIG_DCC_TRACK_BRAKE_PIN != -1
/// H-Bridge BRAKE pin, inverse of the ENABLE pin.
GPIO_PIN(DCC_BRAKE, GpioOutputSafeHigh, CONFIG_DCC_TRACK_BRAKE_PIN);
#else
/// H-Bridge BRAKE pin, inverse of the ENABLE pin.
typedef DummyPin DCC_BRAKE_Pin;
#endif // CONFIG_DCC_TRACK_BRAKE_PIN != -1

#if CONFIG_OPS_TRACK_ENABLED
/// Enables the OPS track output h-bridge.
GPIO_PIN(OPS_ENABLE, GpioOutputSafeLow, CONFIG_OPS_TRACK_ENABLE_PIN);

#if CONFIG_OPSTRACK_ADC_I2C_CHANNEL_1 || \
    CONFIG_OPSTRACK_ADC_I2C_CHANNEL_2 || \
    CONFIG_OPSTRACK_ADC_I2C_CHANNEL_3
#define USE_I2C_FOR_OPS_CURRENT_SENSE 1

/// Fake OPS Current sense pin since it will be reported via I2C.
typedef DummyPin OPS_CURRENT_SENSE_Pin;
#else

#if CONFIG_OPSTRACK_ADC_CHANNEL_0
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_0, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_1
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_1, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_2
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_2, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_3
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_3, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_4
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_4, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_5
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_5, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_6
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_6, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_7
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_7, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_8
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_8, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC_CHANNEL_9
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC1_CHANNEL_9, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_0
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_0, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_1
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_1, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_2
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_2, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_3
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_3, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_4
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_4, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_5
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_5, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_6
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_6, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_7
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_7, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_8
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_8, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_OPSTRACK_ADC2_CHANNEL_9
/// OPS track current sense input pin.
ADC_PIN(OPS_CURRENT_SENSE, ADC2_CHANNEL_9, ADC_ATTEN_DB_11, ADC_WIDTH);
#endif
#endif // OPS ADC_I2C

#else // OPS DISABLED
/// Fake OPS enable pin since the track is disabled.
typedef DummyPin OPS_ENABLE_Pin;

/// Fake OPS Current sense pin since the track is disabled.
typedef DummyPin OPS_CURRENT_SENSE_Pin;
#endif // CONFIG_OPS_TRACK_ENABLED

#if CONFIG_PROG_TRACK_ENABLED
/// Enables the PROG track output h-bridge.
GPIO_PIN(PROG_ENABLE, GpioOutputSafeLow, CONFIG_PROG_TRACK_ENABLE_PIN);

#if CONFIG_PROGTRACK_ADC_I2C_CHANNEL_1 || \
    CONFIG_PROGTRACK_ADC_I2C_CHANNEL_2 || \
    CONFIG_PROGTRACK_ADC_I2C_CHANNEL_3
#define USE_I2C_FOR_PROG_CURRENT_SENSE 1

/// Fake PROG Current sense pin since it will be reported via I2C.
typedef DummyPin PROG_CURRENT_SENSE_Pin;
#else

#if CONFIG_PROGTRACK_ADC_CHANNEL_0
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_0, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_1
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_1, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_2
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_2, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_3
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_3, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_4
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_4, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_5
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_5, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_6
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_6, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_7
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_7, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_8
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_8, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_9
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_9, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_8
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_8, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC_CHANNEL_9
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC1_CHANNEL_9, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_0
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_0, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_1
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_1, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_2
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_2, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_3
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_3, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_4
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_4, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_5
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_5, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_6
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_6, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_7
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_7, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_8
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_8, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_PROGTRACK_ADC2_CHANNEL_9
/// PROG track current sense input pin.
ADC_PIN(PROG_CURRENT_SENSE, ADC2_CHANNEL_9, ADC_ATTEN_DB_11, ADC_WIDTH);
#endif

#endif // PROG ADC I2C

#else // PROG DISABLED
/// Fake PROG enable pin since the track is disabled.
typedef DummyPin PROG_ENABLE_Pin;

/// Fake PROG Current sense pin since the track is disabled.
typedef DummyPin PROG_CURRENT_SENSE_Pin;
#endif // CONFIG_PROG_TRACK_ENABLED

#if CONFIG_DCC_OLCB_ENABLE_PIN == -1
/// Enables the OpenLCB DCC signal output.
typedef DummyPin OLCB_DCC_ENABLE_Pin;
#else
/// Enables the OpenLCB DCC signal output.
GPIO_PIN(OLCB_DCC_ENABLE, GpioOutputSafeLow, CONFIG_DCC_OLCB_ENABLE_PIN);
#endif //  CONFIG_DCC_OLCB_ENABLE_PIN

/// OpenLCB RailCom detector enable pin.
/// For the ESP32CSPCB this is shared with OPS as a single unified detector
/// however the API requires a pin to be declared regardless.
typedef DummyPin OLCB_RAILCOM_TRIGGER_Pin;

#if CONFIG_RAILCOM_TRIGGER_PIN == -1
/// RailCom detector enable pin.
typedef DummyPin RAILCOM_TRIGGER_Pin;
#else
/// RailCom detector enable pin.
GPIO_PIN(RAILCOM_TRIGGER, GpioOutputSafeLow, CONFIG_RAILCOM_TRIGGER_PIN);
#endif // CONFIG_RAILCOM_TRIGGER_PIN == -1

#if CONFIG_RAILCOM_SHORT_PIN == -1
/// RailCom detector current drain pin.
typedef DummyPin RAILCOM_SHORT_Pin;
#else
/// RailCom detector current drain pin.
GPIO_PIN(RAILCOM_SHORT, GpioOutputSafeHigh, CONFIG_RAILCOM_SHORT_PIN);
#endif // CONFIG_RAILCOM_SHORT_PIN == -1

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

#if CONFIG_I2C_ADC_ALERT_PIN == -1
/// RailCom detector current drain pin.
typedef DummyPin I2C_ADC_ALERT_Pin;
#else
/// External ADC alert pin.
GPIO_PIN(I2C_ADC_ALERT, GpioInputPU, CONFIG_I2C_ADC_ALERT_PIN);
#endif

#if CONFIG_TEMPSENSOR_ADC_CHANNEL_0
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_0, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_1
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_1, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_2
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_2, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_3
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_3, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_4
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_4, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_5
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_5, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_6
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_6, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_7
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_7, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_8
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_8, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC_CHANNEL_9
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC1_CHANNEL_9, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_0
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_0, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_1
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_1, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_2
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_2, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_3
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_3, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_4
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_4, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_5
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_5, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_6
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_6, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_7
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_7, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_8
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_8, ADC_ATTEN_DB_11, ADC_WIDTH);
#elif CONFIG_TEMPSENSOR_ADC2_CHANNEL_9
/// External thermal sensor input pin.
ADC_PIN(THERMAL_SENSOR, ADC2_CHANNEL_9, ADC_ATTEN_DB_11, ADC_WIDTH);
#else
/// Fake thermal sensor pin.
typedef DummyPin THERMAL_SENSOR_Pin;
#endif

/// GPIO Pin initializer.
typedef GpioInitializer<OLED_RESET_Pin, RAILCOM_TRIGGER_Pin, RAILCOM_SHORT_Pin,
                        DCC_SIGNAL_Pin, DCC_BRAKE_Pin, OPS_ENABLE_Pin,
                        PROG_ENABLE_Pin, FACTORY_RESET_BUTTON_Pin,
                        OPS_CURRENT_SENSE_Pin, PROG_CURRENT_SENSE_Pin,
                        THERMAL_SENSOR_Pin, BOOTLOADER_BUTTON_Pin,
                        OLCB_DCC_ENABLE_Pin, OLCB_RAILCOM_TRIGGER_Pin,
                        I2C_ADC_ALERT_Pin> GpioInit;

/// RailCom hardware definition
struct RailComHwDefs
{
#if CONFIG_RAILCOM_DATA_ENABLED
#if CONFIG_RAILCOM_UART1
  static constexpr uart_port_t UART = 1;
  static constexpr uart_dev_t *UART_BASE = &UART1;
  static constexpr periph_module_t UART_PERIPH = PERIPH_UART1_MODULE;
  static constexpr int UART_ISR_SOURCE = ETS_UART1_INTR_SOURCE;
  static constexpr uint32_t UART_MATRIX_IDX = U1RXD_IN_IDX;
#if CONFIG_IDF_TARGET_ESP32
  static constexpr uint32_t UART_CLOCK_EN_BIT = DPORT_UART1_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = DPORT_UART1_RST;
#elif CONFIG_IDF_TARGET_ESP32S3
  static constexpr uint32_t UART_CLOCK_EN_BIT = SYSTEM_UART1_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = SYSTEM_UART1_RST;
#endif
#elif CONFIG_RAILCOM_UART2
  static constexpr uart_port_t UART = 2;
  static constexpr uart_dev_t *UART_BASE = &UART2;
  static constexpr periph_module_t UART_PERIPH = PERIPH_UART2_MODULE;
  static constexpr int UART_ISR_SOURCE = ETS_UART2_INTR_SOURCE;
  static constexpr uint32_t UART_MATRIX_IDX = U2RXD_IN_IDX;
#if CONFIG_IDF_TARGET_ESP32
  static constexpr uint32_t UART_CLOCK_EN_BIT = DPORT_UART2_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = DPORT_UART2_RST;
#elif CONFIG_IDF_TARGET_ESP32S3
  static constexpr uint32_t UART_CLOCK_EN_BIT = SYSTEM_UART2_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = SYSTEM_UART2_RST;
#endif
#else
  #error Unsupported UART selected for OPS RailCom!
#endif
#endif // CONFIG_RAILCOM_DATA_ENABLED

  /// RailCom detector enable pin
  using RAILCOM_TRIGGER_Pin = ::RAILCOM_TRIGGER_Pin;

#if CONFIG_RAILCOM_DATA_ENABLED
  /// RailCom data pin
  static constexpr gpio_num_t RAILCOM_DATA_PIN =
    (gpio_num_t)CONFIG_RAILCOM_DATA_PIN;
  /// RailCom direction pin
  static constexpr gpio_num_t RAILCOM_DIRECTION_PIN =
    (gpio_num_t)CONFIG_RAILCOM_DIRECTION_PIN;
#endif // CONFIG_RAILCOM_DATA_ENABLED

  /// Number of RailCom packets to queue
  static constexpr size_t PACKET_Q_SIZE = CONFIG_RAILCOM_FEEDBACK_QUEUE;

  static void hw_init()
  {
#if CONFIG_RAILCOM_DATA_ENABLED
    // initialize the UART
    periph_module_enable(UART_PERIPH);
    esp_rom_gpio_pad_select_gpio(RAILCOM_DATA_PIN);
    esp_rom_gpio_connect_in_signal(RAILCOM_DATA_PIN, UART_MATRIX_IDX, false);
#endif // CONFIG_RAILCOM_DATA_ENABLED
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

#if CONFIG_DCC_TRACK_BRAKE_PIN != -1
  /// Wrapper which handles the toggling of brake pin in conjuction with the
  /// enable pins.
  struct BOOSTER_ENABLE_Pin
  {
      static void set(bool value)
      {
        if (value)
        {
          // disable the brake pin first
          DCC_BRAKE_Pin::set(false);
          // delay 1usec to allow h-bridge to catch up
          ets_delay_us(1);
          // enable the h-bridge output pin
          OPS_ENABLE_Pin::set(true);
        }
        else
        {
          // disable the h-bridge output pin
          OPS_ENABLE_Pin::set(false);
          // delay 1usec to allow h-bridge to catch up
          ets_delay_us(1);
          // enable the h-bridge brake pin
          DCC_BRAKE_Pin::set(true);
        }
      }

      static void hw_init()
      {
        DCC_BRAKE_Pin::hw_init();
        OPS_ENABLE_Pin::hw_init();
        set(false);
      }
  };
#else
  using BOOSTER_ENABLE_Pin = ::OPS_ENABLE_Pin;
#endif // CONFIG_OPS_TRACK_BRAKE_PIN != -1

  /// The number of preamble bits to send exclusive of end of packet '1' bit
  /// for DCC packets that are not service mode.
  static constexpr uint32_t DCC_PREAMBLE_BITS = CONFIG_OPS_DCC_PREAMBLE_BITS;

  /// The number of preamble bits to send exclusive of end of packet '1' bit
  /// for service mode DCC packets.
  static constexpr uint32_t DCC_SERVICE_MODE_PREAMBLE_BITS =
    CONFIG_PROG_DCC_PREAMBLE_BITS;

  /// Number of RMT ticks for each half of the RMT encoded ZERO bit.
  static constexpr uint8_t DCC_ZERO_RMT_TICKS =
    CONFIG_DCC_RMT_TICKS_ZERO_PULSE;

  /// Number of RMT ticks for each half of the RMT encoded ONE bit.
  static constexpr uint8_t DCC_ONE_RMT_TICKS = CONFIG_DCC_RMT_TICKS_ONE_PULSE;

  /// RMT Channel to use for the DCC Signal output.
  static const rmt_channel_t RMT_CHANNEL = RMT_CHANNEL_0;

#if CONFIG_DCC_RMT_HIGH_FIRST
  /// Visual name of the DCC Wave pattern.
  static constexpr const char * const RMT_WAVE_FMT = "high,low";

  /// Voltage to output on the signal pin for the first half of the DCC signal
  /// wave pattern.
  static constexpr uint8_t RMT_DCC_FIRST_HALF = 1;

  /// Voltage to output on the signal pin for the second half of the DCC
  /// signal wave pattern.
  static constexpr uint8_t RMT_DCC_SECOND_HALF = 0;
#else
  /// Visual name of the DCC Wave pattern.
  static constexpr const char * const RMT_WAVE_FMT = "low,high";

  /// Voltage to output on the signal pin for the first half of the DCC signal
  /// wave pattern.
  static constexpr uint8_t RMT_DCC_FIRST_HALF = 0;

  /// Voltage to output on the signal pin for the second half of the DCC
  /// signal wave pattern.
  static constexpr uint8_t RMT_DCC_SECOND_HALF = 1;
#endif // CONFIG_DCC_RMT_HIGH_FIRST

  /// RMT Clock configuration.
  static constexpr rmt_source_clk_t RMT_CLOCK_SOURCE =
    (rmt_source_clk_t)CONFIG_DCC_RMT_CLOCK_SOURCE;

  /// Number of outgoing DCC packets to allow in the queue.
  static const size_t PACKET_Q_SIZE = CONFIG_PACKET_QUEUE_SIZE;

  /// Track Booster output.
  using InternalBoosterOutput =
    DccOutputHwReal<DccOutput::TRACK, BOOSTER_ENABLE_Pin, RAILCOM_TRIGGER_Pin,
                    RailComHwDefs::RAILCOM_START_PHASE1_DELAY_USEC,
                    RailComHwDefs::RAILCOM_START_PHASE2_DELAY_USEC,
                    RailComHwDefs::RAILCOM_STOP_DELAY_USEC>;
  /// Fake output hardware for program track.
  using ProgBoosterOutput = DccOutputHwDummy<DccOutput::PGM>;
  /// OpenLCB Booster output.
  using OpenLCBBoosterOutput =
    DccOutputHwReal<DccOutput::LCC, OLCB_DCC_ENABLE_Pin,
                    OLCB_RAILCOM_TRIGGER_Pin, 0, 0, 0>;
}; // DccHwDefs

#endif // HARDWARE_HXX_
