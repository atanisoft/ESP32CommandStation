/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020-2021 Mike Dunston

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

#include "sdkconfig.h"
#include <algorithm>
#include <driver/gpio.h>
#include <stdint.h>
#include <utils/logging.h>
#include <vector>

#if CONFIG_GPIO_OUTPUTS || CONFIG_GPIO_SENSORS
bool is_restricted_pin(gpio_num_t pin)
{
  // early exit for pins that are outside the supported range
  if (!GPIO_IS_VALID_GPIO(pin))
  {
    LOG(WARNING
      , "[GPIO] Rejecting attempt to use pin %d as it is not valid", pin);
    return true;
  }
  // list of restricted pins and pins that are currently in use by various
  // hardware options. This will not take into account sensors, outputs, etc.
  std::vector<gpio_num_t> restrictedPins
  {
#ifndef CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
    GPIO_NUM_0,                           // Bootstrap / Firmware Download
    GPIO_NUM_1,                           // UART0 TX
    GPIO_NUM_2,                           // Bootstrap / Firmware Download
    GPIO_NUM_3,                           // UART0 RX
    GPIO_NUM_5,                           // Bootstrap
    GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_8,
    GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_11, // on-chip flash pins
    GPIO_NUM_12, GPIO_NUM_15,             // Bootstrap / SD pins
#endif // ! CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
    (gpio_num_t)CONFIG_OPS_HBRIDGE_ENABLE_PIN
  , (gpio_num_t)CONFIG_OPS_HBRIDGE_SIGNAL_PIN
#if CONFIG_OPS_HBRIDGE_BRAKE_PIN
  , (gpio_num_t)CONFIG_OPS_HBRIDGE_BRAKE_PIN
#endif
  , (gpio_num_t)CONFIG_PROG_HBRIDGE_ENABLE_PIN
  , (gpio_num_t)CONFIG_PROG_HBRIDGE_SIGNAL_PIN

#if CONFIG_OPS_RAILCOM
  , (gpio_num_t)CONFIG_OPS_RAILCOM_ENABLE_PIN
  , (gpio_num_t)CONFIG_OPS_RAILCOM_UART_RX_PIN
#endif // CONFIG_OPS_RAILCOM

#if CONFIG_LCC_CAN_RX_PIN != -1 && CONFIG_LCC_CAN_TX_PIN != -1
  , (gpio_num_t)CONFIG_LCC_CAN_RX_PIN
  , (gpio_num_t)CONFIG_LCC_CAN_TX_PIN
#endif

#if CONFIG_HC12
  , (gpio_num_t)CONFIG_HC12_RX_PIN
  , (gpio_num_t)CONFIG_HC12_TX_PIN
#endif

#if CONFIG_NEXTION
  , (gpio_num_t)CONFIG_NEXTION_RX_PIN
  , (gpio_num_t)CONFIG_NEXTION_TX_PIN
#endif

#if CONFIG_DISPLAY_TYPE_OLED || CONFIG_DISPLAY_TYPE_LCD
  , (gpio_num_t)CONFIG_DISPLAY_SCL
  , (gpio_num_t)CONFIG_DISPLAY_SDA
#if CONFIG_DISPLAY_OLED_RESET_PIN && \
    CONFIG_DISPLAY_OLED_RESET_PIN != GPIO_NUM_NC
  , (gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN
#endif
#endif

#if CONFIG_GPIO_S88
  , (gpio_num_t)CONFIG_GPIO_S88_CLOCK_PIN
  , (gpio_num_t)CONFIG_GPIO_S88_LOAD_PIN
#if CONFIG_GPIO_S88_RESET_PIN && \
    CONFIG_GPIO_S88_RESET_PIN != GPIO_NUM_NC
  , (gpio_num_t)CONFIG_GPIO_S88_RESET_PIN
#endif
#endif

#if CONFIG_STATUS_LED
  , (gpio_num_t)CONFIG_STATUS_LED_DATA_PIN
#endif
  };

  return std::find(restrictedPins.begin()
                 , restrictedPins.end()
                 , pin) != restrictedPins.end();
}
#endif // CONFIG_GPIO_OUTPUTS || CONFIG_GPIO_SENSORS