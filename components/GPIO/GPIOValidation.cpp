/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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

#if defined(CONFIG_GPIO_OUTPUTS) || defined(CONFIG_GPIO_SENSORS)
bool is_restricted_pin(int8_t pin)
{
  vector<uint8_t> restrictedPins
  {
#if !defined(CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS)
    0,                        // Bootstrap / Firmware Flash Download
    1,                        // UART0 TX
    2,                        // Bootstrap / Firmware Flash Download
    3,                        // UART0 RX
    5,                        // Bootstrap
    6, 7, 8, 9, 10, 11,       // on-chip flash pins
    12, 15,                   // Bootstrap / SD pins
#endif // ! CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
    CONFIG_OPS_ENABLE_PIN
  , CONFIG_OPS_SIGNAL_PIN
  , CONFIG_PROG_ENABLE_PIN
  , CONFIG_PROG_SIGNAL_PIN

#if defined(CONFIG_OPS_RAILCOM)
#if defined(CONFIG_OPS_HBRIDGE_LMD18200)
  , CONFIG_OPS_RAILCOM_BRAKE_PIN
#endif
  , CONFIG_OPS_RAILCOM_ENABLE_PIN
  , CONFIG_OPS_RAILCOM_UART_RX_PIN
#endif // CONFIG_OPS_RAILCOM

#if defined(CONFIG_LCC_CAN_ENABLED)
  , CONFIG_LCC_CAN_RX_PIN
  , CONFIG_LCC_CAN_TX_PIN
#endif

#if defined(CONFIG_HC12)
  , CONFIG_HC12_RX_PIN
  , CONFIG_HC12_TX_PIN
#endif

#if defined(CONFIG_NEXTION)
  , CONFIG_NEXTION_RX_PIN
  , CONFIG_NEXTION_TX_PIN
#endif

#if defined(CONFIG_DISPLAY_TYPE_OLED) || defined(CONFIG_DISPLAY_TYPE_LCD)
  , CONFIG_DISPLAY_SCL
  , CONFIG_DISPLAY_SDA
#if defined(CONFIG_DISPLAY_OLED_RESET_PIN) && CONFIG_DISPLAY_OLED_RESET_PIN != -1
  , CONFIG_DISPLAY_OLED_RESET_PIN
#endif
#endif

#if defined(CONFIG_LOCONET)
  , CONFIG_LOCONET_RX_PIN
  , CONFIG_LOCONET_TX_PIN
#endif

#if defined(CONFIG_GPIO_S88)
  , CONFIG_GPIO_S88_CLOCK_PIN
  , CONFIG_GPIO_S88_LOAD_PIN
#if defined(CONFIG_GPIO_S88_RESET_PIN) && CONFIG_GPIO_S88_RESET_PIN != -1
  , CONFIG_GPIO_S88_RESET_PIN
#endif
#endif

#if defined(CONFIG_STATUS_LED)
  , CONFIG_STATUS_LED_DATA_PIN
#endif
  };

  return std::find(restrictedPins.begin()
                 , restrictedPins.end()
                 , pin) != restrictedPins.end();
}
#endif // CONFIG_GPIO_OUTPUTS || CONFIG_GPIO_SENSORS