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

#include "sdkconfig.h"

#include <bootloader_hal.h>
#include "Esp32BootloaderHal.hxx"

extern "C"
{

/// Initializes the node specific bootloader hardware (LEDs)
void bootloader_hw_set_to_safe(void)
{
  LOG(VERBOSE, "[Bootloader] bootloader_hw_set_to_safe");
}

/// Verifies that the bootloader has been requested.
///
/// @return true (always).
///
/// NOTE: On the ESP32 this defaults to always return true since this code will
/// not be invoked through normal node startup.
bool request_bootloader(void)
{
  LOG(VERBOSE, "[Bootloader] request_bootloader");
  // Default to allow bootloader to run since we are not entering the
  // bootloader loop unless requested by app_main.
  return true;
}

/// Updates the state of a status LED.
///
/// @param led is the LED to update.
/// @param value is the new state of the LED.
///
/// NOTE: Currently the following mapping is being used for the LEDs:
/// LED_ACTIVE -> Activity LED
/// LED_WRITING -> WiFi LED
/// LED_REQUEST -> Used only as a hook for printing bootloader startup.
void bootloader_led(enum BootloaderLed led, bool value)
{
  LOG(VERBOSE, "[Bootloader] bootloader_led(%d, %d)", led, value);
  if (led == LED_REQUEST)
  {
    LOG(INFO, "[Bootloader] Preparing to receive firmware");
    LOG(INFO, "[Bootloader] Current partition: %s", current->label);
    LOG(INFO, "[Bootloader] Target partition: %s", target->label);
  }
}

} // extern "C"

/// Starts the ESP32 Bootloader "lean" stack.
///
/// @param id is the node identifier to use.
void start_bootloader_stack(uint64_t id)
{
  esp32_bootloader_run(id, (gpio_num_t)CONFIG_LCC_CAN_RX_PIN
                     , (gpio_num_t)CONFIG_LCC_CAN_TX_PIN, true);
}