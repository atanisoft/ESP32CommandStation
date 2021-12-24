/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2021 Mike Dunston

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

#ifndef STATUS_LED_H_
#define STATUS_LED_H_

#include <esp_event.h>
#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <NeoPixelBrightnessBus.h>
#include <utils/constants.hxx>
#include <utils/Singleton.hxx>

#include "sdkconfig.h"

#ifndef CONFIG_STATUS_LED_BRIGHTNESS
#define CONFIG_STATUS_LED_BRIGHTNESS 128
#endif

namespace esp32cs
{

class StatusLED : public Singleton<StatusLED>
{
public:
  enum COLOR : uint8_t
  {
    OFF
  , RED
  , GREEN
  , YELLOW
  , BLUE
  , RED_BLINK
  , GREEN_BLINK
  , BLUE_BLINK
  , YELLOW_BLINK
  };

  enum LED : uint8_t
  {
    WIFI_STA,
    WIFI_AP,
    BOOTLOADER,
    OPS_TRACK,
    PROG_TRACK,
    MAX_LED
  };

  StatusLED();
  void hw_init();
  void set(const LED led, const COLOR color, const bool on = false);
  void clear();
  void setBrightness(uint8_t level)
  {
    brightness_ = level;
  }
  uint8_t getBrightness()
  {
    return brightness_;
  }
  void start_task();

  /// Refreshes the LEDs.
  ///
  /// NOTE: This is not intended to be called from outside of the StatuLED
  /// module.
  void refresh();

private:
  uint8_t brightness_{CONFIG_STATUS_LED_BRIGHTNESS};
  COLOR colors_[LED::MAX_LED];
  bool state_[LED::MAX_LED];
};

} // namespace esp32cs
#endif // STATUS_LED_H_