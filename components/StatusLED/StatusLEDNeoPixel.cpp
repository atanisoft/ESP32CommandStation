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

#include "StatusLED.hxx"
#include <freertos_includes.h>
#include <os/os.h>

namespace esp32cs
{

#ifndef CONFIG_STATUS_LED_DATA_PIN
#define CONFIG_STATUS_LED_DATA_PIN -1
#endif

#if CONFIG_STATUS_LED_DATA_PIN == -1 || defined(CONFIG_STATUS_LED_COLOR_RGB)
#define NEO_COLOR_TYPE RgbColor
#define NEO_COLOR_MODE NeoRgbFeature
#define NEO_COLOR_MODE_NAME "RGB"
#elif defined(CONFIG_STATUS_LED_COLOR_GRB)
#define NEO_COLOR_TYPE RgbColor
#define NEO_COLOR_MODE NeoGrbFeature
#define NEO_COLOR_MODE_NAME "GRB"
#elif defined(CONFIG_STATUS_LED_COLOR_RGBW)
#define NEO_COLOR_MODE NeoRgbwFeature
#define NEO_COLOR_TYPE RgbwColor
#define NEO_COLOR_MODE_NAME "RGBW"
#elif defined(CONFIG_STATUS_LED_COLOR_GRBW)
#define NEO_COLOR_MODE NeoGrbwFeature
#define NEO_COLOR_TYPE RgbwColor
#define NEO_COLOR_MODE_NAME "GRBW"
#elif defined(CONFIG_STATUS_LED_COLOR_BRG)
#define NEO_COLOR_TYPE RgbColor
#define NEO_COLOR_MODE NeoBrgFeature
#define NEO_COLOR_MODE_NAME "BRG"
#elif defined(CONFIG_STATUS_LED_COLOR_RBG)
#define NEO_COLOR_TYPE RgbColor
#define NEO_COLOR_MODE NeoRbgFeature
#define NEO_COLOR_MODE_NAME "RBG"
#endif

#if CONFIG_STATUS_LED_DATA_PIN == -1 || defined(CONFIG_STATUS_LED_TYPE_WS2811)
#if CONFIG_IDF_TARGET_ESP32S2
#define NEO_METHOD NeoEsp32Rmt1Ws2811Method
#define NEO_METHOD_NAME "RMT(1)-Ws2811"
#else
#define NEO_METHOD NeoEsp32Rmt6Ws2811Method
#define NEO_METHOD_NAME "RMT(6)-Ws2811"
#endif
#elif defined(CONFIG_STATUS_LED_TYPE_WS281X)
#define NEO_METHOD NeoEsp32Rmt6Ws2812xMethod
#define NEO_METHOD_NAME "RMT(6)-Ws2812"
#elif defined(CONFIG_STATUS_LED_TYPE_WS281X_800K)
#define NEO_METHOD NeoEsp32Rmt6800KbpsMethod
#define NEO_METHOD_NAME "RMT(6)-Ws2812-800kbps"
#elif defined(CONFIG_STATUS_LED_TYPE_WS281X_400K)
#define NEO_METHOD NeoEsp32Rmt6400KbpsMethod
#define NEO_METHOD_NAME "RMT(6)-Ws2812-400kbps"
#elif defined(CONFIG_STATUS_LED_TYPE_SK6812) || \
      defined(CONFIG_STATUS_LED_TYPE_LC6812)
#define NEO_METHOD NeoEsp32Rmt6Sk6812Method
#define NEO_METHOD_NAME "RMT(6)-sk6812"
#elif defined(CONFIG_STATUS_LED_TYPE_APA106)
#define NEO_METHOD NeoEsp32Rmt6Apa106Method
#define NEO_METHOD_NAME "RMT(6)-APA106"
#elif defined(CONFIG_STATUS_LED_TYPE_TX1812)
#define NEO_METHOD NeoEsp32Rmt6Tx1812Method
#define NEO_METHOD_NAME "RMT(6)-TX1812"
#endif

#ifndef CONFIG_STATUS_LED_UPDATE_INTERVAL_MSEC
#define CONFIG_STATUS_LED_UPDATE_INTERVAL_MSEC 450
#endif

NeoPixelBrightnessBus<NEO_COLOR_MODE, NEO_METHOD> pixelBus(
    StatusLED::LED::MAX_LED, CONFIG_STATUS_LED_DATA_PIN);

NEO_COLOR_TYPE RGB_RED = NEO_COLOR_TYPE(255, 0, 0);
NEO_COLOR_TYPE RGB_GREEN = NEO_COLOR_TYPE(0, 255, 0);
NEO_COLOR_TYPE RGB_YELLOW = NEO_COLOR_TYPE(255, 255, 0);
NEO_COLOR_TYPE RGB_BLUE = NEO_COLOR_TYPE(0, 0, 255);
NEO_COLOR_TYPE RGB_OFF = NEO_COLOR_TYPE(0);

#define SET_LED_COLOR_BLINK(led, val, color) \
  else if(colors_[led] == val) \
  { \
    pixelBus.SetPixelColor(led, color); \
  }

#define SET_LED_STATE(led, val, color) \
  else if (colors_[led] == val && pixelBus.GetPixelColor(led) != color) \
  { \
    pixelBus.SetPixelColor(led, color); \
  }

void StatusLED::hw_init()
{
#if CONFIG_STATUS_LED_DATA_PIN != -1
  static NEO_COLOR_TYPE LED_COLOR_CYCLE[5] =
  {
    RGB_RED,
    RGB_GREEN,
    RGB_BLUE,
    RGB_YELLOW,
    RGB_OFF
  };
  LOG(INFO,
      "[Status] Initializing LEDs (color-mode:%s, protocol:%s, pin: %d, "
      "brightness: %d)",
      NEO_COLOR_MODE_NAME, NEO_METHOD_NAME, CONFIG_STATUS_LED_DATA_PIN,
      brightness_);
  pixelBus.Begin();
  pixelBus.SetBrightness(brightness_);
  for (auto color : LED_COLOR_CYCLE)
  {
    pixelBus.ClearTo(color);
    pixelBus.Show();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  start_task();
#endif // CONFIG_STATUS_LED_DATA_PIN != -1
}

void StatusLED::refresh()
{
#if CONFIG_STATUS_LED_DATA_PIN != -1
  for(int led = 0; led < LED::MAX_LED; led++)
  {
    // if the LED is set to blink, toggle it
    if(colors_[led] == RED_BLINK ||
       colors_[led] == GREEN_BLINK ||
       colors_[led] == BLUE_BLINK ||
       colors_[led] == YELLOW_BLINK)
    {
      if(state_[led])
      {
        pixelBus.SetPixelColor(led, RGB_OFF);
      }
      SET_LED_COLOR_BLINK(led, RED_BLINK, RGB_RED)
      SET_LED_COLOR_BLINK(led, GREEN_BLINK, RGB_GREEN)
      SET_LED_COLOR_BLINK(led, BLUE_BLINK, RGB_BLUE)
      SET_LED_COLOR_BLINK(led, YELLOW_BLINK, RGB_YELLOW)
      state_[led] = !state_[led];
    }
    SET_LED_STATE(led, RED, RGB_RED)
    SET_LED_STATE(led, GREEN, RGB_GREEN)
    SET_LED_STATE(led, BLUE, RGB_BLUE)
    SET_LED_STATE(led, YELLOW, RGB_YELLOW)
    SET_LED_STATE(led, OFF, RGB_OFF)
  }
  if (pixelBus.GetBrightness() != brightness_)
  {
    pixelBus.SetBrightness(brightness_);
  }
  if (pixelBus.CanShow())
  {
    pixelBus.Show();
  }
#endif // CONFIG_STATUS_LED_DATA_PIN != -1
}

} // namespace esp32cs