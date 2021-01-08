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

#include "StatusLED.h"
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>

StateFlowBase::Action StatusLED::init()
{
#if !CONFIG_STATUS_LED
  return exit();
#else
  LOG(INFO
    , "[Status] Initializing LEDs (color-mode:%s, protocol:%s, pin: %d, "
      "brightness: %d)"
    , NEO_COLOR_MODE_NAME, NEO_METHOD_NAME, CONFIG_STATUS_LED_DATA_PIN
    , brightness_);
  bus_.reset(
    new NeoPixelBrightnessBus<NEO_COLOR_MODE, NEO_METHOD>(LED::MAX_LED
                                                        , CONFIG_STATUS_LED_DATA_PIN));
  bus_->Begin();
  bus_->SetBrightness(brightness_);
  bus_->ClearTo(RGB_OFF_);
  bus_->Show();
  Singleton<Esp32WiFiManager>::instance()->register_network_up_callback(
  [&](esp_interface_t interface, uint32_t ip)
  {
    if (interface == ESP_IF_WIFI_AP)
    {
      setStatusLED(StatusLED::LED::WIFI, StatusLED::COLOR::BLUE);
    }
    else if (interface == ESP_IF_WIFI_STA)
    {
      setStatusLED(StatusLED::LED::WIFI, StatusLED::COLOR::GREEN);
    }
  });
  Singleton<Esp32WiFiManager>::instance()->register_network_down_callback(
  [&](esp_interface_t interface)
  {
    setStatusLED(StatusLED::LED::WIFI, StatusLED::COLOR::RED);
  });
  Singleton<Esp32WiFiManager>::instance()->register_network_init_callback(
  [&](esp_interface_t interface)
  {
    setStatusLED(StatusLED::LED::WIFI, StatusLED::COLOR::GREEN_BLINK);
  });
  return sleep_and_call(&timer_, updateInterval_, STATE(update));
#endif
}

#define SET_LED_COLOR_BLINK(led, val, color) \
  else if(colors_[led] == val) \
  { \
    bus_->SetPixelColor(led, color); \
  } \

#define SET_LED_STATE(led, val, color) \
  else if (colors_[led] == val && bus_->GetPixelColor(led) != color) \
  { \
    bus_->SetPixelColor(led, color); \
  } \

StateFlowBase::Action StatusLED::update()
{
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
        bus_->SetPixelColor(led, RGB_OFF_);
      }
      SET_LED_COLOR_BLINK(led, RED_BLINK, RGB_RED_)
      SET_LED_COLOR_BLINK(led, GREEN_BLINK, RGB_GREEN_)
      SET_LED_COLOR_BLINK(led, BLUE_BLINK, RGB_BLUE_)
      SET_LED_COLOR_BLINK(led, YELLOW_BLINK, RGB_YELLOW_)
      state_[led] = !state_[led];
    }
    SET_LED_STATE(led, RED, RGB_RED_)
    SET_LED_STATE(led, GREEN, RGB_GREEN_)
    SET_LED_STATE(led, BLUE, RGB_BLUE_)
    SET_LED_STATE(led, YELLOW, RGB_YELLOW_)
    SET_LED_STATE(led, OFF, RGB_OFF_)
  }
  return yield_and_call(STATE(update_bus));
}

StateFlowBase::Action StatusLED::update_bus()
{
  if (bus_->GetBrightness() != brightness_)
  {
    bus_->SetBrightness(brightness_);
  }
  if (bus_->CanShow())
  {
    bus_->Show();
    return sleep_and_call(&timer_, updateInterval_, STATE(update));
  }
  return yield_and_call(STATE(update_bus));
}

void StatusLED::setStatusLED(const LED led, const COLOR color, const bool on)
{
  colors_[led] = color;
  state_[led] = on;
}

void StatusLED::setBrightness(uint8_t level)
{
  brightness_ = level;
}