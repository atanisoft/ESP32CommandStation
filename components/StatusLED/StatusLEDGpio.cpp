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
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <utils/GpioInitializer.hxx>

namespace esp32cs
{

#if CONFIG_STATUS_LED_WIFI_STA_PIN == -1
typedef DummyPinWithRead WIFI_STA_Pin;
#else
GPIO_PIN(WIFI_STA, GpioOutputSafeHighInvert, CONFIG_STATUS_LED_WIFI_STA_PIN);
#endif

#if CONFIG_STATUS_LED_WIFI_AP_PIN == -1
typedef DummyPinWithRead WIFI_AP_Pin;
#else
GPIO_PIN(WIFI_AP, GpioOutputSafeHighInvert, CONFIG_STATUS_LED_WIFI_AP_PIN);
#endif

#if CONFIG_STATUS_LED_BOOTLOADER_ACTIVE_PIN == -1
typedef DummyPinWithRead BOOTLOADER_Pin;
#else
GPIO_PIN(BOOTLOADER, GpioOutputSafeHighInvert,
         CONFIG_STATUS_LED_BOOTLOADER_ACTIVE_PIN);
#endif

#if CONFIG_STATUS_LED_OPS_ACTIVE_PIN == -1
typedef DummyPinWithRead OPS_ACTIVE_Pin;
#else
GPIO_PIN(OPS_ACTIVE, GpioOutputSafeHighInvert,
         CONFIG_STATUS_LED_OPS_ACTIVE_PIN);
#endif

#if CONFIG_STATUS_LED_PROG_ACTIVE_PIN == -1
typedef DummyPinWithRead PROG_ACTIVE_Pin;
#else
GPIO_PIN(PROG_ACTIVE, GpioOutputSafeHighInvert,
         CONFIG_STATUS_LED_PROG_ACTIVE_PIN);
#endif

typedef GpioInitializer<WIFI_STA_Pin, WIFI_AP_Pin,
                        BOOTLOADER_Pin, OPS_ACTIVE_Pin,
                        PROG_ACTIVE_Pin> GpioInit;

const Gpio *LEDS[StatusLED::LED::MAX_LED] = 
{
  WIFI_STA_Pin::instance(),
  WIFI_AP_Pin::instance(),
  BOOTLOADER_Pin::instance(),
  OPS_ACTIVE_Pin::instance(),
  PROG_ACTIVE_Pin::instance()
};

void StatusLED::hw_init()
{
  GpioInit::hw_init();
  for(int led = 0; led < LED::MAX_LED; led++)
  {
    LEDS[led]->set();
    vTaskDelay(pdMS_TO_TICKS(100));
    LEDS[led]->clr();
  }
  start_task();
}

void StatusLED::refresh()
{
  for(int led = 0; led < LED::MAX_LED; led++)
  {
    if(colors_[led] == RED_BLINK ||
       colors_[led] == GREEN_BLINK ||
       colors_[led] == BLUE_BLINK ||
       colors_[led] == YELLOW_BLINK)
    {
      state_[led] = !state_[led];
      LEDS[led]->write(state_[led]);
    }
    else if (colors_[led] == OFF)
    {
      LEDS[led]->clr();
    }
    else
    {
      LEDS[led]->set();
    }
  }
}

} // namespace esp32cs