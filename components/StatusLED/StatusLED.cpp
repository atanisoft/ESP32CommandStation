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

namespace esp32cs
{

StatusLED::StatusLED()
{
  if (brightness_ < 8)
  {
    brightness_ = 8;
  }
  clear();
}

#ifndef CONFIG_STATUS_LED_UPDATE_INTERVAL_MSEC
#define CONFIG_STATUS_LED_UPDATE_INTERVAL_MSEC 450
#endif

static constexpr uint32_t LED_UPDATE_TASK_STACK = 2048;
static constexpr BaseType_t LED_UPDATE_TASK_PRIORITY = 3;
#if CONFIG_IDF_TARGET_ESP32S2
static constexpr BaseType_t LED_UPDATE_TASK_CORE = PRO_CPU_NUM;
#else
static constexpr BaseType_t LED_UPDATE_TASK_CORE = APP_CPU_NUM;
#endif
static constexpr TickType_t LED_UPDATE_INTERVAL = 
  pdMS_TO_TICKS(CONFIG_STATUS_LED_UPDATE_INTERVAL_MSEC);

static void led_update(void *arg)
{
  StatusLED *led = (StatusLED *)arg;
  while(true)
  {
    vTaskDelay(LED_UPDATE_INTERVAL);
    led->refresh();
  }
}

void StatusLED::start_task()
{
  xTaskCreatePinnedToCore(led_update, "StatusLED", LED_UPDATE_TASK_STACK, this,
                          LED_UPDATE_TASK_PRIORITY, nullptr /* task handle */,
                          LED_UPDATE_TASK_CORE);
}

void StatusLED::set(const LED led, const COLOR color, const bool on)
{
  colors_[led] = color;
  state_[led] = on;
}

void StatusLED::clear()
{
  for(int index = 0; index < LED::MAX_LED; index++)
  {
    colors_[index] = OFF;
    state_[index] = false;
  }
}

} // namespace esp32cs