/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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

#include <ConfigurationManager.h>
#include <DCCSignalVFS.h>
#include <ESP32TrainDatabase.h>
#include <FreeRTOSTaskMonitor.h>
#include <freertos/task.h>
#include <Httpd.h>
#include <LCCStackManager.h>
#include <LCCWiFiManager.h>
#include <os/OS.hxx>
#include <StatusDisplay.h>
#include <StatusLED.h>
#include <Turnouts.h>
#include <utils/Singleton.hxx>

extern "C"
{

void *node_reboot(void *arg)
{
  // shutdown any background refresh tasks.
  esp32cs::shutdown_dcc_vfs();
  Singleton<FreeRTOSTaskMonitor>::instance()->stop();
  Singleton<StatusDisplay>::instance()->stop();
#if CONFIG_STATUS_LED
  Singleton<StatusLED>::instance()->stop();
#endif
  Singleton<TurnoutManager>::instance()->stop();
  Singleton<esp32cs::Esp32TrainDatabase>::instance()->stop();
  // sleep for 1 sec to give time for restart broadcast (if needed)
  usleep(1000);
  Singleton<http::Httpd>::instance()->executor()->shutdown();
  Singleton<esp32cs::LCCWiFiManager>::instance()->shutdown();
  Singleton<esp32cs::LCCStackManager>::instance()->shutdown();
  
  // shutdown and cleanup the configuration manager
  Singleton<ConfigurationManager>::instance()->shutdown();

  LOG(INFO, "Restarting ESP32 Command Station");
  // restart the node
  esp_restart();
}

void reboot()
{
  os_thread_create(nullptr, nullptr, uxTaskPriorityGet(NULL) + 1, 2048
                 , node_reboot, nullptr);
}

ssize_t os_get_free_heap()
{
  return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

void delay(unsigned long ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms));
}

unsigned long IRAM_ATTR micros()
{
  return (unsigned long)esp_timer_get_time();
}

unsigned long IRAM_ATTR millis()
{
  return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void IRAM_ATTR delayMicroseconds(uint32_t us)
{
  uint32_t m = micros();
  if(us){
    uint32_t e = (m + us);
    if(m > e){ //overflow
      while(micros() > e){
        asm volatile ("nop");
      }
    }
    while(micros() < e){
      asm volatile ("nop");
    }
  }
}

} // extern "C"
