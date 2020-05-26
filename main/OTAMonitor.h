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

#ifndef OTA_MONITOR_H_
#define OTA_MONITOR_H_

#include "StatusDisplay.h"
#include "StatusLED.h"

#if CONFIG_NEXTION
#include "Interfaces/nextion/NextionInterface.h"
#endif

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>

class OTAMonitorFlow : public StateFlowBase, public Singleton<OTAMonitorFlow>
{
public:
  OTAMonitorFlow(Service *service) : StateFlowBase(service)
  {
#if CONFIG_NEXTION
    titlePage_ = static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
#endif
  }

  void report_start()
  {
    progress_ = 0;

#if CONFIG_STATUS_LED
    // set blink pattern to alternating green blink
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::WIFI, StatusLED::COLOR::GREEN_BLINK, true);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::OPS_TRACK, StatusLED::COLOR::GREEN_BLINK);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN_BLINK, true);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::EXT_1, StatusLED::COLOR::GREEN_BLINK);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::EXT_2, StatusLED::COLOR::GREEN_BLINK, true);
#endif // CONFIG_STATUS_LED
    Singleton<StatusDisplay>::instance()->status("Update starting");
#if CONFIG_NEXTION
    titlePage_->show();
    titlePage_->setStatusText(0, "OTA Started...");
#endif
  }

  void report_success()
  {
#if CONFIG_STATUS_LED
    // report successful OTA receipt
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::WIFI, StatusLED::COLOR::GREEN);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::OPS_TRACK, StatusLED::COLOR::GREEN);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::EXT_1, StatusLED::COLOR::GREEN);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::EXT_2, StatusLED::COLOR::GREEN);
#endif // CONFIG_STATUS_LED
#if CONFIG_NEXTION
    titlePage_->setStatusText(1, "Update Complete");
    titlePage_->setStatusText(2, "Rebooting");
#endif
    Singleton<StatusDisplay>::instance()->status("Update Complete");

    // reset countdown and trigger the restart
    countdown_ = StatusLED::LED::MAX_LED;
    start_flow(STATE(delay_and_reboot));
  }

  void report_failure(esp_err_t err)
  {
#if CONFIG_STATUS_LED
    // set blink pattern for alternating red blink
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::WIFI, StatusLED::COLOR::RED_BLINK, true);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::OPS_TRACK, StatusLED::COLOR::RED_BLINK);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::PROG_TRACK, StatusLED::COLOR::RED_BLINK, true);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::EXT_1, StatusLED::COLOR::RED_BLINK);
    Singleton<StatusLED>::instance()->setStatusLED(
      StatusLED::LED::EXT_2, StatusLED::COLOR::RED_BLINK, true);
#endif // CONFIG_STATUS_LED
#if CONFIG_NEXTION
    titlePage_->setStatusText(1, esp_err_to_name(err));
#endif
    Singleton<StatusDisplay>::instance()->status(esp_err_to_name(err));

    // restart the node due to failure
    start_flow(STATE(reboot_node));
   }

  void report_progress(uint32_t progress)
  {
    progress_ += progress;
    Singleton<StatusDisplay>::instance()->status("Recv: %d", progress_);
#if CONFIG_NEXTION
    titlePage_->setStatusText(1
                            , StringPrintf("Received: %d", progress_).c_str());
#endif
  }

private:
  StateFlowTimer timer_{this};
  uint8_t countdown_{StatusLED::LED::MAX_LED};
  uint32_t progress_{0};
#if CONFIG_NEXTION
  NextionTitlePage *titlePage_;
#endif

  Action reboot_node()
  {
    reboot();
    return exit();
  }

  Action delay_and_reboot()
  {
    if(countdown_ > 0)
    {
#if CONFIG_STATUS_LED
      // turn off lights
      Singleton<StatusLED>::instance()->setStatusLED(
        (StatusLED::LED)countdown_, StatusLED::COLOR::OFF);
#endif // CONFIG_STATUS_LED
      Singleton<StatusDisplay>::instance()->status("reboot in %2d...", countdown_);
      LOG(WARNING, "ESP32 will reboot in %d seconds...", countdown_);
      --countdown_;
      return sleep_and_call(&timer_, SEC_TO_NSEC(1), STATE(delay_and_reboot));
    }
    LOG(WARNING, "ESP32 will reboot NOW!");
    return call_immediately(STATE(reboot_node));
  }
};

#endif // OTA_MONITOR_H_