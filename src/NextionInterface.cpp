/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2018-2019 NormHal
COPYRIGHT (c) 2018-2019 Mike Dunston

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

#include "DCCppESP32.h"

#ifndef NEXTION_UART_NUM
#define NEXTION_UART_NUM 2
#endif
#ifndef NEXTION_UART_BAUD
#define NEXTION_UART_BAUD 115200
#endif
#ifndef NEXTION_RX_PIN
#define NEXTION_RX_PIN 14
#endif
#ifndef NEXTION_TX_PIN
#define NEXTION_TX_PIN 27
#endif

#if NEXTION_ENABLED

#if NEXTION_UART_NUM == 2
Nextion nextion(Serial2);
#elif NEXTION_UART_NUM == 1
Nextion nextion(Serial1);
#else
Nextion nextion(Serial);
#endif

TaskHandle_t _nextionTaskHandle;

constexpr uint16_t TITLE_SCREEN_TRANSITION_DELAY = 2500;
constexpr TickType_t NEXTION_INTERFACE_UPDATE_INTERVAL = pdMS_TO_TICKS(50);
constexpr uint8_t NEXTION_INTERFACE_TASK_PRIORITY = 2;
constexpr uint16_t NEXTION_INTERFACE_TASK_STACK_SIZE = DEFAULT_THREAD_STACKSIZE;

DCCPPNextionPage *nextionPages[MAX_PAGES] = {
  new NextionTitlePage(nextion),
  new NextionAddressPage(nextion),
  new NextionThrottlePage(nextion),
  new NextionTurnoutPage(nextion),
  nullptr,
  nullptr
};

NEXTION_DEVICE_TYPE nextionDeviceType = NEXTION_TYPE;

void nextionTask(void *param) {
  nextionPages[TITLE_PAGE]->display();
  bool showingTitlePage = true;
  uint64_t startupTransition = millis() + TITLE_SCREEN_TRANSITION_DELAY;
  while(true) {
    if(showingTitlePage && startupTransition <= millis()) {
      showingTitlePage = false;
      nextionPages[THROTTLE_PAGE]->display();
    }
    nextion.poll();
    vTaskDelay(NEXTION_INTERFACE_UPDATE_INTERVAL);
  }
}

static constexpr char const * NEXTION_DISPLAY_TYPE_STRINGS[] = {
  "basic 3.2\"",
  "basic 3.5\"",
  "enhanced 3.2\"",
  "enhanced 3.5\"",
  "Unknown"
};

void nextionInterfaceInit() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Init Nextion"));
#if NEXTION_UART_NUM == 2
  Serial2.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#elif NEXTION_UART_NUM == 1
  Serial1.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#else
  Serial.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#endif
  if(nextion.init()) {
    // attempt to identify the nextion display.
    constexpr uint8_t MAX_ATTEMPTS = 3;
    uint8_t attempt = 0;
    while(attempt++ <= MAX_ATTEMPTS && nextionDeviceType == NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY) {
      log_i("[%d/%d] Attempting to identify the attached Nextion display", attempt, MAX_ATTEMPTS);
      // if we haven't identified it with the basic command try and force the display out of
      // protocol reparse mode.
      if(attempt > 1) {
        nextion.sendCommand("DRAKJHSUYDGBNCJHGJKSHBDN");
      }
      if(attempt > 2) {
        // attempt to send a broadcast packet to the nextion to see if it responds to that instead
        nextion.sendCommand("%c%cconnect", 0xFF, 0xFF);
      } else {
        // send the default connect command
        nextion.sendCommand("connect");
      }
      String screenID = "";
      size_t res = nextion.receiveString(screenID, false);
      if(res && screenID.indexOf("comok") >= 0) {
        // break the returned string into its comma delimited chunks
        // start after the first space
        int index = screenID.indexOf(' ') + 1;
        std::vector<String> parts;
        while(index < screenID.length()) {
          int previousIndex = index;
          index = screenID.indexOf(',', previousIndex);
          if(index < 0) {
            index = screenID.length();
          }
          parts.push_back(screenID.substring(previousIndex, index));
        }
        for(auto s : parts) {
          log_i("part: %s", s.c_str());
        }
        // check the screen capabilities
        if(parts[0] != "1") {
          log_w("Attached Nextion does not appear to support touch!");
        }
        // detect screen address
        if(parts[1].indexOf("-") > 0 && parts[1].substring(parts[1].indexOf('-') + 1).toInt()) {
          log_i("Address: %d", parts[1].substring(parts[1].indexOf('-') + 1).toInt());
        }

        // attempt to parse device model
        if(parts[2].startsWith("NX4024")) {
          if(parts[2].indexOf("K") > 0) {
            nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_2_DISPLAY;
          } else {
            nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_2_DISPLAY;
          }
        } else if(parts[2].startsWith("NX4832")) {
          if(parts[2].indexOf("K") > 0) {
            nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_5_DISPLAY;
          } else {
            nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_5_DISPLAY;
          }
        } else {
          log_w("Unreognized Nextion Device model: %s", parts[2].c_str());
        }
        log_i("Device type: %s", NEXTION_DISPLAY_TYPE_STRINGS[nextionDeviceType]);
        log_i("Firmware Version: %S", parts[3].c_str());
        log_i("MCU Code: %S", parts[4].c_str());
        log_i("Serial #: %s", parts[5].c_str());
        log_i("Flash size: %d bytes", parts[6].toInt());
      } else {
        log_w("Unable to determine Nextion device type: %s", screenID.c_str());
      }
    }
    if(nextionDeviceType == NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY) {
      log_w("Failed to identify the attached Nextion display, defaulting to 3.2\" basic display");
      nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_2_DISPLAY;
    }
    xTaskCreate(nextionTask, "Nextion", NEXTION_INTERFACE_TASK_STACK_SIZE,
      NULL, NEXTION_INTERFACE_TASK_PRIORITY, &_nextionTaskHandle);
  } else {
    log_e("Nextion init failed");
  }
}

#endif
