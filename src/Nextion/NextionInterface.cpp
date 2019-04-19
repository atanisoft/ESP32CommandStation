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

#include <bits/stdc++.h> 

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

static constexpr char const * NEXTION_DISPLAY_TYPE_STRINGS[] = {
  "basic 3.2\"",
  "basic 3.5\"",
  "basic 5.0\"",
  "enhanced 3.2\"",
  "enhanced 3.5\"",
  "enhanced 5.0\"",
  "Unknown"
};

NEXTION_DEVICE_TYPE nextionDeviceType = NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY;

void nextionTask(void *param) {
  nextionPages[TITLE_PAGE]->display();
  static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE])->setStatusText(3, "Detected Screen type:");
  static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE])->setStatusText(4, NEXTION_DISPLAY_TYPE_STRINGS[nextionDeviceType]);
  while(true) {
    nextion.poll();
    vTaskDelay(NEXTION_INTERFACE_UPDATE_INTERVAL);
  }
}

void nextionInterfaceInit() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Init Nextion"));
#if NEXTION_UART_NUM == 2
  Serial2.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#elif NEXTION_UART_NUM == 1
  Serial1.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#else
  Serial.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#endif
  nextion.init();

  // attempt to identify the nextion display.
  constexpr uint8_t MAX_ATTEMPTS = 3;
  uint8_t attempt = 0;
  while(attempt++ <= MAX_ATTEMPTS && nextionDeviceType == NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY) {
    LOG(INFO, "[Nextion] [%d/%d] Attempting to identify the attached Nextion display", attempt, MAX_ATTEMPTS);
    nextion.sendCommand("DRAKJHSUYDGBNCJHGJKSHBDN");
    nextion.sendCommand("connect");
    String screenID = "";
    size_t res = nextion.receiveString(screenID, false);
    if(res && screenID.indexOf("comok") >= 0) {
      // break the returned string into its comma delimited chunks
      // start after the first space
      std::stringstream buf(screenID.substring(screenID.indexOf(' ') + 1).c_str());
      std::vector<std::string> parts;
      std::string part;
      while(getline(buf, part, ',')) {
        parts.push_back(part);
      }

      // attempt to parse device model
      if(parts[2].compare(0, 7, "NX4024K") == 0) {
        nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_2_DISPLAY;
      } else if(parts[2].compare(0, 7, "NX4024T") == 0) {
        nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_2_DISPLAY;
      } else if(parts[2].compare(0, 7, "NX4832K") == 0) {
        nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_5_DISPLAY;
      } else if(parts[2].compare(0, 7, "NX4832T") == 0) {
        nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_5_DISPLAY;
      } else if(parts[2].compare(0, 7, "NX8048K") == 0) {
        nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_5_DISPLAY;
      } else if(parts[2].compare(0, 7, "NX8048T") == 0) {
        nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_5_DISPLAY;
      } else {
        LOG(WARNING, "[Nextion] Unrecognized Nextion Device model: %s", parts[2].c_str());
      }
      LOG(INFO, "[Nextion] Device type: %s", NEXTION_DISPLAY_TYPE_STRINGS[nextionDeviceType]);
      LOG(INFO, "[Nextion] Firmware Version: %s", parts[3].c_str());
      LOG(INFO, "[Nextion] MCU Code: %s", parts[4].c_str());
      LOG(INFO, "[Nextion] Serial #: %s", parts[5].c_str());
      LOG(INFO, "[Nextion] Flash size: %s bytes", parts[6].c_str());
    } else {
      LOG(WARNING, "[Nextion] Unable to determine Nextion device type: %s", screenID.c_str());
    }
  }
  if(nextionDeviceType == NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY) {
    LOG(WARNING, "[Nextion] Failed to identify the attached Nextion display, defaulting to 3.2\" basic display");
    nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_2_DISPLAY;
  }
  xTaskCreate(nextionTask, "Nextion", NEXTION_INTERFACE_TASK_STACK_SIZE,
    NULL, NEXTION_INTERFACE_TASK_PRIORITY, &_nextionTaskHandle);
}

#endif
