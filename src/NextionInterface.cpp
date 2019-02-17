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
#if 0
    // identify nextion display
    nextion.sendCommand("");
    nextion.sendCommand("connect");
    String screenID;
    size_t res = nextion.receiveString(screenID, false);
    if(res) {
      // if we got back a valid string, check its content
      if(screenID.indexOf("comok") != -1 && screenID.indexOf("NX4024T032") != -1) {
        log_i("Nextion device is a 3.2\" basic display");
        nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_2_DISPLAY;
      } else if(screenID.indexOf("comok") != -1 && screenID.indexOf("NX4832T035") != -1) {
        log_i("Nextion device is a 3.5\" basic display");
        nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_5_DISPLAY;
      } else if(screenID.indexOf("comok") != -1 && screenID.indexOf("NX4024K032") != -1) {
        log_i("Nextion device is a 3.2\" enhanced display");
        nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_2_DISPLAY;
      } else if(screenID.indexOf("comok") != -1 && screenID.indexOf("NX4832K035") != -1) {
        log_i("Nextion device is a 3.5\" enhanced display");
        nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_5_DISPLAY;
      } else {
        log_w("Unable to determine Nextion device type: %s", screenID.c_str());
      }
    }
#endif
    xTaskCreate(nextionTask, "Nextion", NEXTION_INTERFACE_TASK_STACK_SIZE,
      NULL, NEXTION_INTERFACE_TASK_PRIORITY, &_nextionTaskHandle);
  } else {
    log_e("Nextion init failed");
  }
}

#endif
