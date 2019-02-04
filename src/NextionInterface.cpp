/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2018 NormHal
COPYRIGHT (c) 2018 Mike Dunston

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
TaskHandle_t NextionInterface::_taskHandle;

DCCPPNextionPage *nextionPages[MAX_PAGES] = {
  new NextionTitlePage(nextion),
  new NextionAddressPage(nextion),
  new NextionThrottlePage(nextion),
  new NextionTurnoutPage(nextion),
  nullptr,
  nullptr
};

void NextionInterface::init() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Init Nextion"));
#if NEXTION_UART_NUM == 2
  Serial2.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#elif NEXTION_UART_NUM == 1
  Serial1.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#else
  Serial.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
#endif
  if(nextion.init()) {
    xTaskCreate(nextionTask, "NextionInterface", DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, &_taskHandle);
  } else {
    log_e("Nextion init failed");
  }
}

void NextionInterface::nextionTask(void *param) {
  nextionPages[TITLE_PAGE]->display();
  bool showingTitlePage = true;
  uint64_t startupTransition = millis() + 2500;
  while(true) {
    if(showingTitlePage && startupTransition <= millis()) {
      showingTitlePage = false;
      nextionPages[THROTTLE_PAGE]->display();
    }
    nextion.poll();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

#endif
