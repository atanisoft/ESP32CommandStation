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
#include "NextionInterface.h"

#ifndef NEXTION_UART_NUM
#define NEXTION_UART_NUM 2
#endif
#ifndef NEXTION_UART_BAUD
#define NEXTION_UART_BAUD 115200
#endif
#ifndef NEXTION_RX_PIN
#define NEXTION_RX_PIN 12
#endif
#ifndef NEXTION_TX_PIN
#define NEXTION_TX_PIN 13
#endif

#if defined(NEXTION_ENABLED) && NEXTION_ENABLED

HardwareSerial nextionSerial(NEXTION_UART_NUM);
Nextion nextion(nextionSerial);

void NextionInterface::init() {
  nextionSerial.begin(NEXTION_UART_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);

  // defaults to page 0 as part of init
  nextion.init();

  // refresh the version string
  String versionCmd = String("Version.txt=") + VERSION;
  // cast is necessary due to a poor choice of types in base library :(
  nextion.sendCommand((char *)(versionCmd.c_str()));

  // redraw the screen after setting the version above (may not be needed)
  nextion.refresh();

  // delay on title screen
  sleep(2000);

  // switch to page 1
  nextion.sendCommand("page 1");
  // add any other init commands here

  nextion.refresh();
}

void NextionInterface::update() {
  nextion.poll();
}

#endif
