/**********************************************************************
DCC COMMAND STATION FOR ESP32

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
#include "HC12Interface.h"

#ifndef HC12_RADIO_BAUD
#define HC12_RADIO_BAUD 19200
#endif
#ifndef HC12_UART_NUM
#define HC12_UART_NUM 1
#endif
#ifndef HC12_RX_PIN
#define HC12_RX_PIN 9
#endif
#ifndef HC12_TX_PIN
#define HC12_TX_PIN 10
#endif

HardwareSerial hc12Serial(HC12_UART_NUM);
DCCPPProtocolConsumer hc12Consumer;

void HC12Interface::init() {
  hc12Serial.begin(HC12_RADIO_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
}

void HC12Interface::update() {
  uint8_t buf[128];
  while (hc12Serial.available()) {
    auto len = hc12Serial.available();
    auto added = hc12Serial.readBytes(&buf[0], len < 128 ? len : 128);
    hc12Consumer.feed(&buf[0], added);
  }
}

void HC12Interface::send(const String &buf) {
  hc12Serial.print(buf);
}
