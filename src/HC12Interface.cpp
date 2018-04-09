/**********************************************************************
DCC++ BASE STATION FOR ESP32

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
std::vector<uint8_t> hc12Buffer;

void HC12Interface::init() {
  hc12Serial.begin(HC12_RADIO_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN);
}

void HC12Interface::update() {
  if(hc12Serial.available()) {
    auto len = hc12Serial.available();
    auto read_dest = hc12Buffer.insert(hc12Buffer.end(), len + 1, 0);
    auto added = hc12Serial.readBytes(&*read_dest, len);
    hc12Buffer.erase(read_dest + added, hc12Buffer.end());
    auto s = hc12Buffer.begin();
    auto consumed = hc12Buffer.begin();
    for(; s != hc12Buffer.end();) {
      s = std::find(s, hc12Buffer.end(), '<');
      auto e = std::find(s, hc12Buffer.end(), '>');
      if(s != hc12Buffer.end() && e != hc12Buffer.end()) {
        // discard the <
        s++;
        // discard the >
        *e = 0;
        String str(reinterpret_cast<char*>(&*s));
        wifiInterface.printf(F("<%s>"), str.c_str());
        log_d("Command: <%s>", str.c_str());
        DCCPPProtocolHandler::process(std::move(str));
        consumed = e;
      }
      s = e;
    }
    hc12Buffer.erase(hc12Buffer.begin(), consumed); // drop everything we used from the buffer.
  }
}

void HC12Interface::send(const char *buf) {
  hc12Serial.print(buf);
}
