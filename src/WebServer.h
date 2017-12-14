/**********************************************************************
DCC++ BASE STATION FOR ESP32

COPYRIGHT (c) 2017 Mike Dunston

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
#include <ESPAsyncWebServer.h>

class DCCPPWebServer : public AsyncWebServer {
public:
  DCCPPWebServer();
  void begin() {
    AsyncWebServer::begin();
    #if (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED) || (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD_LINES > 2)
      InfoScreen::printf(0, 2, F("WS Clients: 0"));
    #endif
  }
  void broadcastToWS(const char *buf) {
    webSocket.textAll(buf);
  }
private:
  AsyncWebSocket webSocket;
  void handleESPInfo(AsyncWebServerRequest *);
  void handleProgrammer(AsyncWebServerRequest *);
  void handlePowerStatus(AsyncWebServerRequest *);
  void handleOutputs(AsyncWebServerRequest *);
  void handleTurnouts(AsyncWebServerRequest *);
  void handleSensors(AsyncWebServerRequest *);
};
