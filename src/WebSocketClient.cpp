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
#include "DCCppESP32.h"
#include <ESPAsyncWebServer.h>
#include "WebSocketClient.h"

AsyncWebSocket webSocket("/ws");

class WebSocketClient {
public:
  WebSocketClient(int clientID) : _id(clientID) {
    buffer.reserve(128);
  }
  int getID() {
    return _id;
  }
  void appendData(uint8_t * data, size_t len) {
    for(int i = 0; i < len; i++) {
      buffer.emplace_back(data[i]);
    }
    auto s = buffer.begin();
    auto consumed = buffer.begin();
    for(; s != buffer.end();) {
      s = std::find(s, buffer.end(), '<');
      auto e = std::find(s, buffer.end(), '>');
      if(s != buffer.end() && e != buffer.end()) {
        // discard the <
        s++;
        // discard the >
        *e = 0;
        String str(reinterpret_cast<char*>(&*s));
        wifiInterface.printf(F("<%s>"), str.c_str());
        DCCPPProtocolHandler::process(std::move(str));
        consumed = e;
      }
      s = e;
    }
    buffer.erase(buffer.begin(), consumed); // drop everything we used from the buffer.
  }
private:
  uint32_t _id;
  std::vector<uint8_t> buffer;
};
LinkedList<WebSocketClient *> webSocketClients([](WebSocketClient *client) {delete client;});

WebSocketClientManager::WebSocketClientManager(AsyncWebServer &webServer) {
  webServer.addHandler(&webSocket);
  webSocket.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client,
  		AwsEventType type, void * arg, uint8_t *data, size_t len) {
  	if (type == WS_EVT_CONNECT) {
      webSocketClients.add(new WebSocketClient(client->id()));
  		client->text(F("DCC++ESP v"));
      client->text(VERSION);
      client->text(F(". READY!"));
#if (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED) || (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD_LINES > 2)
      InfoScreen::printf(12, 2, F("%02d"), webSocketClients.length());
#endif
  	} else if (type == WS_EVT_DISCONNECT) {
      WebSocketClient *toRemove = NULL;
      for (const auto& clientNode : webSocketClients) {
        if(clientNode->getID() == client->id()) {
          toRemove = clientNode;
        }
      }
      if(toRemove != NULL) {
        webSocketClients.remove(toRemove);
      }
#if (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED) || (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD_LINES > 2)
      InfoScreen::printf(12, 2, F("%02d"), webSocketClients.length());
#endif
  	} else if (type == WS_EVT_DATA) {
      for (const auto& clientNode : webSocketClients) {
        if(clientNode->getID() == client->id()) {
          clientNode->appendData(data, len);
        }
      }
  	}
  });
}
void WebSocketClientManager::begin() {
#if (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED) || (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD_LINES > 2)
  InfoScreen::printf(0, 2, F("WS Clients: 0"));
#endif
}

void WebSocketClientManager::broadcast(const char *buf) {
  webSocket.textAll(buf);
}
