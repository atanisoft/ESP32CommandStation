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
#include <esp_event.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include "WebSocketClient.h"
#include "WebServer.h"

AsyncWebServer webServer(80);
DCCPPWebServer dccppWebServer(webServer);
WebSocketClientManager webSocketClientManager(webServer);

const String wifiSSID = WIFI_SSID;
const String wifiPassword = WIFI_PASSWORD;

WiFiServer DCCppServer(DCCPP_CLIENT_PORT);
WiFiClient DCCppClients[MAX_DCCPP_CLIENTS];
std::vector<uint8_t> DCCppClientBuffer[MAX_DCCPP_CLIENTS];

WiFiInterface::WiFiInterface() {
}

void WiFiInterface::begin() {
  InfoScreen::printf(0, 1, F("IP:Pending"));
  log_i("Connecting to WiFi: %s", wifiSSID.c_str());
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  log_i("Waiting for WiFi to connect");
  WiFi.waitForConnectResult();
  uint8_t wifiConnectAttempts = 250;
  while(WiFi.status() != WL_CONNECTED && wifiConnectAttempts-- > 0) {
    log_i("WiFi status: %d", WiFi.status());
    delay(250);
  }
  if(WiFi.status() != WL_CONNECTED) {
    InfoScreen::printf(3, 1, F("Failed"));
    log_i("WiFI connect failed, restarting");
    ESP.restart();
  }
  InfoScreen::printf(3, 1, WiFi.localIP().toString().c_str());
  DCCppServer.setNoDelay(true);
  DCCppServer.begin();
  webSocketClientManager.begin();
  webServer.begin();
}

void WiFiInterface::update() {
	if (DCCppServer.hasClient()) {
		for (int i = 0; i < MAX_DCCPP_CLIENTS; i++) {
			if (!DCCppClients[i] || !DCCppClients[i].connected()) {
				if (DCCppClients[i]) {
					DCCppClients[i].stop();
				}
				DCCppClients[i] = DCCppServer.available();
				continue;
			}
		}
		DCCppServer.available().stop();
	}
	//check clients for data
	for (int i = 0; i < MAX_DCCPP_CLIENTS; i++) {
		if (DCCppClients[i] && DCCppClients[i].connected()) {
			if (DCCppClients[i].available()) {
        auto len = DCCppClients[i].available();
				auto read_dest = DCCppClientBuffer[i].insert(DCCppClientBuffer[i].end(), len + 1, 0);
				auto added = DCCppClients[i].read(&*read_dest, len);
				DCCppClientBuffer[i].erase(read_dest + added, DCCppClientBuffer[i].end());
				auto s = DCCppClientBuffer[i].begin();
				auto consumed = DCCppClientBuffer[i].begin();
				for(; s != DCCppClientBuffer[i].end();) {
					s = std::find(s, DCCppClientBuffer[i].end(), '<');
					auto e = std::find(s, DCCppClientBuffer[i].end(), '>');
					if(s != DCCppClientBuffer[i].end() && e != DCCppClientBuffer[i].end()) {
            // discard the <
            s++;
            // discard the >
						*e = 0;
						String str(reinterpret_cast<char*>(&*s));
            printf(F("<%s>"), str.c_str());
            DCCPPProtocolHandler::process(std::move(str));
						consumed = e;
					}
					s = e;
				}
				DCCppClientBuffer[i].erase(DCCppClientBuffer[i].begin(), consumed); // drop everything we used from the buffer.
			}
		}
	}
}

void WiFiInterface::showInitInfo() {
	printf(F("<N1: %s>"), WiFi.localIP().toString().c_str());
}

void WiFiInterface::send(const char *buf) {
  for (int i = 0; i < MAX_DCCPP_CLIENTS; i++) {
    if (DCCppClients[i] && DCCppClients[i].connected()) {
      DCCppClients[i].print(buf);
      delay(1);
    }
  }
  webSocketClientManager.broadcast(buf);
}

void WiFiInterface::printf(const __FlashStringHelper *fmt, ...) {
	char buf[256] = {0};
	va_list args;
	va_start(args, fmt);
	vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
	va_end(args);
	send(buf);
}
