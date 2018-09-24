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
#include <IPAddress.h>
#include "WebServer.h"

#if defined(HC12_RADIO_ENABLED) && HC12_RADIO_ENABLED
#include "HC12Interface.h"
#endif

class WiFiClientWrapper : public DCCPPProtocolConsumer {
public:
  WiFiClientWrapper(WiFiClient client) : _client(client) {
    log_i("WiFiClient connected from %s", _client.remoteIP().toString().c_str());
    _client.setNoDelay(true);
  }

  virtual ~WiFiClientWrapper() {
    stop();
  }

  void stop() {
    log_i("Disconnecting %s", _client.remoteIP().toString().c_str());
    _client.stop();
  }

  void update() {
    uint8_t buf[128];
    //log_d("[%s] checking for available data", _client.remoteIP().toString().c_str());
    while (_client.available()) {
      auto len = _client.available();
      log_d("[%s] reading %d bytes", _client.remoteIP().toString().c_str(), len);
      auto added = _client.readBytes(&buf[0], len < 128 ? len : 128);
      feed(&buf[0], added);
    }
  }

  WiFiClient getClient() {
    return _client;
  }
private:
  WiFiClient _client;
};

const String wifiSSID = WIFI_SSID;
const String wifiPassword = WIFI_PASSWORD;

DCCPPWebServer dccppWebServer;
WiFiServer DCCppServer(DCCPP_CLIENT_PORT);
LinkedList<WiFiClientWrapper *> DCCppClients([](WiFiClientWrapper *consumer) {consumer->stop(); delete consumer; });
bool wifiConnected = false;

WiFiInterface::WiFiInterface() {
}

void WiFiInterface::begin() {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Init WiFI"));
	InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, F("IP:Pending"));
#if defined(WIFI_STATIC_IP_ADDRESS) && defined(WIFI_STATIC_IP_GATEWAY) && defined(WIFI_STATIC_IP_SUBNET)
	IPAddress staticIP, gatewayIP, subnetMask, dnsServer;
	staticIP.fromString(WIFI_STATIC_IP_ADDRESS);
	gatewayIP.fromString(WIFI_STATIC_IP_GATEWAY);
	subnetMask.fromString(WIFI_STATIC_IP_SUBNET);
#if defined(WIFI_STATIC_IP_DNS)
	dnsServer.fromString(WIFI_STATIC_IP_DNS);
#else
	dnsServer.fromString("8.8.8.8");
#endif
	WiFi.config(staticIP, gatewayIP, subnetMask, dnsServer);
#endif
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.onEvent([](system_event_id_t event) {
    if(wifiConnected) {
      return;
    }
    wifiConnected = true;
#if defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD && defined(INFO_SCREEN_LCD_COLUMNS) && INFO_SCREEN_LCD_COLUMNS < 20
    InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, WiFi.localIP().toString().c_str());
#else
    InfoScreen::printf(3, INFO_SCREEN_IP_ADDR_LINE, WiFi.localIP().toString().c_str());
#endif

    MDNS.begin(HOSTNAME);

    DCCppServer.setNoDelay(true);
    DCCppServer.begin();
    dccppWebServer.begin();
    MDNS.addService("dccpp", "tcp", DCCPP_CLIENT_PORT);
  }, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent([](system_event_id_t event) {
    wifiConnected = false;
#if defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD && defined(INFO_SCREEN_LCD_COLUMNS) && INFO_SCREEN_LCD_COLUMNS < 20
    InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, "Disconnected");
#else
    InfoScreen::printf(3, INFO_SCREEN_IP_ADDR_LINE, "Disconnected");
#endif
  }, SYSTEM_EVENT_STA_LOST_IP);

  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Connecting to AP"));
	log_i("Connecting to WiFi: %s", wifiSSID.c_str());
  WiFi.setHostname(HOSTNAME);
	WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
	log_i("Waiting for WiFi to connect");
  if(WiFi.waitForConnectResult() == WL_NO_SSID_AVAIL) {
#if defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD && defined(INFO_SCREEN_LCD_COLUMNS) && INFO_SCREEN_LCD_COLUMNS < 20
		InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, F("WiFi Connection"));
    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Failed, NO AP"));
#else
    InfoScreen::printf(3, INFO_SCREEN_IP_ADDR_LINE, F("Failed"));
    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("NO AP Found"));
#endif
		log_i("WiFI connect failed, restarting");
		ESP.restart();
	}
}

void WiFiInterface::update() {
	if (DCCppServer.hasClient()) {
    WiFiClient client = DCCppServer.available();
    if(client) {
      DCCppClients.add(new WiFiClientWrapper(client));
    }
  }
  for (const auto& client : DCCppClients) {
    //log_d("Checking TCP/IP Client(%p/%s) for data", client, client->getClient().remoteIP().toString().c_str());
    client->update();
  }
}

void WiFiInterface::showInitInfo() {
	printf(F("<N1: %s>"), WiFi.localIP().toString().c_str());
}

void WiFiInterface::send(const String &buf) {
  for (const auto& client : DCCppClients) {
    client->getClient().print(buf);
    delay(1);
  }
	dccppWebServer.broadcastToWS(buf);
#if defined(HC12_RADIO_ENABLED) && HC12_RADIO_ENABLED
	HC12Interface::send(buf);
#endif
}

void WiFiInterface::printf(const __FlashStringHelper *fmt, ...) {
	char buf[256] = {0};
	va_list args;
	va_start(args, fmt);
	vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
	va_end(args);
	send(buf);
}
