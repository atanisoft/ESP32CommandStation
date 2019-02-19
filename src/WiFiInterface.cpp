/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2017-2019 Mike Dunston

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
#include <esp_log.h>
#include <esp_wifi_internal.h>
#include <esp_task_wdt.h>

#if defined(HC12_RADIO_ENABLED) && HC12_RADIO_ENABLED
#include "HC12Interface.h"
#endif

static constexpr char const * WIFI_ENC_TYPES[] = {
  "OPEN",
  "WEP",
  "WPA (PSK)",
  "WPA2 (PSK)",
  "WPA/WPA2 (PSK)",
  "WPA2 Enterprise"
};

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

  bool update() {
    uint8_t buf[128];
    while (_client.available()) {
      auto len = _client.available();
      log_v("[%s] reading %d bytes", _client.remoteIP().toString().c_str(), len);
      auto added = _client.readBytes(&buf[0], len < 128 ? len : 128);
      feed(&buf[0], added);
    }
    return _client.connected();
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
WiFiServer DCCppServer(DCCPP_JMRI_CLIENT_PORT);
LinkedList<WiFiClientWrapper *> DCCppClients([](WiFiClientWrapper *consumer) {consumer->stop(); delete consumer; });
bool wifiConnected = false;

static constexpr const char *WIFI_STATUS_STRINGS[] =
{
    "WiFi Idle",            // WL_IDLE_STATUS
    "SSID not found",       // WL_NO_SSID_AVAIL
    "SSID scan completed",  // WL_SCAN_COMPLETED
    "WiFi connected",       // WL_CONNECTED
    "SSID connect failed",  // WL_CONNECT_FAILED
    "WiFi connection lost", // WL_CONNECTION_LOST
    "WiFi disconnected"     // WL_DISCONNECTED
};

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

  //esp_log_level_set("wifi", ESP_LOG_VERBOSE);
  //esp_wifi_internal_set_log_level(WIFI_LOG_VERBOSE);
  //esp_wifi_internal_set_log_mod(WIFI_LOG_MODULE_ALL, WIFI_LOG_SUBMODULE_ALL, true);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.onEvent([](system_event_id_t event) {
    if (wifiConnected) {
      return;
    }
    wifiConnected = true;
#if INFO_SCREEN_ENABLED
  #if INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS < 20
    InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, WiFi.localIP().toString().c_str());
  #else
    InfoScreen::printf(3, INFO_SCREEN_IP_ADDR_LINE, WiFi.localIP().toString().c_str());
  #endif
#endif
    log_i("WiFi IP: %s", WiFi.localIP().toString().c_str());

    if (!MDNS.begin(HOSTNAME)) {
      log_e("Failed to start mDNS");
    } else {
      log_i("Adding dccpp.tcp service to mDNS advertiser");
      MDNS.addService("dccpp", "tcp", DCCPP_JMRI_CLIENT_PORT);
    }

    DCCppServer.setNoDelay(true);
    DCCppServer.begin();
    dccppWebServer.begin();
#if LCC_ENABLED
    lccInterface.startWiFiDependencies();
#endif
  }, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent([](system_event_id_t event) {
    wifiConnected = false;
#if INFO_SCREEN_ENABLED
  #if INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS < 20
    InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, "Disconnected");
  #else
    InfoScreen::printf(3, INFO_SCREEN_IP_ADDR_LINE, "Disconnected");
  #endif
#endif
  }, SYSTEM_EVENT_STA_LOST_IP);
  WiFi.onEvent([](system_event_id_t event) {
    if(wifiConnected) {
      log_e("Connection to WiFi lost, reconnecting...");
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    }
  }, SYSTEM_EVENT_STA_DISCONNECTED);

  WiFi.mode(WIFI_STA);
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("WiFi Connecting"));
  log_i("WiFi details:\nHostname:%s\nMAC:%s\nSSID: %s", HOSTNAME, WiFi.macAddress().c_str(), wifiSSID.c_str());
  WiFi.setHostname(HOSTNAME);
  if (WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str()) != WL_CONNECT_FAILED) {
    log_i("Waiting for WiFi to connect");
    // this call waits up to 10sec for a result before timing out so it needs to be called a few times
    // until we get a real final result
    uint8_t attemptsRemaining = 10;
    uint8_t wifiStatus = WiFi.waitForConnectResult();
    while(wifiStatus != WL_CONNECTED && wifiStatus != WL_NO_SSID_AVAIL && wifiStatus != WL_CONNECT_FAILED && attemptsRemaining--) {
      esp_task_wdt_reset();
      log_i("WiFi not connected yet, status: %d (%s), attempts remaining: %d", wifiStatus, WIFI_STATUS_STRINGS[wifiStatus], attemptsRemaining);
      wifiStatus = WiFi.waitForConnectResult();
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
#if INFO_SCREEN_ENABLED
  #if INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS < 20
		InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, F("WiFi Connection"));
    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Failed"));
  #else
    InfoScreen::printf(3, INFO_SCREEN_IP_ADDR_LINE, F("Failed"));
    if (WiFi.status() == WL_NO_SSID_AVAIL) {
      InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("SSID not found"));
    } else {
      InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Generic WiFi fail"));
    }
  #endif
#endif
    if (WiFi.status() == WL_NO_SSID_AVAIL) {
      // since we couldn't find the configured SSID, perform a scan to help in
      // troubleshooting.
      int networks = WiFi.scanNetworks();
      if(networks) {
        log_i("Available WiFi networks:");
        for(int index = 0; index < networks; index++) {
          log_i("SSID: %s (RSSI: %d) Encryption: %s",
            WiFi.SSID(index), WiFi.RSSI(index), WIFI_ENC_TYPES[WiFi.encryptionType(index)]);
        }
      } else {
        log_w("Unable to find any WiFi networks!");
      }
    }
    log_e("WiFI connect failed, restarting");
    esp32_restart();
  } else {
    log_i("WiFi connected!");
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
    if(!client->update()) {
      log_d("dropping dead connection from %s", client->getClient().remoteIP().toString().c_str());
      DCCppClients.remove(client);
    }
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
#if HC12_RADIO_ENABLED
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
