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
#include <AsyncJson.h>
#include <ESPmDNS.h>

#include "WebServer.h"
#include "MotorBoard.h"
#include "Outputs.h"
#include "Turnouts.h"
#include "Sensors.h"
#include "S88Sensors.h"
#include "index_html.h"

enum HTTP_STATUS_CODES {
  STATUS_OK = 200,
  STATUS_NOT_MODIFIED = 304,
  STATUS_NOT_FOUND = 404,
  STATUS_NOT_ALLOWED = 405,
  STATUS_NOT_ACCEPTABLE = 406,
  STATUS_CONFLICT = 409,
  STATUS_PRECONDITION_FAILED = 412,
  STATUS_SERVER_ERROR = 500
};

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

DCCPPWebServer::DCCPPWebServer() : AsyncWebServer(80), webSocket("/ws") {
  MDNS.addService("http", "tcp", 80);
  rewrite("/", "/index.html");
  on("/index.html", HTTP_GET,
    [](AsyncWebServerRequest *request) {
      const char * htmlBuildTime = __DATE__ " " __TIME__;
      if (request->header("If-Modified-Since").equals(htmlBuildTime)) {
        request->send(STATUS_NOT_MODIFIED);
      } else {
        AsyncWebServerResponse *response = request->beginResponse_P(STATUS_OK, "text/html", indexHtmlGz, indexHtmlGz_size);
        response->addHeader("Content-Encoding", "gzip");
        response->addHeader("Last-Modified", htmlBuildTime);
        request->send(response);
      }
    });
  on("/features", HTTP_GET, [](AsyncWebServerRequest *request) {
    auto jsonResponse = new AsyncJsonResponse();
    JsonObject &root = jsonResponse->getRoot();
#if defined(S88_ENABLED) && S88_ENABLED
    root[F("s88")] = "true";
#else
    root[F("s88")] = "false";
#endif
    jsonResponse->setCode(STATUS_OK);
    jsonResponse->setLength();
   	request->send(jsonResponse);
  });
  on("/programmer", HTTP_GET | HTTP_POST,
    std::bind(&DCCPPWebServer::handleProgrammer, this, std::placeholders::_1));
  on("/powerStatus", HTTP_GET,
    std::bind(&DCCPPWebServer::handlePowerStatus, this, std::placeholders::_1));
  on("/outputs", HTTP_GET | HTTP_POST | HTTP_PUT | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleOutputs, this, std::placeholders::_1));
  on("/turnouts", HTTP_GET | HTTP_POST | HTTP_PUT | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleTurnouts, this, std::placeholders::_1));
  on("/sensors", HTTP_GET | HTTP_POST | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleSensors, this, std::placeholders::_1));
#if defined(S88_ENABLED) && S88_ENABLED
  on("/s88sensors", HTTP_GET | HTTP_POST | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleS88Sensors, this, std::placeholders::_1));
#endif
  on("/config", HTTP_POST | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleConfig, this, std::placeholders::_1));
  webSocket.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client,
      AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      webSocketClients.add(new WebSocketClient(client->id()));
      client->printf("DCC++ESP v%s. READY!", VERSION);
  #if INFO_SCREEN_WS_CLIENTS_LINE >= 0
      InfoScreen::printf(12, INFO_SCREEN_WS_CLIENTS_LINE, F("%02d"), webSocketClients.length());
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
  #if INFO_SCREEN_WS_CLIENTS_LINE >= 0
      InfoScreen::printf(12, INFO_SCREEN_WS_CLIENTS_LINE, F("%02d"), webSocketClients.length());
  #endif
    } else if (type == WS_EVT_DATA) {
      for (const auto& clientNode : webSocketClients) {
        if(clientNode->getID() == client->id()) {
          clientNode->appendData(data, len);
        }
      }
    }
  });
  addHandler(&webSocket);
}

void DCCPPWebServer::handleProgrammer(AsyncWebServerRequest *request) {
 	auto jsonResponse = new AsyncJsonResponse();
	// new programmer request
	if (request->method() == HTTP_GET) {
		if (request->arg("target") == "true") {
			jsonResponse->setCode(STATUS_NOT_ALLOWED);
		} else {
      uint16_t cvNumber = request->arg(F("cv")).toInt();
      int16_t cvValue = readCV(cvNumber);
			JsonObject &node = jsonResponse->getRoot();
			node[F("cv")] = cvNumber;
      node[F("value")] = cvValue;
      if(cvValue < 0) {
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      } else {
        jsonResponse->setCode(STATUS_OK);
     }
		}
  } else if(request->method() == HTTP_POST) {
    if (request->arg("target") == "true") {
      if(request->hasArg("bit")) {
        writeOpsCVBit(request->arg(F("loco")).toInt(), request->arg(F("cv")).toInt(), request->arg(F("bit")).toInt(), request->arg(F("bitValue")) == F("true"));
      } else {
        writeOpsCVByte(request->arg(F("loco")).toInt(), request->arg(F("cv")).toInt(), request->arg(F("value")).toInt());
      }
			jsonResponse->setCode(STATUS_OK);
		} else {
      bool writeSuccess = false;
      if(request->hasArg("bit")) {
        writeSuccess = writeProgCVBit(request->arg(F("cv")).toInt(), request->arg(F("bit")).toInt(), request->arg(F("bitValue")) == F("true"));
      } else {
        writeSuccess = writeProgCVByte(request->arg(F("cv")).toInt(), request->arg(F("value")).toInt());
      }
      if(writeSuccess) {
        jsonResponse->setCode(STATUS_OK);
      } else {
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      }
		}
  }
	jsonResponse->setLength();
	request->send(jsonResponse);
 }

void DCCPPWebServer::handlePowerStatus(AsyncWebServerRequest *request) {
 	auto jsonResponse = new AsyncJsonResponse(true);
 	JsonArray &array = jsonResponse->getRoot();
  MotorBoardManager::getState(array);
 	jsonResponse->setLength();
 	request->send(jsonResponse);
 }

void DCCPPWebServer::handleOutputs(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if (request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    OutputManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    uint16_t outputID = request->arg(F("id")).toInt();
    uint8_t pin = request->arg(F("pin")).toInt();
    bool inverted = request->arg(F("inverted")) == "true";
    bool forceState = request->arg(F("forceState")) == "true";
    bool defaultState = request->arg(F("defaultState")) == "true";
    uint8_t outputFlags = 0;
    if(inverted) {
      bitSet(outputFlags, OUTPUT_IFLAG_INVERT);
    }
    if(forceState) {
      bitSet(outputFlags, OUTPUT_IFLAG_RESTORE_STATE);
      if(defaultState) {
        bitSet(outputFlags, OUTPUT_IFLAG_FORCE_STATE);
      }
    }
    OutputManager::createOrUpdate(outputID, pin, outputFlags);
  } else if(request->method() == HTTP_DELETE) {
    if(!OutputManager::remove(request->arg(F("id")).toInt())) {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  } else if(request->method() == HTTP_PUT) {
   OutputManager::toggle(request->arg(F("id")).toInt());
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void DCCPPWebServer::handleTurnouts(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if (request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    TurnoutManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    uint16_t turnoutID = request->arg(F("id")).toInt();
    uint16_t turnoutAddress = request->arg(F("address")).toInt();
    uint8_t turnoutSubAddress = request->arg(F("subAddress")).toInt();
    TurnoutManager::createOrUpdate(turnoutID, turnoutAddress, turnoutSubAddress);
  } else if(request->method() == HTTP_DELETE) {
    uint16_t turnoutID = request->arg(F("id")).toInt();
    if(!TurnoutManager::remove(turnoutID)) {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  } else if(request->method() == HTTP_PUT) {
    uint16_t turnoutID = request->arg(F("id")).toInt();
    TurnoutManager::toggle(turnoutID);
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void DCCPPWebServer::handleSensors(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if (request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    SensorManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    uint16_t sensorID = request->arg(F("id")).toInt();
    uint8_t sensorPin = request->arg(F("pin")).toInt();
    bool sensorPullUp = request->arg(F("pullUp")) == "true";
    if(sensorPin <= 0) {
      jsonResponse->setCode(STATUS_NOT_ACCEPTABLE);
    } else {
      SensorManager::createOrUpdate(sensorID, sensorPin, sensorPullUp);
    }
  } else if(request->method() == HTTP_DELETE) {
    uint16_t sensorID = request->arg(F("id")).toInt();
    if(SensorManager::getSensorPin(sensorID) < 0) {
      // attempt to delete S88
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    } else if(!SensorManager::remove(sensorID)) {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void DCCPPWebServer::handleConfig(AsyncWebServerRequest *request) {
  std::vector<String> arguments;
  if(request->method() == HTTP_POST) {
    DCCPPProtocolHandler::getCommandHandler("E")->process(arguments);
  } else {
    DCCPPProtocolHandler::getCommandHandler("e")->process(arguments);
  }
}

#if defined(S88_ENABLED) && S88_ENABLED
void DCCPPWebServer::handleS88Sensors(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if(request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    S88BusManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    if(!S88BusManager::createOrUpdateBus(
      request->arg(F("bus")).toInt(),
      request->arg(F("dataPin")).toInt(),
      request->arg(F("sensorCount")).toInt())) {
      // duplicate pin/id
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    }
  } else if(request->method() == HTTP_DELETE) {
    uint8_t sensorBus = request->arg(F("id")).toInt();
    S88BusManager::removeBus(sensorBus);
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}
#endif
