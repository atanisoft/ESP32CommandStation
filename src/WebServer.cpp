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
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <StringArray.h>

#include "WebServer.h"
#include "MotorBoard.h"
#include "Outputs.h"
#include "Turnouts.h"
#include "Sensors.h"
#include "index_html.h"

DCCPPWebServer::DCCPPWebServer(AsyncWebServer & webServer) {
  webServer.rewrite("/", "/index.html");
  webServer.on("/index.html", HTTP_GET,
    [](AsyncWebServerRequest *request) {
      const char * htmlBuildTime = __DATE__ " " __TIME__;
      if (request->header("If-Modified-Since").equals(htmlBuildTime)) {
        request->send(304);
      } else {
        AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", indexHtmlGz, indexHtmlGz_size);
        response->addHeader("Content-Encoding", "gzip");
        response->addHeader("Last-Modified", htmlBuildTime);
        request->send(response);
      }
    });
  webServer.on("/programmer", HTTP_GET | HTTP_POST,
    std::bind(&DCCPPWebServer::handleProgrammer, this, std::placeholders::_1));
  webServer.on("/powerStatus", HTTP_GET,
    std::bind(&DCCPPWebServer::handlePowerStatus, this, std::placeholders::_1));
  webServer.on("/outputs", HTTP_GET | HTTP_POST | HTTP_PUT | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleOutputs, this, std::placeholders::_1));
  webServer.on("/turnouts", HTTP_GET | HTTP_POST | HTTP_PUT | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleTurnouts, this, std::placeholders::_1));
  webServer.on("/sensors", HTTP_GET | HTTP_POST | HTTP_PUT | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleSensors, this, std::placeholders::_1));
}

void DCCPPWebServer::handleProgrammer(AsyncWebServerRequest *request) {
 	auto jsonResponse = new AsyncJsonResponse();
	// new programmer request
	if (request->method() == HTTP_GET) {
		if (request->arg("target") == "true") {
			jsonResponse->setCode(405);
		} else {
      uint16_t cvNumber = request->arg(F("cv")).toInt();
      int16_t cvValue = readCV(cvNumber);
			JsonObject &node = jsonResponse->getRoot();
			node[F("cv")] = cvNumber;
      node[F("value")] = cvValue;
      if(cvValue < 0) {
        jsonResponse->setCode(500);
      } else {
        jsonResponse->setCode(200);
     }
		}
  } else if(request->method() == HTTP_POST) {
    if (request->arg("target") == "true") {
      if(request->hasArg("bit")) {
        writeOpsCVBit(request->arg(F("loco")).toInt(), request->arg(F("cv")).toInt(), request->arg(F("bit")).toInt(), request->arg(F("bitValue")) == F("true"));
      } else {
        writeOpsCVByte(request->arg(F("loco")).toInt(), request->arg(F("cv")).toInt(), request->arg(F("value")).toInt());
      }
			jsonResponse->setCode(200);
		} else {
      bool writeSuccess = false;
      if(request->hasArg("bit")) {
        writeSuccess = writeProgCVBit(request->arg(F("cv")).toInt(), request->arg(F("bit")).toInt(), request->arg(F("bitValue")) == F("true"));
      } else {
        writeSuccess = writeProgCVByte(request->arg(F("cv")).toInt(), request->arg(F("value")).toInt());
      }
      if(writeSuccess) {
        jsonResponse->setCode(200);
      } else {
        jsonResponse->setCode(500);
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
  auto jsonResponse = new AsyncJsonResponse();
  if (request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    OutputManager::getState(array);
  } else if(request->method() == HTTP_POST) {
  } else if(request->method() == HTTP_DELETE) {
  } else if(request->method() == HTTP_PUT) {
   uint16_t outputID = request->arg(F("id")).toInt();
   bool state = request->arg(F("state")).toInt() == 1;
   OutputManager::set(outputID, state);
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void DCCPPWebServer::handleTurnouts(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse();
  if (request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    TurnoutManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    uint16_t turnoutID = request->arg(F("id")).toInt();
    uint16_t turnoutAddress = request->arg(F("address")).toInt();
    uint8_t turnoutSubAddress = request->arg(F("subAddress")).toInt();
    if(!TurnoutManager::create(turnoutID, turnoutAddress, turnoutSubAddress)) {
      jsonResponse->setCode(406);
      jsonResponse->getRoot()[F("message")] = F("Duplicate ID");
    }
  } else if(request->method() == HTTP_DELETE) {
    uint16_t turnoutID = request->arg(F("id")).toInt();
    if(!TurnoutManager::remove(turnoutID)) {
      jsonResponse->setCode(404);
    }
  } else if(request->method() == HTTP_PUT) {
    uint16_t turnoutID = request->arg(F("id")).toInt();
    bool state = request->arg(F("state")).toInt() == 1;
    TurnoutManager::set(turnoutID, state);
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void DCCPPWebServer::handleSensors(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse();
  if (request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    SensorManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    uint16_t sensorID = request->arg(F("id")).toInt();
    uint8_t sensorPin = request->arg(F("pin")).toInt();
    bool sensorPullUp = request->arg(F("pullUp")).toInt() == 1;
    if(!SensorManager::create(sensorID, sensorPin, sensorPullUp)) {
      jsonResponse->setCode(406);
      jsonResponse->getRoot()[F("message")] = F("Duplicate ID or Pin");
    }
  } else if(request->method() == HTTP_DELETE) {
    uint16_t sensorID = request->arg(F("id")).toInt();
    if(!SensorManager::remove(sensorID)) {
      jsonResponse->setCode(404);
    }
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}
