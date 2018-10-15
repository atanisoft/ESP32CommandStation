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

#include "WebServer.h"
#include "Outputs.h"
#include "Turnouts.h"
#include "S88Sensors.h"
#include "RemoteSensors.h"
#include "index_html.h"

#if defined(NEXTION_ENABLED) && NEXTION_ENABLED
#include "NextionInterface.h"
#endif

enum HTTP_STATUS_CODES {
  STATUS_OK = 200,
  STATUS_NOT_MODIFIED = 304,
  STATUS_BAD_REQUEST = 400,
  STATUS_NOT_FOUND = 404,
  STATUS_NOT_ALLOWED = 405,
  STATUS_NOT_ACCEPTABLE = 406,
  STATUS_CONFLICT = 409,
  STATUS_PRECONDITION_FAILED = 412,
  STATUS_SERVER_ERROR = 500
};

class WebSocketClient : public DCCPPProtocolConsumer {
public:
  WebSocketClient(int clientID, IPAddress remoteIP) : _id(clientID), _remoteIP(remoteIP) {
  }
  virtual ~WebSocketClient() {}
  int getID() {
    return _id;
  }
  String getName() {
    return _remoteIP.toString() + "/" + String(_id);
  }
private:
  uint32_t _id;
  IPAddress _remoteIP;
};
LinkedList<WebSocketClient *> webSocketClients([](WebSocketClient *client) {delete client;});

DCCPPWebServer::DCCPPWebServer() : AsyncWebServer(80), webSocket("/ws") {
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
    root[JSON_S88_NODE] = JSON_VALUE_TRUE;
#else
    root[JSON_S88_NODE] = JSON_VALUE_FALSE;
#endif
    jsonResponse->setCode(STATUS_OK);
    jsonResponse->setLength();
    request->send(jsonResponse);
  });
  on("/programmer", HTTP_GET | HTTP_POST,
    std::bind(&DCCPPWebServer::handleProgrammer, this, std::placeholders::_1));
  on("/power", HTTP_GET | HTTP_PUT,
    std::bind(&DCCPPWebServer::handlePower, this, std::placeholders::_1));
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
  on("/remoteSensors", HTTP_GET | HTTP_POST | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleRemoteSensors, this, std::placeholders::_1));
  on("/config", HTTP_POST | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleConfig, this, std::placeholders::_1));
  on("/locomotive", HTTP_GET | HTTP_POST | HTTP_PUT | HTTP_DELETE,
    std::bind(&DCCPPWebServer::handleLocomotive, this, std::placeholders::_1));
  webSocket.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client,
      AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      webSocketClients.add(new WebSocketClient(client->id(), client->remoteIP()));
      client->printf("DCC++ESP32 v%s. READY!", VERSION);
  #if INFO_SCREEN_WS_CLIENTS_LINE >= 0
      InfoScreen::printf(12, INFO_SCREEN_WS_CLIENTS_LINE, F("%02d"), webSocketClients.length());
  #endif
    } else if (type == WS_EVT_DISCONNECT) {
      WebSocketClient *toRemove = nullptr;
      for (const auto& clientNode : webSocketClients) {
        if(clientNode->getID() == client->id()) {
          toRemove = clientNode;
        }
      }
      if(toRemove != nullptr) {
        webSocketClients.remove(toRemove);
      }
  #if INFO_SCREEN_WS_CLIENTS_LINE >= 0
      InfoScreen::printf(12, INFO_SCREEN_WS_CLIENTS_LINE, F("%02d"), webSocketClients.length());
  #endif
    } else if (type == WS_EVT_DATA) {
      for (const auto& clientNode : webSocketClients) {
        if(clientNode->getID() == client->id()) {
          clientNode->feed(data, len);
        }
      }
    }
  });
  addHandler(&webSocket);
}

void DCCPPWebServer::handleProgrammer(AsyncWebServerRequest *request) {
 	auto jsonResponse = new AsyncJsonResponse();
  if(!MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG)->isOn()) {
    MotorBoardManager::powerOn(MOTORBOARD_NAME_PROG);
  }
	// new programmer request
	if (request->method() == HTTP_GET) {
		if (request->arg(JSON_PROG_ON_MAIN.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE)) {
			jsonResponse->setCode(STATUS_NOT_ALLOWED);
		} else if(request->hasArg(JSON_IDENTIFY_NODE.c_str())) {
      JsonObject &node = jsonResponse->getRoot();
      int16_t decoderConfig = readCV(CV_NAMES::DECODER_CONFIG);
      uint16_t decoderAddress = 0;
      if(decoderConfig > 0) {
        if(bitRead(decoderConfig, DECODER_CONFIG_BITS::DECODER_TYPE)) {
          uint8_t decoderManufacturer = readCV(CV_NAMES::DECODER_MANUFACTURER);
          int16_t addrMSB = readCV(CV_NAMES::ACCESSORY_DECODER_MSB_ADDRESS);
          int16_t addrLSB = readCV(CV_NAMES::SHORT_ADDRESS);
          if(addrMSB >= 0 && addrLSB >= 0) {
            if(decoderManufacturer == 0xA5) { // MERG uses 7 bit LSB
              decoderAddress = (uint16_t)(((addrMSB & 0x07) << 7) | (addrLSB & 0x7F));
            } else if(decoderManufacturer == 0x19) { // Team Digital uses 8 bit LSB and 4 bit MSB
              decoderAddress = (uint16_t)(((addrMSB & 0x0F) << 8) | addrLSB);
            } else { // NMRA spec shows 6 bit LSB
              decoderAddress = (uint16_t)(((addrMSB & 0x07) << 6) | (addrLSB & 0x1F));
            }
            node[JSON_ADDRESS_MODE_NODE] = JSON_VALUE_LONG_ADDRESS;
          } else {
            log_w("Failed to read address MSB/LSB");
            jsonResponse->setCode(STATUS_SERVER_ERROR);
          }
        } else {
          if(bitRead(decoderConfig, DECODER_CONFIG_BITS::SHORT_OR_LONG_ADDRESS)) {
            int16_t addrMSB = readCV(CV_NAMES::LONG_ADDRESS_MSB_ADDRESS);
            int16_t addrLSB = readCV(CV_NAMES::LONG_ADDRESS_LSB_ADDRESS);
            if(addrMSB >= 0 && addrLSB >= 0) {
              decoderAddress = (uint16_t)(((addrMSB & 0xFF) << 8) | (addrLSB & 0xFF));
              node[JSON_ADDRESS_MODE_NODE] = JSON_VALUE_LONG_ADDRESS;
            } else {
              log_w("Unable to read address MSB/LSB");
              jsonResponse->setCode(STATUS_SERVER_ERROR);
            }
          } else {
            int16_t shortAddr = readCV(CV_NAMES::SHORT_ADDRESS);
            if(shortAddr > 0) {
              decoderAddress = shortAddr;
              node[JSON_ADDRESS_MODE_NODE] = JSON_VALUE_SHORT_ADDRESS;
            } else {
              log_w("Unable to read short address CV");
              jsonResponse->setCode(STATUS_SERVER_ERROR);
            }
          }
          if(bitRead(decoderConfig, DECODER_CONFIG_BITS::SPEED_TABLE)) {
            node[JSON_SPEED_TABLE_NODE] = JSON_VALUE_ON;
          } else {
            node[JSON_SPEED_TABLE_NODE] = JSON_VALUE_OFF;
          }
        }
        if(decoderAddress > 0) {
          node[JSON_ADDRESS_NODE] = decoderAddress;
          auto roster = LocomotiveManager::getRosterEntry(decoderAddress, false);
          if(roster) {
            node[JSON_LOCO_NODE] = roster;
          } else if(request->hasArg(JSON_CREATE_NODE.c_str()) && request->arg(JSON_CREATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE)) {
            roster = LocomotiveManager::getRosterEntry(decoderAddress);
            if(decoderConfig > 0) {
              if(bitRead(decoderConfig, DECODER_CONFIG_BITS::DECODER_TYPE)) {
                roster->setType(JSON_VALUE_STATIONARY_DECODER);
              } else {
                roster->setType(JSON_VALUE_MOBILE_DECODER);
              }
            }
            node[JSON_LOCO_NODE] = roster;
          }
        } else {
          log_w("Failed to read decoder address");
          jsonResponse->setCode(STATUS_SERVER_ERROR);
        }
      } else {
        log_w("Failed to read decoder configuration");
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      }
    } else {
      uint16_t cvNumber = request->arg(JSON_CV_NODE.c_str()).toInt();
      int16_t cvValue = readCV(cvNumber);
			JsonObject &node = jsonResponse->getRoot();
			node[JSON_CV_NODE] = cvNumber;
      node[JSON_VALUE_NODE] = cvValue;
      if(cvValue < 0) {
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      } else {
        jsonResponse->setCode(STATUS_OK);
     }
		}
  } else if(request->method() == HTTP_POST && request->hasArg(JSON_PROG_ON_MAIN.c_str())) {
    if (request->arg(JSON_PROG_ON_MAIN.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE)) {
      if(request->hasArg(JSON_CV_BIT_NODE.c_str())) {
        writeOpsCVBit(request->arg(JSON_ADDRESS_NODE.c_str()).toInt(), request->arg(JSON_CV_NODE.c_str()).toInt(),
          request->arg(JSON_CV_BIT_NODE.c_str()).toInt(), request->arg(JSON_VALUE_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE));
      } else {
        writeOpsCVByte(request->arg(JSON_ADDRESS_NODE.c_str()).toInt(), request->arg(JSON_CV_NODE.c_str()).toInt(),
          request->arg(JSON_VALUE_NODE.c_str()).toInt());
      }
			jsonResponse->setCode(STATUS_OK);
		} else {
      bool writeSuccess = false;
      if(request->hasArg(JSON_CV_BIT_NODE.c_str())) {
        writeSuccess = writeProgCVBit(request->arg(JSON_CV_NODE.c_str()).toInt(), request->arg(JSON_CV_BIT_NODE.c_str()).toInt(),
          request->arg(JSON_VALUE_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE));
      } else {
        writeSuccess = writeProgCVByte(request->arg(JSON_CV_NODE.c_str()).toInt(), request->arg(JSON_VALUE_NODE.c_str()).toInt());
      }
      if(writeSuccess) {
        jsonResponse->setCode(STATUS_OK);
      } else {
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      }
		}
  } else {
    jsonResponse->setCode(STATUS_BAD_REQUEST);
  }
  log_d("Setting response size");
	jsonResponse->setLength();
  log_d("Sending response, %d bytes", jsonResponse->getSize());
	request->send(jsonResponse);
 }

void DCCPPWebServer::handlePower(AsyncWebServerRequest *request) {
 	auto jsonResponse = new AsyncJsonResponse(true);
  if(request->method() == HTTP_GET) {
    log_i("/power: %d", request->params());
    if(request->params()) {
      jsonResponse->getRoot()[JSON_STATE_NODE] = MotorBoardManager::isTrackPowerOn() ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
    } else {
      JsonArray &array = jsonResponse->getRoot();
      MotorBoardManager::getState(array);
    }
  } else if (request->method() == HTTP_PUT) {
    if(request->hasArg(JSON_OVERALL_STATE_NODE.c_str())) {
      if(request->arg(JSON_OVERALL_STATE_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE)) {
        MotorBoardManager::powerOnAll();
      } else {
        MotorBoardManager::powerOffAll();
      }
    } else if(request->hasArg(JSON_NAME_NODE.c_str())) {
      if(request->arg(JSON_STATE_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE)) {
        MotorBoardManager::powerOn(request->arg(JSON_NAME_NODE.c_str()));
      } else {
        MotorBoardManager::powerOff(request->arg(JSON_NAME_NODE.c_str()));
      }
    } else {
      jsonResponse->setCode(STATUS_BAD_REQUEST);
    }
  }
 	jsonResponse->setLength();
 	request->send(jsonResponse);
 }

void DCCPPWebServer::handleOutputs(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if (request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    OutputManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    uint16_t outputID = request->arg(JSON_ID_NODE.c_str()).toInt();
    uint8_t pin = request->arg(JSON_PIN_NODE.c_str()).toInt();
    bool inverted = request->arg(JSON_INVERTED_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE);
    bool forceState = request->arg(JSON_FORCE_STATE_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE);
    bool defaultState = request->arg(JSON_DEFAULT_STATE_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE);
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
    if(!OutputManager::createOrUpdate(outputID, pin, outputFlags)) {
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    }
  } else if(request->method() == HTTP_DELETE) {
    if(!OutputManager::remove(request->arg(JSON_ID_NODE.c_str()).toInt())) {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  } else if(request->method() == HTTP_PUT) {
   OutputManager::toggle(request->arg(JSON_ID_NODE.c_str()).toInt());
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
    uint16_t turnoutID = request->arg(JSON_ID_NODE.c_str()).toInt();
    uint16_t turnoutAddress = request->arg(JSON_ADDRESS_NODE.c_str()).toInt();
    uint8_t turnoutSubAddress = request->arg(JSON_SUB_ADDRESS_NODE.c_str()).toInt();
    TurnoutManager::createOrUpdate(turnoutID, turnoutAddress, turnoutSubAddress);
  } else if(request->method() == HTTP_DELETE) {
    uint16_t turnoutID = request->arg(JSON_ID_NODE.c_str()).toInt();
    if(!TurnoutManager::remove(turnoutID)) {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  } else if(request->method() == HTTP_PUT) {
    uint16_t turnoutID = request->arg(JSON_ID_NODE.c_str()).toInt();
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
    uint16_t sensorID = request->arg(JSON_ID_NODE.c_str()).toInt();
    uint8_t sensorPin = request->arg(JSON_PIN_NODE.c_str()).toInt();
    bool sensorPullUp = request->arg(JSON_PULLUP_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE);
    if(!SensorManager::createOrUpdate(sensorID, sensorPin, sensorPullUp)) {
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    }
  } else if(request->method() == HTTP_DELETE) {
    uint16_t sensorID = request->arg(JSON_ID_NODE.c_str()).toInt();
    if(SensorManager::getSensorPin(sensorID) < 0) {
      // attempt to delete S88/RemoteSensor
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
      request->arg(JSON_ID_NODE.c_str()).toInt(),
      request->arg(JSON_PIN_NODE.c_str()).toInt(),
      request->arg(JSON_COUNT_NODE.c_str()).toInt())) {
      // duplicate pin/id
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    }
  } else if(request->method() == HTTP_DELETE) {
    S88BusManager::removeBus(request->arg(JSON_ID_NODE.c_str()).toInt());
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}
#endif

void DCCPPWebServer::handleRemoteSensors(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if(request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    RemoteSensorManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    RemoteSensorManager::createOrUpdate(request->arg(JSON_ID_NODE.c_str()).toInt(),
      request->arg(JSON_VALUE_NODE.c_str()).toInt());
  } else if(request->method() == HTTP_DELETE) {
    RemoteSensorManager::remove(request->arg(JSON_ID_NODE.c_str()).toInt());
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void DCCPPWebServer::handleLocomotive(AsyncWebServerRequest *request) {
  // method - url pattern - meaning
  // ANY /locomotive/estop - send emergency stop to all locomotives
  // GET /locomotive/roster - roster
  // GET /locomotive/roster?address=<address> - get roster entry
  // PUT /locomotive/roster?address=<address> - update roster entry
  // POST /locomotive/roster?address=<address> - create roster entry
  // DELETE /locomotive/roster?address=<address> - delete roster entry
  // GET /locomotive - get active locomotives
  // POST /locomotive?address=<address> - add locomotive to active management
  // GET /locomotive?address=<address> - get locomotive state
  // PUT /locomotive?address=<address>&speed=<speed>&dir=[FWD|REV]&fX=[true|false] - Update locomotive state, fX is short for function X where X is 0-28.
  // DELETE /locomotive?address=<address> - removes locomotive from active management
  auto jsonResponse = new AsyncJsonResponse(true);
  jsonResponse->setCode(STATUS_OK);
  const String url = request->url();
  // check if we have an eStop command, we don't care how this gets sent to the
  // base station (method) so check it first
  if(url.endsWith("/estop")) {
    LocomotiveManager::emergencyStop();
  } else if(url.indexOf("/roster") > 0) {
    if(request->method() == HTTP_GET && !request->hasArg(JSON_ADDRESS_NODE.c_str())) {
      LocomotiveManager::getRosterEntries(jsonResponse->getRoot());
    } else if (request->hasArg(JSON_ADDRESS_NODE.c_str())) {
      if(request->method() == HTTP_DELETE) {
        LocomotiveManager::removeRosterEntry(request->arg(JSON_ADDRESS_NODE).toInt());
      } else {
        RosterEntry *entry = LocomotiveManager::getRosterEntry(request->arg(JSON_ADDRESS_NODE).toInt());
        if(request->method() == HTTP_PUT || request->method() == HTTP_POST) {
          if(request->hasArg(JSON_DESCRIPTION_NODE.c_str())) {
            entry->setDescription(request->arg(JSON_DESCRIPTION_NODE));
          }
          if(request->hasArg(JSON_TYPE_NODE.c_str())) {
            entry->setType(request->arg(JSON_TYPE_NODE));
          }
          if(request->hasArg(JSON_IDLE_ON_STARTUP_NODE.c_str())) {
            entry->setIdleOnStartup(request->arg(JSON_IDLE_ON_STARTUP_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
          if(request->hasArg(JSON_DEFAULT_ON_THROTTLE_NODE.c_str())) {
            entry->setDefaultOnThrottles(request->arg(JSON_DEFAULT_ON_THROTTLE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
        }
        entry->toJson(jsonResponse->getRoot());
      }
    }
  } else {
    // Since it is not an eStop or roster command we need to check the request
    // method and ensure it contains the required arguments otherwise the
    // request should be rejected
    if(request->method() == HTTP_GET && !request->hasArg(JSON_ADDRESS_NODE.c_str())) {
      // get all active locomotives
      LocomotiveManager::getActiveLocos(jsonResponse->getRoot()); 
    } else if (request->hasArg(JSON_ADDRESS_NODE.c_str())) {
      auto loco = LocomotiveManager::getLocomotive(request->arg(JSON_ADDRESS_NODE.c_str()).toInt());
      if(request->method() == HTTP_PUT || request->method() == HTTP_POST) {
        // Creation / Update of active locomotive
        bool needUpdate = false;
        if(request->hasArg(JSON_IDLE_NODE.c_str()) && request->arg(JSON_IDLE_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE)) {
          loco->setIdle();
          needUpdate = true;
        }
        if(request->hasArg(JSON_DIRECTION_NODE.c_str())) {
          loco->setDirection(request->arg(JSON_DIRECTION_NODE.c_str()).equalsIgnoreCase(JSON_VALUE_FORWARD));
          needUpdate = true;
        }
        if(request->hasArg(JSON_SPEED_NODE.c_str())) {
          loco->setSpeed(request->arg(JSON_SPEED_NODE.c_str()).toInt());
          needUpdate = true;
        }
        for(uint8_t funcID = 0; funcID <=28 ; funcID++) {
          String fArg = "f" + String(funcID);
          if(request->hasArg(fArg.c_str())) {
            loco->setFunction(funcID, request->arg(fArg.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
        }
        if(needUpdate) {
          loco->sendLocoUpdate();
        }
      } else if(request->method() == HTTP_DELETE) {
        // Removal of an active locomotive
        LocomotiveManager::removeLocomotive(request->arg(JSON_ADDRESS_NODE.c_str()).toInt());
#if defined(NEXTION_ENABLED) && NEXTION_ENABLED
        static_cast<NextionThrottlePage *>(nextionPages[THROTTLE_PAGE])->invalidateLocomotive(request->arg(JSON_ADDRESS_NODE.c_str()).toInt());
#endif
      }
      loco->toJson(jsonResponse->getRoot());
    } else {
      // missing arg or unknown request
      jsonResponse->setCode(STATUS_BAD_REQUEST);
    }
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}
