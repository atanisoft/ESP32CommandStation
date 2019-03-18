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

#ifndef ADC_CURRENT_ATTENUATION
#define ADC_CURRENT_ATTENUATION ADC_ATTEN_DB_11
#endif

///////////////////////////////////////////////////////////////////////////////

const uint8_t motorBoardADCSampleCount = 50;
const uint16_t motorBoardCheckInterval = 250;
const uint16_t motorBoardCheckFaultCountdownInterval = 40;

LinkedList<GenericMotorBoard *> motorBoards([](GenericMotorBoard *board) {delete board; });

GenericMotorBoard::GenericMotorBoard(adc1_channel_t senseChannel, uint8_t enablePin,
  uint16_t triggerMilliAmps, uint32_t maxMilliAmps, String name, bool programmingTrack) :
  _name(name), _senseChannel(senseChannel), _enablePin(enablePin),
  _maxMilliAmps(maxMilliAmps), _triggerValue(4096 * triggerMilliAmps / maxMilliAmps),
  _progTrack(programmingTrack), _current(0), _state(false), _triggered(false),
  _triggerClearedCountdown(0), _triggerRecurrenceCount(0) {
  adc1_config_channel_atten(_senseChannel, ADC_CURRENT_ATTENUATION);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  log_i("[%s] Configuring motor board [ADC1 Channel: %d, currentLimit: %d, enablePin: %d]",
    _name.c_str(), _senseChannel, _triggerValue, _enablePin);
}

void GenericMotorBoard::powerOn(bool announce) {
  log_i("[%s] Enabling DCC Signal", _name.c_str());
  digitalWrite(_enablePin, HIGH);
  _state = true;
	if(announce) {
#if LOCONET_ENABLED
    locoNet.reportPower(true);
#endif
		wifiInterface.printf(F("<p1 %s>"), _name.c_str());
	}
}

void GenericMotorBoard::powerOff(bool announce, bool overCurrent) {
  log_i("[%s] Disabling DCC Signal", _name.c_str());
  digitalWrite(_enablePin, LOW);
  _state = false;
  if(!_progTrack) {
    if(announce) {
      if(overCurrent) {
#if LOCONET_ENABLED
        locoNet.send(OPC_IDLE, 0, 0);
#endif
			  wifiInterface.printf(F("<p2 %s>"), _name.c_str());
		  } else {
#if LOCONET_ENABLED
        locoNet.reportPower(false);
#endif
			  wifiInterface.printf(F("<p0 %s>"), _name.c_str());
      }
		}
	}
}

void GenericMotorBoard::showStatus() {
  if(!_progTrack) {
    if(_state) {
      wifiInterface.printf(F("<p1 %s>"), _name.c_str());
      wifiInterface.printf(F("<a %s %d>"), _name.c_str(), getLastRead());
    } else {
      wifiInterface.printf(F("<p0 %s>"), _name.c_str());
    }
	}
}

void GenericMotorBoard::check() {
	// if we have exceeded the CURRENT_SAMPLE_TIME we need to check if we are over/under current.
	if(millis() - _lastCheckTime > motorBoardCheckInterval) {
    _lastCheckTime = millis();
		_current = captureSample(motorBoardADCSampleCount);
		if(_current >= _triggerValue && isOn()) {
      log_i("[%s] Overcurrent detected %2.2f mA (raw: %d)", _name.c_str(), getCurrentDraw(), _current);
			powerOff(true, true);
			_triggered = true;
      _triggerClearedCountdown = motorBoardCheckFaultCountdownInterval;
      _triggerRecurrenceCount = 0;
    } else if(_current >= _triggerValue && _triggered) {
      _triggerRecurrenceCount++;
      log_i("[%s] Overcurrent persists (%d ms) %2.2f mA (raw: %d)", _name.c_str(), _triggerRecurrenceCount * motorBoardCheckInterval, getCurrentDraw(), _current);
		} else if(_current < _triggerValue && _triggered) {
      _triggerClearedCountdown--;
      if(_triggerClearedCountdown == 0) {
        log_i("[%s] Overcurrent cleared %2.2f mA, enabling (raw: %d)", _name.c_str(), getCurrentDraw(), _current);
  			powerOn();
  			_triggered=false;
      } else {
        log_i("[%s] Overcurrent cleared %2.2f mA, %d ms before re-enable (raw: %d)", _name.c_str(), _triggerClearedCountdown * motorBoardCheckInterval, getCurrentDraw(), _current);
      }
    }
	}
}

uint16_t GenericMotorBoard::captureSample(uint8_t sampleCount, bool logResults) {
  std::vector<int> readings;
  while(readings.size() < sampleCount) {
    readings.push_back(adc1_get_raw(_senseChannel));
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  auto avgReading = std::accumulate(readings.begin(), readings.end(), 0) / readings.size();
  if(logResults) {
    log_i("ADC(%d) average: %d, samples: %d", _senseChannel, avgReading, readings.size());
  }
  return avgReading;
}

void MotorBoardManager::registerBoard(adc1_channel_t sensePin, uint8_t enablePin, MOTOR_BOARD_TYPE type, String name, bool programmingTrack) {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s Init"), name.c_str());
  uint32_t maxAmps = 0;
  uint32_t triggerAmps = 0;
  switch(type) {
    case ARDUINO_SHIELD:
      maxAmps = 2000;
      triggerAmps = 1750;
      break;
    case POLOLU:
      maxAmps = 2500;
      triggerAmps = 2250;
      break;
    case BTS7960B_5A:
      maxAmps = 43000;
      triggerAmps = 5000;
      break;
    case BTS7960B_10A:
      maxAmps = 43000;
      triggerAmps = 10000;
      break;
  }
  // programming tracks need a much lower rate limit, the value below
  // gives a 20% buffer over RCN-216. the programming track code itself
  // will limit to ~250mA.
  if(programmingTrack) {
    triggerAmps = 300;
  }
  motorBoards.add(new GenericMotorBoard(sensePin, enablePin, triggerAmps, maxAmps, name, programmingTrack));
}

GenericMotorBoard *MotorBoardManager::getBoardByName(String name) {
  for (const auto& board : motorBoards) {
		if(board->getName() == name) {
      return board;
    }
	}
  return nullptr;
}

void MotorBoardManager::check() {
  for (const auto& board : motorBoards) {
		board->check();
	}
}

void MotorBoardManager::powerOnAll() {
  log_i("Enabling DCC Signal for all boards");
  for (const auto& board : motorBoards) {
    if(!board->isProgrammingTrack()) {
      board->powerOn(false);
      board->showStatus();
    }
  }
#if INFO_SCREEN_TRACK_POWER_LINE >= 0
  InfoScreen::printf(13, INFO_SCREEN_TRACK_POWER_LINE, F("ON   "));
#endif
#if LOCONET_ENABLED
  locoNet.reportPower(true);
#endif
  startDCCSignalGenerators();
}

void MotorBoardManager::powerOffAll() {
  log_i("Disabling DCC Signal for all boards");
  for (const auto& board : motorBoards) {
    board->powerOff(false);
    board->showStatus();
  }
#if INFO_SCREEN_TRACK_POWER_LINE >= 0
  InfoScreen::printf(13, INFO_SCREEN_TRACK_POWER_LINE, F("OFF  "));
#endif
#if LOCONET_ENABLED
  locoNet.reportPower(false);
#endif
  stopDCCSignalGenerators();
}

bool MotorBoardManager::powerOn(const String name) {
  for (const auto& board : motorBoards) {
    if(name.equalsIgnoreCase(board->getName())) {
      board->powerOn(false);
      board->showStatus();
      return true;
    }
  }
  return false;
}

bool MotorBoardManager::powerOff(const String name) {
  for (const auto& board : motorBoards) {
    if(name.equalsIgnoreCase(board->getName())) {
      board->powerOff(false);
      board->showStatus();
      return true;
    }
  }
  return false;
}

int MotorBoardManager::getLastRead(const String name) {
  for (const auto& board : motorBoards) {
    if(name.equalsIgnoreCase(board->getName())) {
      return board->getLastRead();
    }
  }
  return -1;
}

void MotorBoardManager::showStatus() {
  for (const auto& board : motorBoards) {
		board->showStatus();
	}
}

std::vector<String> MotorBoardManager::getBoardNames() {
  std::vector<String> boardNames;
  for (const auto& board : motorBoards) {
    boardNames.push_back(board->getName());
  }
  return boardNames;
}

uint8_t MotorBoardManager::getMotorBoardCount() {
  return motorBoards.length();
}

void MotorBoardManager::getState(JsonArray &array) {
  for (const auto& motorBoard : motorBoards) {
    JsonObject &board = array.createNestedObject();
    board[JSON_NAME_NODE] = motorBoard->getName();
    if(motorBoard->isOn()) {
      board[JSON_STATE_NODE] = JSON_VALUE_NORMAL;
      board[JSON_USAGE_NODE] = motorBoard->getCurrentDraw();
    } else if(motorBoard->isOverCurrent()) {
      board[JSON_STATE_NODE] = JSON_VALUE_FAULT;
      board[JSON_USAGE_NODE] = motorBoard->getCurrentDraw();
    } else {
      board[JSON_STATE_NODE] = JSON_VALUE_OFF;
      board[JSON_USAGE_NODE] = 0;
    }
 	}
}

bool MotorBoardManager::isTrackPowerOn() {
  bool state = false;
  for (const auto& motorBoard : motorBoards) {
    if(motorBoard->isOn()) {
      state = true;
    }
  }
  return state;
}

void CurrentDrawCommand::process(const std::vector<String> arguments) {
  if(arguments.size() == 0) {
    MotorBoardManager::showStatus();
  } else {
    wifiInterface.printf(F("<a %d %s>"), MotorBoardManager::getLastRead(arguments[0]), arguments[0].c_str());
  }
}

void PowerOnCommand::process(const std::vector<String> arguments) {
  if(arguments.size() == 0) {
    MotorBoardManager::powerOnAll();
  }
}

void PowerOffCommand::process(const std::vector<String> arguments) {
  if(arguments.size() == 0) {
    MotorBoardManager::powerOffAll();
  }
}
