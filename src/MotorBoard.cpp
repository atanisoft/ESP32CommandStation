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

#ifndef ADC_CURRENT_ATTENUATION
#define ADC_CURRENT_ATTENUATION ADC_ATTEN_DB_11
#endif

///////////////////////////////////////////////////////////////////////////////

const uint8_t motorBoardADCSampleCount = 50;
const uint16_t motorBoardCheckInterval = 250;
const uint16_t motorBoardCheckFaultCountdownInterval = 40;

LinkedList<GenericMotorBoard *> motorBoards([](GenericMotorBoard *board) {delete board; });

GenericMotorBoard::GenericMotorBoard(adc1_channel_t senseChannel, uint8_t enablePin,
  uint16_t triggerMilliAmps, uint32_t maxMilliAmps, String name) :
  _name(name), _senseChannel(senseChannel), _enablePin(enablePin),
  _maxMilliAmps(maxMilliAmps), _triggerValue(4096 * triggerMilliAmps / maxMilliAmps),
  _current(0), _state(false), _triggered(false), _triggerClearedCountdown(0),
  _triggerRecurrenceCount(0) {
  adc1_config_channel_atten(_senseChannel, ADC_CURRENT_ATTENUATION);
  pinMode(enablePin, OUTPUT);
  log_i("[%s] Configuring motor board [ADC1 Channel: %d, currentLimit: %d, enablePin: %d]",
    _name.c_str(), _senseChannel, _triggerValue, _enablePin);
}

void GenericMotorBoard::powerOn(bool announce) {
  log_i("[%s] Enabling DCC Signal", _name.c_str());
  digitalWrite(_enablePin, HIGH);
  _state = true;
	if(announce) {
		wifiInterface.printf(F("<p1 %s>"), _name.c_str());
	}
}

void GenericMotorBoard::powerOff(bool announce, bool overCurrent) {
  log_i("[%s] Disabling DCC Signal", _name.c_str());
  digitalWrite(_enablePin, LOW);
  _state = false;
	if(announce) {
		if(overCurrent) {
			wifiInterface.printf(F("<p2 %s>"), _name.c_str());
		} else {
			wifiInterface.printf(F("<p0 %s>"), _name.c_str());
		}
	}
}

void GenericMotorBoard::showStatus() {
	if(_state) {
		wifiInterface.printf(F("<p1 %s>"), _name.c_str());
    wifiInterface.printf(F("<a %s %d>"), _name.c_str(), getLastRead());
	} else {
		wifiInterface.printf(F("<p0 %s>"), _name.c_str());
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

uint64_t GenericMotorBoard::captureSample(uint8_t sampleCount) {
  uint64_t averageReading = 0;
  int successfulReads = 0;
  for(uint8_t sampleReadCount = 0; sampleReadCount < sampleCount; sampleReadCount++) {
    int reading = adc1_get_raw(_senseChannel);
#if DEBUG_ADC_SAMPLING == 1
    log_d("ADC(%d) sample %d/%d: %d", _senseChannel, sampleReadCount+1, sampleCount, reading);
#endif
    if(reading > 0) {
      averageReading += reading;
      successfulReads++;
    }
    delay(2);
  }
  if(successfulReads) {
    averageReading /= successfulReads;
  }
#if DEBUG_ADC_SAMPLING == 1
  log_d("ADC(%d) average: %d", averageReading);
#endif
  return averageReading;
}

GenericMotorBoard * MotorBoardManager::registerBoard(adc1_channel_t sensePin, uint8_t enablePin, MOTOR_BOARD_TYPE type, String name) {
  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("%s Init"), name.c_str());
  GenericMotorBoard *board;
  switch(type) {
    case ARDUINO_SHIELD:
      board = new GenericMotorBoard(sensePin, enablePin, 980, 2000, name);
      break;
    case POLOLU:
      board = new GenericMotorBoard(sensePin, enablePin, 2750, 3000, name);
      break;
    case BTS7960B_5A:
      board = new GenericMotorBoard(sensePin, enablePin, 5000, 43000, name);
      break;
    case BTS7960B_10A:
      board = new GenericMotorBoard(sensePin, enablePin, 10000, 43000, name);
      break;
  }
  motorBoards.add(board);
  return board;
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
    board->powerOn();
  }
#if INFO_SCREEN_TRACK_POWER_LINE >= 0
  InfoScreen::printf(13, INFO_SCREEN_TRACK_POWER_LINE, F("ON   "));
#endif
}

void MotorBoardManager::powerOffAll() {
  log_i("Disabling DCC Signal for all boards");
  for (const auto& board : motorBoards) {
    board->powerOff();
  }
#if INFO_SCREEN_TRACK_POWER_LINE >= 0
  InfoScreen::printf(13, INFO_SCREEN_TRACK_POWER_LINE, F("OFF  "));
#endif
}

bool MotorBoardManager::powerOn(const String name) {
  for (const auto& board : motorBoards) {
    if(name.equalsIgnoreCase(board->getName()) == 0) {
      board->powerOn();
      return true;
    }
  }
  return false;
}

bool MotorBoardManager::powerOff(const String name) {
  for (const auto& board : motorBoards) {
    if(name.equalsIgnoreCase(board->getName()) == 0) {
      board->powerOff();
      return true;
    }
  }
  return false;
}

int MotorBoardManager::getLastRead(const String name) {
  for (const auto& board : motorBoards) {
    if(name.equalsIgnoreCase(board->getName()) == 0) {
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
    board[F("name")] = motorBoard->getName();
    if(motorBoard->isOn()) {
      board[F("state")] = F("Normal");
    } else if(motorBoard->isOverCurrent()) {
      board[F("state")] = F("Fault");
    } else {
      board[F("state")] = F("Off");
    }
    board[F("usage")] = motorBoard->getCurrentDraw();
 	}
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
