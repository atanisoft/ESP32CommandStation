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

#ifndef _DETECTORS_H_
#define _DETECTORS_H_
#define _MAX_BLOCK  50
#define _MAX_TRAIN  10
#define _BLOCK_TIMEOUT  60000
#define _PREFIX "block"

#include <ArduinoJson.h>
#include "DCCppProtocol.h"

class Detector {
public:
  const int getTrain() {
    return _trainID;      // 0 for empty, -1 for unused
  }
  void setTrain(int);
  bool check();
private:
  int _trainID = -1;
  int _prevTrain = -1;
  unsigned long _lastOccupied = 0;
};


class DetectorManager {
public:
  static void init();
  static void show();
  static void check();
  static uint8_t getMaxBlock() {
    return _maxBlock;
  }
  static void setMaxBlock(uint8_t);

private:
  static uint8_t _maxBlock;
};

class DetectorCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<String>);
  String getID() {
    return "D";
  }
};

#endif
