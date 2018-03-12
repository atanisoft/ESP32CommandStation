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

#include <Arduino.h>
#include <functional>
#include <StringArray.h>
#include <esp32-hal-log.h>

#include "DCCppESP32.h"
#include "Detectors.h"

/**********************************************************************

DCC++ESP32 BASE STATION supports external detector inputs that are connected via
wifi connection using telnet.  From JMRI's point of view, The detectors are added
the sensor input list but are not stored in flash memory.

During startup, the base station scans for Access Points that have a matching
_PREFIX defined in Detectors.h for example "block06".  The base station assumes that
blocks exist beginning with block01 up to an including the highest block number
found.  If none are found, then they will be added later when status is reported.

The following varations of the "D" command :

  <D BLOCK TRAIN>:    Informs the base station of the occupancy status of a given
                        BLOCK.  If TRAIN is 0, then it is unoccupied.  If TRAIN is
                        one or greater, than the block is occupied.
  <D BLOCK TRAIN MAX>:Same as above with an additional MAX variable that indicates
                        maximum block number found.
  <D>:                Lists all defined detectors.
                        returns: <D BLOCK TRAIN> for each defined detector or 
                        <X> if no detectors are defined.

where

  BLOCK:  the numeric ID (0-32667) of the detector
  TRAIN:  the numeric ID (0-32767) of the train occupying the block or zero if
                the block is unoccupied.
  MAX:    the numeric ID (0-32667) of the highest number detector available

All detectors defined as per above are repeatedly and sequentially checked within
the main loop of this sketch. If a Detector is found to have transitioned from
one state to another, one of the following serial messages are generated. 100 is
added to the Detector ID so they do not interfere with Sensors already defined
using the "S" command:

  <Q ID+100>   - for transition of Detector ID from HIGH state to LOW state
               (i.e. the sensor is triggered)
  <q ID+100>   - for transition of Detector ID from LOW state to HIGH state
               (i.e. the sensor is no longer triggered)

If status is not reported for a given block after _BLOCK_TIMEOUT milliseconds,
then that block is marked empty (zero).  It is therefore important that a
detector continually report its status via the "D" command.

Depending on whether the physical sensor is acting as an "event-trigger" or a
"detection-sensor," you may decide to ignore the <q ID> return and only react to
<Q ID> triggers.

**********************************************************************/

Detector block[_MAX_BLOCK];

uint8_t DetectorManager::_maxBlock = 0;

void Detector::setTrain( int trainNum ) {
  _trainID = trainNum;
  _lastOccupied = millis();
}

bool Detector::check() {
  if( millis() > _lastOccupied + _BLOCK_TIMEOUT )
    setTrain(0);   // delete the train
  if(_trainID != _prevTrain) {
    _prevTrain = _trainID;
    return true;
  } 
  else  
    return false;
}

void DetectorManager::init() {
  int networksFound, i, blockNum, maxBlock = 0;
  char buffer[80];

  Serial.println("Scanning for blocks");
  WiFi.scanNetworks(true);
  while( (networksFound = WiFi.scanComplete()) < 0 ) {
    delay( 100 );
    Serial.print("*");
  }
  for (i = 0; i < networksFound; i++) {
    Serial.println( WiFi.SSID(i) );
    strcpy( buffer, WiFi.SSID(i).c_str() );
    if( !strncmp( _PREFIX, buffer, strlen( _PREFIX ) ) ) {
       blockNum = atoi( &buffer[strlen(_PREFIX)] );
       if( blockNum > maxBlock )
          maxBlock = blockNum;
    }
  }
  Serial.println( maxBlock );
  setMaxBlock( maxBlock );
}

void DetectorManager::show(){
  int i;

  for( int i=1; i <= _maxBlock; i++ )
    wifiInterface.printf(F("<Q %d %d %d>"), i+100, i+100, false);
}

void DetectorManager::check(){
  int i, id;
 
  for( int i=1; i <= _maxBlock; i++ )
    if( block[i].check() )    // state changed?
      if( id = block[i].getTrain() )
        wifiInterface.printf(F("<Q %d>"), i+100);
      else
        wifiInterface.printf(F("<q %d>"), i+100);
}

void DetectorManager::setMaxBlock(uint8_t newMax ) {
  uint8_t i;
  if( newMax > _maxBlock) {
    for( i=_maxBlock; i<= newMax; i++ )
      if(block[i].getTrain() < 0)
        block[i].setTrain(0);
    _maxBlock = newMax;
  }
}

void DetectorCommandAdapter::process(const std::vector<String> arguments) {
  int blockNum, maxBlock = DetectorManager::getMaxBlock();
  if(arguments.empty()) {
    // list all detectors
    if( maxBlock )
      for( blockNum = 1; blockNum <= maxBlock; blockNum++ )
        wifiInterface.printf(F("<D %d %d>"), blockNum, block[blockNum].getTrain() ); 
    else
      wifiInterface.printf(F("<X>") ); 
  } else {
    switch(arguments.size()) {
      case 1: // delete train
        wifiInterface.printf(F("<X>")); 
        break;
      case 3: // adjust new max
        DetectorManager::setMaxBlock(arguments[2].toInt());
      case 2: 
        block[arguments[0].toInt()].setTrain(arguments[1].toInt());
        DetectorManager::setMaxBlock(arguments[0].toInt());  // in case it is bigger
        wifiInterface.printf(F("<O>"));
        break;
      default:
        wifiInterface.printf(F("<X>"));
    }
  }
}
