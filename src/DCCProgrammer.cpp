/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2019 Mike Dunston

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

// number of samples to take when monitoring current after a CV verify
// (bit or byte) has been sent
static constexpr DRAM_ATTR uint8_t CVSampleCount = 150;

// number of attempts the programming track will make to read/write a CV
static constexpr uint8_t PROG_TRACK_CV_ATTEMPTS = 3;

// flag for when programming track is actively being used
bool progTrackBusy = false;

bool enterProgrammingMode() {
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpStartupLimit = (4096 * 100 / motorBoard->getMaxMilliAmps());

  // check if the programming track is already in use
  if(progTrackBusy) {
    return false;
  }

  // flag that we are currently using the programming track
  progTrackBusy = true;

  // energize the programming track
  motorBoard->powerOn(false);
  dccSignal[DCC_SIGNAL_PROGRAMMING]->startSignal(false);
  dccSignal[DCC_SIGNAL_PROGRAMMING]->waitForQueueEmpty();
  // give decoder time to start up and stabilize to under 100mA draw
  log_v("[PROG] waiting for power draw to stabilize");
  vTaskDelay(pdMS_TO_TICKS(100));

  // check that the current is under 100mA limit, this will take ~50ms
  if(motorBoard->captureSample(50) > milliAmpStartupLimit) {
    log_e("[PROG] current draw is over 100mA, aborting");
    leaveProgrammingMode();
    return false;
  }

  // delay for a short bit before entering programming mode
  vTaskDelay(pdMS_TO_TICKS(40));

  return true;
}

void leaveProgrammingMode() {
  if(!progTrackBusy) {
    return;
  }
  // deenergize the programming track
  MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG)->powerOff(false);
  dccSignal[DCC_SIGNAL_PROGRAMMING]->stopSignal();

  // reset flag to indicate the programming track is free
  progTrackBusy = false;
}

int16_t readCV(const uint16_t cv) {
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpAck = (4096 * 60 / motorBoard->getMaxMilliAmps());
  uint8_t readCVBitPacket[4] = { (uint8_t)(0x78 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), 0x00, 0x00};
  uint8_t verifyCVPacket[4] = { (uint8_t)(0x74 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), 0x00, 0x00};
  int16_t cvValue = -1;
  auto& signalGenerator = dccSignal[DCC_SIGNAL_PROGRAMMING];

  for(int attempt = 0; attempt < PROG_TRACK_CV_ATTEMPTS && cvValue == -1; attempt++) {
    log_i("[PROG %d/%d] Attempting to read CV %d", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv);
    if(attempt) {
      log_v("[PROG] Resetting DCC Decoder");
      signalGenerator->loadBytePacket(resetPacket, 2, 25);
      signalGenerator->waitForQueueEmpty();
    }

    // reset cvValue to all bits OFF
    cvValue = 0;
    for(uint8_t bit = 0; bit < 8; bit++) {
      log_v("[PROG] CV %d, bit [%d/7]", cv, bit);
      readCVBitPacket[2] = 0xE8 + bit;
      signalGenerator->loadBytePacket(resetPacket, 2, 3);
      signalGenerator->loadBytePacket(readCVBitPacket, 3, 5);
      signalGenerator->waitForQueueEmpty();
      if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
        log_v("[PROG] CV %d, bit [%d/7] ON", cv, bit);
        bitWrite(cvValue, bit, 1);
      } else {
        log_v("[PROG] CV %d, bit [%d/7] OFF", cv, bit);
      }
    }

    // verify the byte we received
    verifyCVPacket[2] = cvValue & 0xFF;
    log_i("[PROG %d/%d] Attempting to verify read of CV %d as %d", attempt+1, PROG_TRACK_CV_ATTEMPTS, cv, cvValue);
    signalGenerator->loadBytePacket(resetPacket, 2, 3);
    signalGenerator->loadBytePacket(verifyCVPacket, 3, 5);
    signalGenerator->waitForQueueEmpty();
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      log_i("[PROG] CV %d, verified as %d", cv, cvValue);
    } else {
      log_w("[PROG] CV %d, could not be verified", cv);
      cvValue = -1;
    }
  }
  log_i("[PROG] CV %d value is %d", cv, cvValue);
  return cvValue;
}

bool writeProgCVByte(const uint16_t cv, const uint8_t cvValue) {
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpAck = (4096 * 60 / motorBoard->getMaxMilliAmps());
  uint8_t writeCVBytePacket[4] = { (uint8_t)(0x7C + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), cvValue, 0x00};
  uint8_t verifyCVBytePacket[4] = { (uint8_t)(0x74 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), cvValue, 0x00};
  bool writeVerified = false;
  auto& signalGenerator = dccSignal[DCC_SIGNAL_PROGRAMMING];

  for(uint8_t attempt = 1; attempt <= PROG_TRACK_CV_ATTEMPTS && !writeVerified; attempt++) {
    log_i("[PROG %d/%d] Attempting to write CV %d as %d", attempt, PROG_TRACK_CV_ATTEMPTS, cv, cvValue);
    if(attempt) {
      log_v("[PROG] Resetting DCC Decoder");
      signalGenerator->loadBytePacket(resetPacket, 2, 25);
    }
    signalGenerator->loadBytePacket(resetPacket, 2, 3);
    signalGenerator->loadBytePacket(writeCVBytePacket, 3, 4);
    signalGenerator->waitForQueueEmpty();

    // verify that the decoder received the write byte packet and sent an ACK
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      signalGenerator->loadBytePacket(verifyCVBytePacket, 3, 5);
      signalGenerator->waitForQueueEmpty();
      // check that decoder sends an ACK for the verify operation
      if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
        writeVerified = true;
        log_i("[PROG] CV %d write value %d verified.", cv, cvValue);
      }
    } else {
      log_w("[PROG] CV %d write value %d could not be verified.", cv, cvValue);
    }
  }
  return writeVerified;
}

bool writeProgCVBit(const uint16_t cv, const uint8_t bit, const bool value) {
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpAck = (4096 * 60 / motorBoard->getMaxMilliAmps());
  uint8_t writeCVBitPacket[4] = { (uint8_t)(0x78 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), (uint8_t)(0xF0 + bit + value * 8), 0x00};
  uint8_t verifyCVBitPacket[4] = { (uint8_t)(0x74 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), (uint8_t)(0xB0 + bit + value * 8), 0x00};
  bool writeVerified = false;
  auto& signalGenerator = dccSignal[DCC_SIGNAL_PROGRAMMING];

  for(uint8_t attempt = 1; attempt <= PROG_TRACK_CV_ATTEMPTS && !writeVerified; attempt++) {
    log_d("[PROG %d/%d] Attempting to write CV %d bit %d as %d", attempt, PROG_TRACK_CV_ATTEMPTS, cv, bit, value);
    if(attempt) {
      log_v("[PROG] Resetting DCC Decoder");
      signalGenerator->loadBytePacket(resetPacket, 2, 3);
    }
    signalGenerator->loadBytePacket(writeCVBitPacket, 3, 4);
    signalGenerator->waitForQueueEmpty();

    // verify that the decoder received the write byte packet and sent an ACK
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      signalGenerator->loadBytePacket(resetPacket, 2, 3);
      signalGenerator->loadBytePacket(verifyCVBitPacket, 3, 5);
      signalGenerator->waitForQueueEmpty();
      // check that decoder sends an ACK for the verify operation
      if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
        writeVerified = true;
        log_i("[PROG %d/%d] CV %d write bit %d verified.", attempt, PROG_TRACK_CV_ATTEMPTS, cv, bit);
      }
    } else {
      log_w("[PROG %d/%d] CV %d write bit %d could not be verified.", attempt, PROG_TRACK_CV_ATTEMPTS, cv, bit);
    }
  }
  return writeVerified;
}

void writeOpsCVByte(const uint16_t locoAddress, const uint16_t cv, const uint8_t cvValue) {
  auto& signalGenerator = dccSignal[DCC_SIGNAL_OPERATIONS];
  log_d("[OPS] Updating CV %d to %d for loco %d", cv, cvValue, locoAddress);
  if(locoAddress > 127) {
    uint8_t writeCVBytePacket[] = {
      (uint8_t)(0xC0 | highByte(locoAddress)),
      lowByte(locoAddress),
      (uint8_t)(0xEC + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      cvValue,
      0x00};
    signalGenerator->loadBytePacket(writeCVBytePacket, 5, 4);
  } else {
    uint8_t writeCVBytePacket[] = {
      lowByte(locoAddress),
      (uint8_t)(0xEC + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      cvValue,
      0x00};
    signalGenerator->loadBytePacket(writeCVBytePacket, 4, 4);
  }
}

void writeOpsCVBit(const uint16_t locoAddress, const uint16_t cv, const uint8_t bit, const bool value) {
  auto& signalGenerator = dccSignal[DCC_SIGNAL_OPERATIONS];
  log_d("[OPS] Updating CV %d bit %d to %d for loco %d", cv, bit, value, locoAddress);
  if(locoAddress > 127) {
    uint8_t writeCVBitPacket[] = {
      (uint8_t)(0xC0 | highByte(locoAddress)),
      lowByte(locoAddress),
      (uint8_t)(0xE8 + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      (uint8_t)(0xF0 + bit + value * 8),
      0x00};
    signalGenerator->loadBytePacket(writeCVBitPacket, 5, 4);
  } else {
    uint8_t writeCVBitPacket[] = {
      lowByte(locoAddress),
      (uint8_t)(0xE8 + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      (uint8_t)(0xF0 + bit + value * 8),
      0x00};
    signalGenerator->loadBytePacket(writeCVBitPacket, 4, 4);
  }
}
