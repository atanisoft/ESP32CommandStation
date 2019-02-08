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
#include <esp32-hal-timer.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

// Define constants for DCC Signal pattern

// this controls the timer tick frequency
#define DCC_TIMER_PRESCALE 80

// number of microseconds for each half of the DCC signal for a zero
#define DCC_ZERO_BIT_PULSE_DURATION 98

// number of microseconds for each half of the DCC signal for a one
#define DCC_ONE_BIT_PULSE_DURATION 58

// number of samples to take when monitoring current after a CV verify
// (bit or byte) has been sent
static constexpr DRAM_ATTR uint8_t CVSampleCount = 150;

SignalGenerator dccSignal[MAX_DCC_SIGNAL_GENERATORS];

// S-9.2 baseline packet (idle)
static const DRAM_ATTR uint8_t idlePacket[] = {0xFF, 0x00};
// S-9.2 baseline packet (decoder reset)
static const DRAM_ATTR uint8_t resetPacket[] = {0x00, 0x00};
// S-9.2 baseline packet (eStop, direction bit ignored)
static const DRAM_ATTR uint8_t eStopPacket[] = {0x00, 0x41};

// bitmask used by signal generator when processing DCC packet bytes
static const DRAM_ATTR uint8_t DCC_PACKET_BIT_MASK[] = {
  0x80, 0x40, 0x20, 0x10,
  0x08, 0x04, 0x02, 0x01
};

// number of attempts the programming track will make to read/write a CV
static constexpr uint8_t PROG_TRACK_CV_ATTEMPTS = 3;

// flag for when programming track is actively being used
bool progTrackBusy = false;

#define SIGNAL_PIN_CONFIG(name, signalPin) \
  log_i("[%s] Configuring signal pin %d", name, signalPin); \
  pinMode(signalPin, INPUT); \
  digitalWrite(signalPin, LOW); \
  pinMode(signalPin, OUTPUT);

#define INVERTED_SIGNAL_PIN_CONFIG(name, invertedSignalPin) \
  log_i("[%s] Configuring inverted signal pin %d", name, invertedSignalPin); \
  pinMode(invertedSignalPin, INPUT); \
  digitalWrite(invertedSignalPin, LOW); \
  pinMode(invertedSignalPin, OUTPUT); \

void configureDCCSignalGenerators() {
  SIGNAL_PIN_CONFIG("OPS", DCC_SIGNAL_PIN_OPERATIONS)
#if defined(DCC_SIGNAL_PIN_OPERATIONS_INVERTED)
  INVERTED_SIGNAL_PIN_CONFIG("OPS", DCC_SIGNAL_PIN_OPERATIONS_INVERTED)
#endif
  dccSignal[DCC_SIGNAL_OPERATIONS].configureSignal("OPS", 512, DCC_TIMER_OPERATIONS);

  SIGNAL_PIN_CONFIG("PROG", DCC_SIGNAL_PIN_PROGRAMMING)
#if defined(DCC_SIGNAL_PIN_PROGRAMMING_INVERTED)
  INVERTED_SIGNAL_PIN_CONFIG("PROG", DCC_SIGNAL_PIN_PROGRAMMING_INVERTED)
#endif
  dccSignal[DCC_SIGNAL_PROGRAMMING].configureSignal("PROG", 64, DCC_TIMER_PROGRAMMING);
}

void startDCCSignalGenerators() {
  // NOTE: DCC_SIGNAL_PROGRAMMING is intentionally not started here, it will be managed with
  // the programming track methods below.
  if(!dccSignal[DCC_SIGNAL_OPERATIONS].isEnabled()) {
    dccSignal[DCC_SIGNAL_OPERATIONS].startSignal();
  }
}

bool stopDCCSignalGenerators() {
  bool reEnableNeeded = dccSignal[DCC_SIGNAL_OPERATIONS].isEnabled();
  if(dccSignal[DCC_SIGNAL_OPERATIONS].isEnabled()) {
    dccSignal[DCC_SIGNAL_OPERATIONS].stopSignal();
  }
  if(dccSignal[DCC_SIGNAL_PROGRAMMING].isEnabled()) {
    dccSignal[DCC_SIGNAL_PROGRAMMING].stopSignal();
  }
  return reEnableNeeded;
}

bool isDCCSignalEnabled() {
  if(dccSignal[DCC_SIGNAL_OPERATIONS].isEnabled() || dccSignal[DCC_SIGNAL_PROGRAMMING].isEnabled()) {
    return true;
  }
  return false;
}

void sendDCCEmergencyStop() {
  for(auto generator : dccSignal) {
    if(generator.isEnabled()) {
      generator.loadBytePacket(eStopPacket, 2, 0, true);
    }
  }
}

void SignalGenerator::loadBytePacket(const uint8_t *data, uint8_t length, uint8_t repeatCount, bool drainToSendQueue) {
  std::vector<uint8_t> packet;
  for(int i = 0; i < length; i++) {
    packet.push_back(data[i]);
  }
  loadPacket(packet, repeatCount, drainToSendQueue);
}

void SignalGenerator::loadPacket(std::vector<uint8_t> data, int numberOfRepeats, bool drainToSendQueue) {
  if(drainToSendQueue) {
    portENTER_CRITICAL(&_sendQueueMUX);
    while(!isQueueEmpty()) {
      // pull the first packet from the pending to send queue
      Packet *packet = _toSend.front();
      _toSend.pop();
      // make that packet available for re-use
      _availablePackets.push(packet);
    }
    portEXIT_CRITICAL(&_sendQueueMUX);
  }
  while(_availablePackets.empty()) {
    delay(2);
  }
  Packet *packet = _availablePackets.front();
  _availablePackets.pop();

  packet->numberOfRepeats = numberOfRepeats;
  packet->currentBit = 0;

  // calculate checksum (XOR)
  // add first byte as checksum byte
  uint8_t checksum = data[0];
  for(int i = 1; i < data.size(); i++) {
    checksum ^= data[i];
  }
  data.push_back(checksum);

  // standard DCC preamble
  packet->buffer[0] = 0xFF;
  packet->buffer[1] = 0xFF;
  // first bit of actual data at the end of the preamble
  packet->buffer[2] = 0xFC + bitRead(data[0], 7);
  packet->buffer[3] = data[0] << 1;
  packet->buffer[4] = data[1];
  packet->buffer[5] = data[2] >> 1;
  packet->buffer[6] = data[2] << 7;

  if(data.size() == 3){
    packet->numberOfBits = 49;
  } else{
    packet->buffer[6] += data[3] >> 2;
    packet->buffer[7] = data[3] << 6;
    if(data.size() == 4){
      packet->numberOfBits = 58;
    } else{
      packet->buffer[7] += data[4] >> 3;
      packet->buffer[8] = data[4] << 5;
      if(data.size() == 5){
        packet->numberOfBits = 67;
      } else{
        packet->buffer[8] += data[5] >> 4;
        packet->buffer[9] = data[5] << 4;
        packet->numberOfBits = 76;
      } // >5 bytes
    } // >4 bytes
  } // >3 bytes

  portENTER_CRITICAL(&_sendQueueMUX);
  _toSend.push(packet);
  portEXIT_CRITICAL(&_sendQueueMUX);
}

#define UPDATE_DCC_PACKET(G) \
  if(G._currentPacket != nullptr) { \
    if(G._currentPacket->currentBit == G._currentPacket->numberOfBits) { \
      if(G._currentPacket->numberOfRepeats > 0) { \
        G._currentPacket->numberOfRepeats--; \
        G._currentPacket->currentBit = 0; \
      } else { \
        if(G._currentPacket != &G._idlePacket) { \
          G._availablePackets.push(G._currentPacket); \
        } \
        G._currentPacket = nullptr; \
      } \
    } \
  } \
  if (G._currentPacket == nullptr) { \
    portENTER_CRITICAL_ISR(&G._sendQueueMUX); \
    if(!G.isQueueEmpty()) { \
      G._currentPacket = G._toSend.front(); \
      G._toSend.pop(); \
    } else { \
      G._currentPacket = &G._idlePacket; \
      G._currentPacket->currentBit = 0; \
    } \
    portEXIT_CRITICAL_ISR(&G._sendQueueMUX); \
  }

#define DCC_SIGNAL_ISR_IMPL(G, S) \
  if(G._topOfWave) { \
    UPDATE_DCC_PACKET(G) \
    digitalWrite(S, HIGH); \
    if(G._currentPacket->buffer[G._currentPacket->currentBit / 8] & DCC_PACKET_BIT_MASK[G._currentPacket->currentBit % 8]) { \
      timerAlarmWrite(G._timer, DCC_ONE_BIT_PULSE_DURATION, false); \
    } else { \
      timerAlarmWrite(G._timer, DCC_ZERO_BIT_PULSE_DURATION, false); \
    } \
    G._currentPacket->currentBit++; \
  } else { \
    digitalWrite(S, LOW); \
  } \
  G._topOfWave = !G._topOfWave; \
  timerWrite(G._timer, 0); \
  timerAlarmEnable(G._timer);

#define DCC_SIGNAL_ISR_IMPL_WITH_INVERTED_SIGNAL_PIN(G, S, I) \
  if(G._topOfWave) { \
    UPDATE_DCC_PACKET(G) \
    digitalWrite(S, HIGH); \
    digitalWrite(I, LOW); \
    if(G._currentPacket->buffer[G._currentPacket->currentBit / 8] & DCC_PACKET_BIT_MASK[G._currentPacket->currentBit % 8]) { \
      timerAlarmWrite(G._timer, DCC_ONE_BIT_PULSE_DURATION, false); \
    } else { \
      timerAlarmWrite(G._timer, DCC_ZERO_BIT_PULSE_DURATION, false); \
    } \
    G._currentPacket->currentBit++; \
  } else { \
    digitalWrite(S, LOW); \
    digitalWrite(I, HIGH); \
  } \
  G._topOfWave = !G._topOfWave; \
  timerWrite(G._timer, 0); \
  timerAlarmEnable(G._timer);

void IRAM_ATTR signalGeneratorTimerISR_OPS(void)
{
#if defined(DCC_SIGNAL_PIN_OPERATIONS_INVERTED)
  DCC_SIGNAL_ISR_IMPL_WITH_INVERTED_SIGNAL_PIN(dccSignal[DCC_SIGNAL_OPERATIONS], DCC_SIGNAL_PIN_OPERATIONS, DCC_SIGNAL_PIN_OPERATIONS_INVERTED)
#else
  DCC_SIGNAL_ISR_IMPL(dccSignal[DCC_SIGNAL_OPERATIONS], DCC_SIGNAL_PIN_OPERATIONS)
#endif
}

void IRAM_ATTR signalGeneratorTimerISR_PROG(void)
{
#if defined(DCC_SIGNAL_PIN_PROGRAMMING_INVERTED)
  DCC_SIGNAL_ISR_IMPL_WITH_INVERTED_SIGNAL_PIN(dccSignal[DCC_SIGNAL_PROGRAMMING], DCC_SIGNAL_PIN_PROGRAMMING, DCC_SIGNAL_PIN_PROGRAMMING_INVERTED)
#else
  DCC_SIGNAL_ISR_IMPL(dccSignal[DCC_SIGNAL_PROGRAMMING], DCC_SIGNAL_PIN_PROGRAMMING)
#endif
}

void SignalGenerator::configureSignal(String name, uint16_t maxPackets, uint8_t timerNumber) {
  _name = name;
  _timerNumber = timerNumber;

  // create packets for this signal generator up front, they will be reused until
  // the base station is shutdown
  for(int index = 0; index < maxPackets; index++) {
    _availablePackets.push(new Packet());
  }
}

void SignalGenerator::startSignal(bool sendIdlePackets) {
  if(_enabled) {
    return;
  }
  // drain any pending packets before we start the signal so we start with an empty queue
  while(!isQueueEmpty()) {
    _currentPacket = _toSend.front();
    _toSend.pop();
    // make sure the packet is zeroed before pushing it back to the queue
    memset(_currentPacket, 0, sizeof(Packet));
    _availablePackets.push(_currentPacket);
  }

  // reset to initial state
  _topOfWave = true;
  _currentPacket = nullptr;

  // inject the required reset and idle packets into the queue
  // this is required as part of S-9.2.4 section A
  // at least 20 reset packets and 10 idle packets must be sent upon initialization
  // of the base station to force decoders to exit service mode.
  log_i("[%s] Adding reset packet (25 repeats) to packet queue", _name.c_str());
  loadBytePacket(resetPacket, 2, 25);
  if(sendIdlePackets) {
    log_i("[%s] Adding idle packet to packet queue", _name.c_str());
    loadBytePacket(idlePacket, 2, 10);
  }

  log_i("[%s] Configuring Timer(%d) for generating DCC Signal", _name.c_str(),_timerNumber);
  _timer = timerBegin(_timerNumber, DCC_TIMER_PRESCALE, true);
  log_i("[%s] Attaching interrupt handler to Timer(%d)", _name.c_str(), _timerNumber);
  if(_timerNumber == DCC_TIMER_OPERATIONS) {
    timerAttachInterrupt(_timer, &signalGeneratorTimerISR_OPS, true);
  } else {
    timerAttachInterrupt(_timer, &signalGeneratorTimerISR_PROG, true);
  }
  log_i("[%s] Configuring alarm on Timer(%d) to %dus", _name.c_str(), _timerNumber, DCC_ONE_BIT_PULSE_DURATION);
  timerAlarmWrite(_timer, DCC_ONE_BIT_PULSE_DURATION, true);
  log_i("[%s] Setting load on Timer(%d) to zero", _name.c_str(), _timerNumber);
  timerWrite(_timer, 0);

  log_i("[%s] Enabling alarm on Timer(%d)", _name.c_str(), _timerNumber);
  timerAlarmEnable(_timer);

  _enabled = true;
}

void SignalGenerator::stopSignal() {
  // prevent ISR from getting another packet while we are shutting down
  portENTER_CRITICAL(&_sendQueueMUX);

  log_i("[%s] Shutting down Timer(%d)", _name.c_str(), _timerNumber);
  timerStop(_timer);
  timerAlarmDisable(_timer);
  timerDetachInterrupt(_timer);
  timerEnd(_timer);
  _timer = nullptr;

  // give enough time for any timer ISR calls to complete before proceeding
  delay(250);

  // if we have a current packet being processed move it to the available
  // queue if it is not the pre-canned idle packet.
  if(_currentPacket != nullptr && _currentPacket != &_idlePacket) {
    // make sure the packet is zeroed before pushing it back to the queue
    memset(_currentPacket, 0, sizeof(Packet));
    _availablePackets.push(_currentPacket);
    _currentPacket = nullptr;
  }

  // drain any remaining packets that were not sent back into the available
  // to use packets.
  while(!isQueueEmpty()) {
    _currentPacket = _toSend.front();
    _toSend.pop();
    // make sure the packet is zeroed before pushing it back to the queue
    memset(_currentPacket, 0, sizeof(Packet));
    _availablePackets.push(_currentPacket);
  }

  _enabled = false;
  portEXIT_CRITICAL(&_sendQueueMUX);
}

void SignalGenerator::waitForQueueEmpty() {
  while(!isQueueEmpty()) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

bool SignalGenerator::isQueueEmpty() {
  return _toSend.empty();
}

bool SignalGenerator::isEnabled() {
  return _enabled;
}

bool beginProgrammingMode() {
  progTrackBusy = true;
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpStartupLimit = (4096 * 100 / motorBoard->getMaxMilliAmps());

  // energize the programming track
  motorBoard->powerOn(false);
  dccSignal[DCC_SIGNAL_PROGRAMMING].startSignal(false);
  dccSignal[DCC_SIGNAL_PROGRAMMING].waitForQueueEmpty();
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
}

void leaveProgrammingMode() {
  // deenergize the programming track
  MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG)->powerOff(false);
  dccSignal[DCC_SIGNAL_PROGRAMMING].stopSignal();

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
      signalGenerator.loadBytePacket(resetPacket, 2, 25);
      signalGenerator.waitForQueueEmpty();
    }

    // reset cvValue to all bits OFF
    cvValue = 0;
    for(uint8_t bit = 0; bit < 8; bit++) {
      log_v("[PROG] CV %d, bit [%d/7]", cv, bit);
      readCVBitPacket[2] = 0xE8 + bit;
      signalGenerator.loadBytePacket(resetPacket, 2, 3);
      signalGenerator.loadBytePacket(readCVBitPacket, 3, 5);
      signalGenerator.waitForQueueEmpty();
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
    signalGenerator.loadBytePacket(resetPacket, 2, 3);
    signalGenerator.loadBytePacket(verifyCVPacket, 3, 5);
    signalGenerator.waitForQueueEmpty();
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
      signalGenerator.loadBytePacket(resetPacket, 2, 25);
    }
    signalGenerator.loadBytePacket(resetPacket, 2, 3);
    signalGenerator.loadBytePacket(writeCVBytePacket, 3, 4);
    signalGenerator.waitForQueueEmpty();

    // verify that the decoder received the write byte packet and sent an ACK
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      signalGenerator.loadBytePacket(verifyCVBytePacket, 3, 5);
      signalGenerator.waitForQueueEmpty();
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
      signalGenerator.loadBytePacket(resetPacket, 2, 3);
    }
    signalGenerator.loadBytePacket(writeCVBitPacket, 3, 4);
    signalGenerator.waitForQueueEmpty();

    // verify that the decoder received the write byte packet and sent an ACK
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      signalGenerator.loadBytePacket(resetPacket, 2, 3);
      signalGenerator.loadBytePacket(verifyCVBitPacket, 3, 5);
      signalGenerator.waitForQueueEmpty();
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
    signalGenerator.loadBytePacket(writeCVBytePacket, 5, 4);
  } else {
    uint8_t writeCVBytePacket[] = {
      lowByte(locoAddress),
      (uint8_t)(0xEC + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      cvValue,
      0x00};
    signalGenerator.loadBytePacket(writeCVBytePacket, 4, 4);
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
    signalGenerator.loadBytePacket(writeCVBitPacket, 5, 4);
  } else {
    uint8_t writeCVBitPacket[] = {
      lowByte(locoAddress),
      (uint8_t)(0xE8 + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      (uint8_t)(0xF0 + bit + value * 8),
      0x00};
    signalGenerator.loadBytePacket(writeCVBitPacket, 4, 4);
  }
}
