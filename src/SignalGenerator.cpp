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

// number of microseconds for sending a zero via the DCC encoding
#define DCC_ZERO_BIT_TOTAL_DURATION 196
// number of microseconds for each half of the DCC signal for a zero
#define DCC_ZERO_BIT_PULSE_DURATION 98

// number of microseconds for sending a one via the DCC encoding
#define DCC_ONE_BIT_TOTAL_DURATION 116
// number of microseconds for each half of the DCC signal for a one
#define DCC_ONE_BIT_PULSE_DURATION 58

// number of samples to take when monitoring current after a CV verify
// (bit or byte) has been sent
const uint8_t CVSampleCount = 250;

SignalGenerator dccSignal[MAX_DCC_SIGNAL_GENERATORS];

// S-9.2 baseline packet (idle)
DRAM_ATTR uint8_t idlePacket[] = {0xFF, 0x00};
// S-9.2 baseline packet (decoder reset)
DRAM_ATTR uint8_t resetPacket[] = {0x00, 0x00};
// S-9.2 baseline packet (eStop, direction bit ignored)
DRAM_ATTR uint8_t eStopPacket[] = {0x00, 0x41};

void loadBytePacket(SignalGenerator &, uint8_t *, uint8_t, uint8_t, bool=false);

void configureDCCSignalGenerators() {
  dccSignal[DCC_SIGNAL_OPERATIONS].configureSignal<DCC_SIGNAL_OPERATIONS>("OPS",
    DCC_SIGNAL_PIN_OPERATIONS, 512);
  dccSignal[DCC_SIGNAL_PROGRAMMING].configureSignal<DCC_SIGNAL_PROGRAMMING>("PROG",
    DCC_SIGNAL_PIN_PROGRAMMING, 64);
}

void startDCCSignalGenerators() {
  dccSignal[DCC_SIGNAL_OPERATIONS].startSignal<DCC_SIGNAL_OPERATIONS>();
  dccSignal[DCC_SIGNAL_PROGRAMMING].startSignal<DCC_SIGNAL_PROGRAMMING>();
}

void stopDCCSignalGenerators() {
  dccSignal[DCC_SIGNAL_OPERATIONS].stopSignal<DCC_SIGNAL_OPERATIONS>();
  dccSignal[DCC_SIGNAL_PROGRAMMING].stopSignal<DCC_SIGNAL_PROGRAMMING>();
}

bool isDCCSignalEnabled() {
  if(dccSignal[DCC_SIGNAL_OPERATIONS].isEnabled() || dccSignal[DCC_SIGNAL_PROGRAMMING].isEnabled()) {
    return true;
  }
  return false;
}

void sendDCCEmergencyStop() {
  loadBytePacket(dccSignal[DCC_SIGNAL_OPERATIONS], eStopPacket, 2, 0, true);
  loadBytePacket(dccSignal[DCC_SIGNAL_PROGRAMMING], eStopPacket, 2, 0, true);
}

void loadBytePacket(SignalGenerator &signalGenerator, uint8_t *data, uint8_t length, uint8_t repeatCount, bool drainToSendQueue) {
  std::vector<uint8_t> packet;
  for(int i = 0; i < length; i++) {
    packet.push_back(data[i]);
  }
  signalGenerator.loadPacket(packet, repeatCount);
}

bool IRAM_ATTR SignalGenerator::getNextBitToSend() {
  const uint8_t bitMask[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
  bool result = false;
  // if we are processing a packet, check if we have sent all bits or repeats
  if(_currentPacket != nullptr) {
    if(_currentPacket->currentBit == _currentPacket->numberOfBits) {
      if(_currentPacket->numberOfRepeats > 0) {
        _currentPacket->numberOfRepeats--;
        _currentPacket->currentBit = 0;
      } else {
        // if the current packet is not the idle pack get rid of it
        if(_currentPacket != &_idlePacket) {
          _availablePackets.push(_currentPacket);
        }
        _currentPacket = nullptr;
      }
    }
  }
  // if we don't have a packet, check if we have any to send otherwise
  // queue up an idle packet
  if (_currentPacket == nullptr) {
    portENTER_CRITICAL_ISR(&_sendQueueMUX);
    if(!isQueueEmpty()) {
      _currentPacket = _toSend.front();
      _toSend.pop();
    } else {
      _currentPacket = &_idlePacket;
      _currentPacket->currentBit = 0;
    }
    portEXIT_CRITICAL_ISR(&_sendQueueMUX);
  }
  // if we have a packet to send, get the next bit from the packet
  if(_currentPacket != nullptr) {
    result = _currentPacket->buffer[_currentPacket->currentBit / 8] & bitMask[_currentPacket->currentBit % 8];
    _currentPacket->currentBit++;
  }
  return result;
}

void SignalGenerator::loadPacket(std::vector<uint8_t> data, int numberOfRepeats, bool drainToSendQueue) {
#if DEBUG_SIGNAL_GENERATOR
  log_v("[%s] Preparing DCC Packet containing %d bytes, %d repeats [%d in queue]", _name.c_str(), data.size(), numberOfRepeats, _toSend.size());
#endif
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
  for(int i = 1; i < data.size(); i++)
    checksum ^= data[i];
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

#if SHOW_DCC_PACKETS
  String packetHex = "";
  for(int i = 0; i < data.size() + 1; i++) {
    packetHex += String(packet->buffer[i], HEX) + " ";
  }
  log_v("[%s] <* %s / %d / %d>\n", _name.c_str(), packetHex.c_str(),
    packet->numberOfBits, packet->numberOfRepeats);
#endif
  portENTER_CRITICAL(&_sendQueueMUX);
  _toSend.push(packet);
  portEXIT_CRITICAL(&_sendQueueMUX);
}

template<int signalGenerator>
void IRAM_ATTR signalGeneratorPulseTimer(void)
{
  auto& generator = dccSignal[signalGenerator];
  if(generator.getNextBitToSend()) {
    timerAlarmWrite(generator._pulseTimer, DCC_ONE_BIT_PULSE_DURATION, false);
    timerAlarmWrite(generator._fullCycleTimer, DCC_ONE_BIT_TOTAL_DURATION, true);
  } else {
    timerAlarmWrite(generator._pulseTimer, DCC_ZERO_BIT_PULSE_DURATION, false);
    timerAlarmWrite(generator._fullCycleTimer, DCC_ZERO_BIT_TOTAL_DURATION, true);
  }
  timerWrite(generator._pulseTimer, 0);
  timerAlarmEnable(generator._pulseTimer);
  digitalWrite(generator._directionPin, HIGH);
}

template<int signalGenerator>
void IRAM_ATTR signalGeneratorDirectionTimer()
{
  auto& generator = dccSignal[signalGenerator];
  digitalWrite(generator._directionPin, LOW);
}

template<int signalGenerator>
void SignalGenerator::configureSignal(String name, uint8_t directionPin, uint16_t maxPackets) {
  _name = name;
  _directionPin = directionPin;
  _currentPacket = nullptr;

  // create packets for this signal generator up front, they will be reused until
  // the base station is shutdown
  for(int index = 0; index < maxPackets; index++) {
    _availablePackets.push(new Packet());
  }

  // force the directionPin to low since it will be controlled by the DCC timer
  pinMode(_directionPin, INPUT);
  digitalWrite(_directionPin, LOW);
  pinMode(_directionPin, OUTPUT);
  startSignal<signalGenerator>();
}

template<int signalGenerator>
void SignalGenerator::startSignal() {
  // inject the required reset and idle packets into the queue
  // this is required as part of S-9.2.4 section A
  // at least 20 reset packets and 10 idle packets must be sent upon initialization
  // of the base station to force decoders to exit service mode.
  log_i("[%s] Adding reset packet to packet queue", _name.c_str());
  loadBytePacket(dccSignal[signalGenerator], resetPacket, 2, 20);
  log_i("[%s] Adding idle packet to packet queue", _name.c_str());
  loadBytePacket(dccSignal[signalGenerator], idlePacket, 2, 10);

  log_i("[%s] Configuring Timer(%d) for generating DCC Signal (Full Wave)", _name.c_str(), 2 * signalGenerator);
  _fullCycleTimer = timerBegin(2 * signalGenerator, DCC_TIMER_PRESCALE, true);
  log_i("[%s] Attaching interrupt handler to Timer(%d)", _name.c_str(), 2 * signalGenerator);
  timerAttachInterrupt(_fullCycleTimer, &signalGeneratorPulseTimer<signalGenerator>, true);
  log_i("[%s] Configuring alarm on Timer(%d) to %dus", _name.c_str(), 2 * signalGenerator, DCC_ONE_BIT_TOTAL_DURATION);
  timerAlarmWrite(_fullCycleTimer, DCC_ONE_BIT_TOTAL_DURATION, true);
  log_i("[%s] Setting load on Timer(%d) to zero", _name.c_str(), 2 * signalGenerator);
  timerWrite(_fullCycleTimer, 0);

  log_i("[%s] Configuring Timer(%d) for generating DCC Signal (Half Wave)", _name.c_str(), 2 * signalGenerator + 1);
  _pulseTimer = timerBegin(2 * signalGenerator + 1, DCC_TIMER_PRESCALE, true);
  log_i("[%s] Attaching interrupt handler to Timer(%d)", _name.c_str(), 2 * signalGenerator + 1);
  timerAttachInterrupt(_pulseTimer, &signalGeneratorDirectionTimer<signalGenerator>, true);
  log_i("[%s] Configuring alarm on Timer(%d) to %dus", _name.c_str(), 2 * signalGenerator + 1, DCC_ONE_BIT_TOTAL_DURATION / 2);
  timerAlarmWrite(_pulseTimer, DCC_ONE_BIT_PULSE_DURATION, false);
  log_i("[%s] Setting load on Timer(%d) to zero", _name.c_str(), 2 * signalGenerator + 1);
  timerWrite(_pulseTimer, 0);

  log_i("[%s] Enabling alarm on Timer(%d)", _name.c_str(), 2 * signalGenerator);
  timerAlarmEnable(_fullCycleTimer);
  log_i("[%s] Enabling alarm on Timer(%d)", _name.c_str(), 2 * signalGenerator + 1);
  timerAlarmEnable(_pulseTimer);
  
  _enabled = true;
}

template<int signalGenerator>
void SignalGenerator::stopSignal() {
  // prevent ISR from getting another packet while we are shutting down
  portENTER_CRITICAL(&_sendQueueMUX);

  log_i("[%s] Shutting down Timer(%d) (Full Wave)", _name.c_str(), 2 * signalGenerator);
  timerStop(_fullCycleTimer);
  timerAlarmDisable(_fullCycleTimer);
  timerDetachInterrupt(_fullCycleTimer);
  timerEnd(_fullCycleTimer);

  log_i("[%s] Shutting down Timer(%d) (Half Wave)", _name.c_str(), 2 * signalGenerator + 1);
  timerStop(_pulseTimer);
  timerAlarmDisable(_pulseTimer);
  timerDetachInterrupt(_pulseTimer);
  timerEnd(_pulseTimer);

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
    log_v("[%s] Waiting for %d packets to send...", _name.c_str(), _toSend.size());
    delay(10);
  }
}

bool SignalGenerator::isQueueEmpty() {
  return _toSend.empty();
}

bool SignalGenerator::isEnabled() {
  return _enabled;
}

int16_t readCV(const uint16_t cv, uint8_t attempts) {
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpAck = (4096 * 60 / motorBoard->getMaxMilliAmps());
  uint8_t readCVBitPacket[4] = { (uint8_t)(0x78 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), 0x00, 0x00};
  uint8_t verifyCVBitPacket[4] = { (uint8_t)(0x74 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), 0x00, 0x00};
  int16_t cvValue = -1;
  for(int attempt = 0; attempt < attempts && cvValue == -1; attempt++) {
    log_i("[PROG %d/%d] Attempting to read CV %d", attempt+1, attempts, cv);
    auto& signalGenerator = dccSignal[DCC_SIGNAL_PROGRAMMING];

    for(uint8_t bit = 0; bit < 8; bit++) {
      log_v("[PROG] CV %d, bit [%d/7]", cv, bit);
      readCVBitPacket[2] = 0xE8 + bit;
      loadBytePacket(signalGenerator, resetPacket, 2, 3);
      loadBytePacket(signalGenerator, readCVBitPacket, 3, 5);
      signalGenerator.waitForQueueEmpty();
      if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
        log_v("[PROG] CV %d, bit [%d/7] ON", cv, bit);
        bitWrite(cvValue, bit, 1);
      } else {
        log_v("[PROG] CV %d, bit [%d/7] OFF", cv, bit);
      }
    }

    // verify the byte we received
    verifyCVBitPacket[2] = cvValue & 0xFF;
    log_d("[PROG %d/%d] Attempting to verify read of CV %d as %d", attempt+1, attempts, cv, cvValue);
    loadBytePacket(signalGenerator, resetPacket, 2, 3);
    loadBytePacket(signalGenerator, verifyCVBitPacket, 3, 5);
    signalGenerator.waitForQueueEmpty();
    bool verified = false;
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      verified = true;
      log_i("[PROG] CV %d, verified as %d", cv, cvValue);
    }
    if(!verified) {
      log_w("[PROG] CV %d, could not be verified", cv);
      cvValue = -1;
    }
  }
  log_d("[PROG] CV %d value is %d", cv, cvValue);
  return cvValue;
}

bool writeProgCVByte(const uint16_t cv, const uint8_t cvValue) {
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpAck = (4096 * 60 / motorBoard->getMaxMilliAmps());
  const uint8_t maxWriteAttempts = 5;
  uint8_t writeCVBytePacket[4] = { (uint8_t)(0x7C + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), cvValue, 0x00};
  uint8_t verifyCVBytePacket[4] = { (uint8_t)(0x74 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), cvValue, 0x00};
  bool writeVerified = false;
  auto& signalGenerator = dccSignal[DCC_SIGNAL_PROGRAMMING];

  for(uint8_t attempt = 1; attempt <= maxWriteAttempts && !writeVerified; attempt++) {
    log_d("[PROG %d/%d] Attempting to write CV %d as %d", attempt, maxWriteAttempts, cv, cvValue);
    loadBytePacket(signalGenerator, resetPacket, 2, 1);
    loadBytePacket(signalGenerator, writeCVBytePacket, 3, 4);
    signalGenerator.waitForQueueEmpty();
    // verify that the decoder received the write byte packet and sent an ACK
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      loadBytePacket(signalGenerator, resetPacket, 2, 3);
      loadBytePacket(signalGenerator, verifyCVBytePacket, 3, 5);
      signalGenerator.waitForQueueEmpty();
      // check that decoder sends an ACK for the verify operation
      if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
        writeVerified = true;
        log_d("[PROG] CV %d write value %d verified.", cv, cvValue);
      }
    } else {
      log_w("[PROG] CV %d write value %d could not be verified.", cv, cvValue);
    }
    log_i("[PROG] Sending decoder reset packet");
    loadBytePacket(signalGenerator, resetPacket, 2, 3);
  }
  return writeVerified;
}

bool writeProgCVBit(const uint16_t cv, const uint8_t bit, const bool value) {
  const auto motorBoard = MotorBoardManager::getBoardByName(MOTORBOARD_NAME_PROG);
  const uint16_t milliAmpAck = (4096 * 60 / motorBoard->getMaxMilliAmps());
  const uint8_t maxWriteAttempts = 5;
  uint8_t writeCVBitPacket[4] = { (uint8_t)(0x78 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), (uint8_t)(0xF0 + bit + value * 8), 0x00};
  uint8_t verifyCVBitPacket[4] = { (uint8_t)(0x74 + (highByte(cv - 1) & 0x03)), lowByte(cv - 1), (uint8_t)(0xB0 + bit + value * 8), 0x00};
  bool writeVerified = false;
  auto& signalGenerator = dccSignal[DCC_SIGNAL_PROGRAMMING];

  for(uint8_t attempt = 1; attempt <= maxWriteAttempts && !writeVerified; attempt++) {
    log_d("[PROG %d/%d] Attempting to write CV %d bit %d as %d", attempt, maxWriteAttempts, cv, bit, value);
    loadBytePacket(signalGenerator, resetPacket, 2, 1);
    loadBytePacket(signalGenerator, writeCVBitPacket, 3, 4);
    signalGenerator.waitForQueueEmpty();
    // verify that the decoder received the write byte packet and sent an ACK
    if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
      loadBytePacket(signalGenerator, resetPacket, 2, 3);
      loadBytePacket(signalGenerator, verifyCVBitPacket, 3, 5);
      signalGenerator.waitForQueueEmpty();
      // check that decoder sends an ACK for the verify operation
      if(motorBoard->captureSample(CVSampleCount) > milliAmpAck) {
        writeVerified = true;
        log_d("[PROG %d/%d] CV %d write bit %d verified.", attempt, maxWriteAttempts, cv, bit);
      }
    } else {
      log_w("[PROG %d/%d] CV %d write bit %d could not be verified.", attempt, maxWriteAttempts, cv, bit);
    }
    log_i("[PROG] Sending decoder reset packet");
    loadBytePacket(signalGenerator, resetPacket, 2, 3);
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
    loadBytePacket(signalGenerator, writeCVBytePacket, 5, 4);
  } else {
    uint8_t writeCVBytePacket[] = {
      lowByte(locoAddress),
      (uint8_t)(0xEC + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      cvValue,
      0x00};
    loadBytePacket(signalGenerator, writeCVBytePacket, 4, 4);
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
    loadBytePacket(signalGenerator, writeCVBitPacket, 5, 4);
  } else {
    uint8_t writeCVBitPacket[] = {
      lowByte(locoAddress),
      (uint8_t)(0xE8 + (highByte(cv - 1) & 0x03)),
      lowByte(cv - 1),
      (uint8_t)(0xF0 + bit + value * 8),
      0x00};
    loadBytePacket(signalGenerator, writeCVBitPacket, 4, 4);
  }
}
