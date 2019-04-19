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
#include <esp32-hal-timer.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

// number of microseconds for each half of the DCC signal for a zero
static constexpr uint64_t DCC_ZERO_BIT_PULSE_DURATION=98;
// number of microseconds for each half of the DCC signal for a one
static constexpr uint64_t DCC_ONE_BIT_PULSE_DURATION=58;
// this controls the timer tick frequency, this results in a 1uS tick frequency.
static constexpr uint16_t DCC_TIMER_PRESCALE=80;

#define DCC_SIGNAL_ISR_IMPL(G, S) \
  if(G->_topOfWave) { \
    auto pkt = G->getPacket(); \
    digitalWrite(S, HIGH); \
    if(pkt->buffer[pkt->currentBit / 8] & DCC_PACKET_BIT_MASK[pkt->currentBit % 8]) { \
      timerAlarmWrite(G->_timer, DCC_ONE_BIT_PULSE_DURATION, false); \
    } else { \
      timerAlarmWrite(G->_timer, DCC_ZERO_BIT_PULSE_DURATION, false); \
    } \
    pkt->currentBit++; \
  } else { \
    digitalWrite(S, LOW); \
  } \
  G->_topOfWave = !G->_topOfWave; \
  timerWrite(G->_timer, 0); \
  timerAlarmEnable(G->_timer);

void IRAM_ATTR signalGeneratorTimerISR_OPS(void)
{
  DCC_SIGNAL_ISR_IMPL(reinterpret_cast<SignalGenerator_HardwareTimer *>(dccSignal[DCC_SIGNAL_OPERATIONS]), DCC_SIGNAL_PIN_OPERATIONS)
}

void IRAM_ATTR signalGeneratorTimerISR_PROG(void)
{
  DCC_SIGNAL_ISR_IMPL(reinterpret_cast<SignalGenerator_HardwareTimer *>(dccSignal[DCC_SIGNAL_PROGRAMMING]), DCC_SIGNAL_PIN_PROGRAMMING)
}

SignalGenerator_HardwareTimer::SignalGenerator_HardwareTimer(String name, uint16_t maxPackets, uint8_t signalID, uint8_t signalPin) :
    SignalGenerator(name, maxPackets, signalID, signalPin) {
}

void SignalGenerator_HardwareTimer::enable() {
  LOG(INFO, "[%s] Configuring Timer(%d) for generating DCC Signal", _name.c_str(), _signalID + 1);
  _timer = timerBegin(_signalID + 1, DCC_TIMER_PRESCALE, true);
  LOG(INFO, "[%s] Attaching interrupt handler to Timer(%d)", _name.c_str(), _signalID + 1);
  if(_signalID == DCC_SIGNAL_OPERATIONS) {
    timerAttachInterrupt(_timer, &signalGeneratorTimerISR_OPS, true);
  } else {
    timerAttachInterrupt(_timer, &signalGeneratorTimerISR_PROG, true);
  }
  LOG(INFO, "[%s] Configuring alarm on Timer(%d) to %" PRIu64 "us", _name.c_str(), _signalID + 1, DCC_ONE_BIT_PULSE_DURATION);
  timerAlarmWrite(_timer, DCC_ONE_BIT_PULSE_DURATION, true);
  LOG(INFO, "[%s] Setting load on Timer(%d) to zero", _name.c_str(), _signalID + 1);
  timerWrite(_timer, 0);

  LOG(INFO, "[%s] Enabling alarm on Timer(%d)", _name.c_str(), _signalID + 1);
  timerAlarmEnable(_timer);
}

void SignalGenerator_HardwareTimer::disable() {
  LOG(INFO, "[%s] Shutting down Timer(%d)", _name.c_str(), _signalID + 1);
  timerStop(_timer);
  timerAlarmDisable(_timer);
  timerDetachInterrupt(_timer);
  timerEnd(_timer);
  _timer = nullptr;

  // give enough time for any timer ISR calls to complete before proceeding
  delay(250);
}

void SignalGenerator_HardwareTimer::lockSendQueueISR() {
  portENTER_CRITICAL_ISR(&_sendQueueMUX);
}

void SignalGenerator_HardwareTimer::unlockSendQueueISR() {
  portEXIT_CRITICAL_ISR(&_sendQueueMUX);
}
