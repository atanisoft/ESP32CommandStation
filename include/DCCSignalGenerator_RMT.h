/**********************************************************************
ESP32 COMMAND STATION

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
#pragma once

#include "DCCSignalGenerator.h"
#include <driver/rmt.h>
#include <dcc/Railcom.hxx>

class SignalGenerator_RMT : public SignalGenerator {
public:
  SignalGenerator_RMT(String, uint16_t, uint8_t, uint8_t,
                      uint8_t=NOT_A_PIN, uint8_t=NOT_A_PIN,
                      uint8_t=NOT_A_PIN, uint8_t=NOT_A_PIN,
                      uint8_t=NOT_A_PIN, uint8_t=NOT_A_PIN);
  SemaphoreHandle_t _stopRequest;
  SemaphoreHandle_t _stopComplete;
  const rmt_channel_t _rmtChannel;
  const uint8_t _signalPin;
  const uint8_t _outputEnablePin;
  const uint8_t _brakeEnablePin;
  const uint8_t _railComEnablePin;
  const uint8_t _railComShortPin;
  void receiveRailComData();
protected:
  void enable() override;
  void disable() override;
private:
  uart_t *_railComUART;
  std::vector <dcc::Feedback> _railComData;
};
