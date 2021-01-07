/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2021 Mike Dunston

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

#ifndef HC12_RADIO_H_
#define HC12_RADIO_H_

#include <driver/gpio.h>
#include <driver/uart.h>
#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <DCCppProtocol.h>

namespace esp32cs
{

class HC12Radio : public StateFlowBase
                , private DCCPPProtocolConsumer
{
public:
  HC12Radio(Service *, uart_port_t, gpio_num_t, gpio_num_t);
private:
  static constexpr uint8_t RX_BUF_SIZE = 64;
  StateFlowSelectHelper helper_{this};
  uint8_t rx_buffer_[RX_BUF_SIZE];
  string tx_buffer_;
  int uartFd_;
  uart_port_t uart_;
  gpio_num_t rx_;
  gpio_num_t tx_;

  STATE_FLOW_STATE(initialize);
  STATE_FLOW_STATE(data_received);
  STATE_FLOW_STATE(wait_for_data);
};

} // namespace esp32cs

#endif // HC12_RADIO_H_