/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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

#ifndef _RMT_TRACK_DEVICE_H_
#define _RMT_TRACK_DEVICE_H_

#include <driver/rmt.h>

#include <dcc/Packet.hxx>
#include <dcc/PacketFlowInterface.hxx>
#include <dcc/RailCom.hxx>
#include <dcc/RailcomHub.hxx>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos_drivers/arduino/DeviceBuffer.hxx>
#include <freertos_drivers/arduino/RailcomDriver.hxx>
#include <os/OS.hxx>
#include <utils/Atomic.hxx>
#include <utils/macros.h>
#include <utils/Singleton.hxx>
#include <utils/StringPrintf.hxx>

#include "can_ioctl.h"
#include "MonitoredHBridge.h"
#include "sdkconfig.h"

namespace esp32cs
{

class RMTTrackDevice
{
public:
  RMTTrackDevice(const char *name, const rmt_channel_t channel
               , const uint8_t dccPreambleBitCount, size_t packet_queue_len
               , gpio_num_t pin, RailcomDriver *railcomDriver);

  ~RMTTrackDevice();

  // VFS interface helper
  ssize_t write(int, const void *, size_t);

  // VFS interface helper
  int ioctl(int, int, va_list);

  // RMT callback for transmit completion. This will be called via the ISR
  // context but not from an IRAM restricted context.
  void rmt_transmit_complete();

  const char *name() const
  {
    return name_;
  }

private:
  // maximum number of RMT memory blocks (256 bytes each, 4 bytes per data bit)
  // this will result in a max payload of 192 bits which is larger than any
  // known DCC packet with the addition of up to 50 preamble bits.
  static constexpr uint8_t MAX_RMT_MEMORY_BLOCKS = 3;

  // maximum number of bits that can be transmitted as one packet.
  static constexpr uint8_t MAX_RMT_BITS = (RMT_MEM_ITEM_NUM * MAX_RMT_MEMORY_BLOCKS);

  const char *name_;
  const rmt_channel_t channel_;
  const uint8_t dccPreambleBitCount_;
  RailcomDriver *railcomDriver_;
  StaticQueue_t packetQueue_;
  QueueHandle_t packetQueueHandle_;
  void *packetQueueBuf_;
  Notifiable* notifiable_{nullptr};
  int8_t pktRepeatCount_{0};
  uint32_t pktLength_{0};
  rmt_item32_t packet_[MAX_RMT_BITS];

  void encode_next_packet(BaseType_t *woken);

  DISALLOW_COPY_AND_ASSIGN(RMTTrackDevice);
};

} // namespace esp32cs

#endif // _RMT_TRACK_DEVICE_H_
