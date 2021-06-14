/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2021 Mike Dunston

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

#include "sdkconfig.h"
#include <can_ioctl.h>
#include <dcc/DccDebug.hxx>
#include <dcc/Packet.hxx>
#include <driver/rmt.h>
#include <executor/Notifiable.hxx>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos_drivers/arduino/RailcomDriver.hxx>
#include <soc/soc_caps.h>
#include <utils/logging.h>
#include <utils/macros.h>

namespace esp32cs
{
#if CONFIG_DCC_RMT_HIGH_FIRST
// This configures the first half of the bit to be sent as a high and second
// half as low.
static constexpr const char * const RMT_TRACK_DEVICE_DCC_WAVE_FMT =
  "high,low";
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_TOP_HALF = 1;
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_BOTTOM_HALF = 0;
#else
// This configures the first half of the bit to be sent as a low and second
// half as high.
static constexpr const char * const RMT_TRACK_DEVICE_DCC_WAVE_FMT =
  "low,high";
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_TOP_HALF = 0;
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_BOTTOM_HALF = 1;
#endif // CONFIG_DCC_RMT_HIGH_FIRST

///////////////////////////////////////////////////////////////////////////////
// DCC ZERO bit pre-encoded in RMT format.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t DCC_RMT_ZERO_BIT =
{{{
    CONFIG_DCC_RMT_TICKS_ZERO_PULSE, // number of microseconds for TOP half
    RMT_TRACK_DEVICE_DCC_TOP_HALF,   // of the square wave.
    CONFIG_DCC_RMT_TICKS_ZERO_PULSE, // number of microseconds for BOTTOM half
    RMT_TRACK_DEVICE_DCC_BOTTOM_HALF // of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// DCC ONE bit pre-encoded in RMT format.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t DCC_RMT_ONE_BIT =
{{{
    CONFIG_DCC_RMT_TICKS_ONE_PULSE,  // number of microseconds for TOP half
    RMT_TRACK_DEVICE_DCC_TOP_HALF,   // of the square wave.
    CONFIG_DCC_RMT_TICKS_ONE_PULSE,  // number of microseconds for BOTTOM half
    RMT_TRACK_DEVICE_DCC_BOTTOM_HALF // of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// The NMRA DCC Signal is sent as a square wave with each half having
// identical timing (or nearly identical). Packet Bytes have a minimum of 11
// preamble ONE bits in order to be considered valid by the decoder. For
// RailCom support it is recommended to have at least 16 preamble bits. For the
// Programming Track it is required to have a longer preamble of at least 22
// bits. Packet data follows immediately after the preamble bits, between the
// packet bytes a DCC ZERO is sent. After the last byte of packet data a DCC
// ONE is sent.
//
// DCC ZERO:
//    ----------------
//    |     100      |
// ---|     usec     |     100      ---
//                   |     usec     |
//                   ----------------
// DCC ONE:
//    --------
//    |  58  |      
// ---| usec |  58  ---
//           | usec |
//           --------
//
// NMRA S-9.1 reference:
// https://www.nmra.org/sites/default/files/standards/sandrp/pdf/s-9.1_electrical_standards_2020.pdf
//
///////////////////////////////////////////////////////////////////////////////
template<class HW, class DCC_BOOSTER>
class RMTTrackDevice
{
public:
  RMTTrackDevice(RailcomDriver *railcomDriver) : railcomDriver_(railcomDriver)
  {
  }
  /// Hardware initialization routine
  void hw_init()
  {
    // allocate buffer space for the packet queue items.
    packetQueueBuf_ =
      heap_caps_calloc(HW::PACKET_Q_SIZE, sizeof(dcc::Packet),
                       RMT_MALLOC_CAPS);
    HASSERT(packetQueueBuf_ != nullptr);

    // create the packet queue using the pre-allocated memory.
    packetQueueHandle_ =
      xQueueCreateStatic(HW::PACKET_Q_SIZE, sizeof(dcc::Packet)
                      , (uint8_t*)packetQueueBuf_, &packetQueue_);
    HASSERT(packetQueueHandle_ != NULL);

    LOG(INFO,
        "[RMT] DCC config: zero:%duS, one:%duS, preamble-bits:%d/%d, wave:%s",
        CONFIG_DCC_RMT_TICKS_ZERO_PULSE, CONFIG_DCC_RMT_TICKS_ONE_PULSE,
        HW::DCC_PREAMBLE_BITS, HW::DCC_SERVICE_MODE_PREAMBLE_BITS,
        RMT_TRACK_DEVICE_DCC_WAVE_FMT);
#if 0
    LOG(INFO,
        "[RMT] Marklin-Motorola config: zero: %duS (high), zero: %duS (low), "
        "one: %duS (high), one: %duS (low), preamble: %duS (low)",
        MARKLIN_ZERO_BIT_PULSE_HIGH_USEC, MARKLIN_ZERO_BIT_PULSE_LOW_USEC,
        MARKLIN_ONE_BIT_PULSE_HIGH_USEC, MARKLIN_ONE_BIT_PULSE_LOW_USEC,
        MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC + MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC);
#endif
    LOG(INFO, "[RMT] signal pin: %d, RMT-%d", HW::DCC_SIGNAL_PIN_NUM,
        HW::RMT_CHANNEL);

    // Configure the RMT channel for TX
    rmt_config_t config =
      RMT_DEFAULT_CONFIG_TX(HW::DCC_SIGNAL_PIN_NUM, HW::RMT_CHANNEL);
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(
      rmt_driver_install(HW::RMT_CHANNEL, 0 /* rx count */, RMT_ISR_FLAGS));

    LOG(INFO, "[RMT] Starting signal generator");
    // send one bit to kickstart the signal, remaining data will come from the
    // packet queue. We intentionally do not wait for the RMT TX complete here.
    rmt_write_items(HW::RMT_CHANNEL, &DCC_RMT_ONE_BIT, 1, false);
  }

  ~RMTTrackDevice()
  {
    LOG(INFO, "[RMT] Shutting down signal generator");
    rmt_driver_uninstall(HW::RMT_CHANNEL);
    if (packetQueueHandle_)
    {
      vQueueDelete(packetQueueHandle_);
    }
    if (packetQueueBuf_)
    {
      free(packetQueueBuf_);
    }
  }

  /// VFS interface helper
  ssize_t write(int fd, const void * data, size_t size)
  {
    if (size != sizeof(dcc::Packet))
    {
      errno = EINVAL;
      return -1;
    }
    const dcc::Packet *sourcePacket{(dcc::Packet *)data};

    if (sourcePacket->packet_header.is_marklin)
    {
      // drop marklin packets as unsupported for now.
      errno = ENOTSUP;
      return -1;
    }
#if CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY
    if (sourcePacket->packet_header.send_long_preamble)
    {
      // If the packet looks like a programming track packet, drop it since as
      // only short preamble packets will be accepted for TX.
      return 1;
    }
#elif CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY
    // force long preamble for all packets.
    sourcePacket->packet_header.send_long_preamble = 1;
#endif // CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY
    if (xQueueSendToBack(packetQueueHandle_, sourcePacket
                      , MAX_PACKET_QUEUE_WAIT_TIME) == pdTRUE)
    {
      return 1;
    }

    // packet queue is full!
    errno = ENOSPC;
    return -1;
  }

  /// VFS interface helper
  int ioctl(int fd, int cmd, va_list args)
  {
    // Attempt to write a Packet to the queue
    if (IOC_TYPE(cmd) == CAN_IOC_MAGIC && IOC_SIZE(cmd) == NOTIFIABLE_TYPE &&
        cmd == CAN_IOC_WRITE_ACTIVE)
    {
      Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
      HASSERT(n);
      // if there is no space available in the queue, stash the notifiable
      // handle so we can wake it up later.
      if (!uxQueueSpacesAvailable(packetQueueHandle_))
      {
        std::swap(n, notifiable_);
      }
      if (n)
      {
        n->notify();
      }
      return 0;
    }

    // Unknown ioctl operation
    errno = EINVAL;
    return -1;
  }

  /// RMT callback for transmit completion. This will be called via the ISR
  /// context but not from an IRAM restricted context.
  void rmt_transmit_complete()
  {
    BaseType_t woken = pdFALSE;
    encode_next_packet(&woken);
    if (DCC_BOOSTER::need_railcom_cutout())
    {
      railcomDriver_->start_cutout();
    }
    else
    {
      railcomDriver_->no_cutout();
      if (DCC_BOOSTER::should_be_enabled())
      {
        DCC_BOOSTER::enable_output();
      }
    }

    // NOTE: This is not using rmt_write_items as it is not safe within an ISR
    // context which this callback is invoked from.
    rmt_fill_tx_items(HW::RMT_CHANNEL, packet_, pktLength_, 0);

    // start the transmit using the rmt_tx_start method which is ISR safe as of
    // IDF v4.1.
    rmt_tx_start(HW::RMT_CHANNEL, true);

    // if we need to wake up another task we can safely do it after sending the
    // packet off to be transmitted.
    if (woken == pdTRUE)
    {
      portYIELD_FROM_ISR();
    }
  }

private:
  // maximum number of RMT memory blocks (256 bytes each, 4 bytes per data bit)
  // this will result in a max payload of 192 bits which is larger than any
  // known DCC packet with the addition of up to 50 preamble bits.
  static constexpr uint8_t MAX_RMT_MEMORY_BLOCKS = 3;

  /// malloc() capabilities to use for the DCC packet queue. This is configured
  /// to use internal 8-bit capable memory only.
  static constexpr uint32_t RMT_MALLOC_CAPS = MALLOC_CAP_INTERNAL |
                                              MALLOC_CAP_8BIT;

  ///////////////////////////////////////////////////////////////////////////////
  // Declare ISR flags for the RMT driver ISR.
  //
  // NOTE: ESP_INTR_FLAG_IRAM is *NOT* included in this bitmask so that we do not
  // have a dependency on execution from IRAM and the related software
  // limitations of execution from there.
  ///////////////////////////////////////////////////////////////////////////////
  static constexpr uint32_t RMT_ISR_FLAGS =
  (
      ESP_INTR_FLAG_LOWMED |            // ISR is implemented in C code
      ESP_INTR_FLAG_SHARED              // ISR is shared across multiple handlers
  );

  ///////////////////////////////////////////////////////////////////////////////
  // Maximum amount of time to wait for the packet queue to have space available
  // for another packet before giving up. Currently configured to not wait.
  ///////////////////////////////////////////////////////////////////////////////
  static constexpr BaseType_t MAX_PACKET_QUEUE_WAIT_TIME = 0;
  RailcomDriver *railcomDriver_;
  StaticQueue_t packetQueue_;
  QueueHandle_t packetQueueHandle_{nullptr};
  void *packetQueueBuf_{nullptr};
  Notifiable* notifiable_{nullptr};
  int8_t pktRepeatCount_{0};
  uint32_t pktLength_{0};
#ifdef SOC_RMT_CHANNEL_MEM_WORDS
  rmt_item32_t packet_[SOC_RMT_CHANNEL_MEM_WORDS * MAX_RMT_MEMORY_BLOCKS];
#elif defined(SOC_RMT_MEM_WORDS_PER_CHANNEL)
  rmt_item32_t packet_[SOC_RMT_MEM_WORDS_PER_CHANNEL * MAX_RMT_MEMORY_BLOCKS];
#else
#error Unable to determine RMT bits per channel
#endif

#if CONFIG_DCC_RMT_EMC_SPREAD
  /// Maximum number of microseconds that can be added to each bit pulse time.
  ///
  /// S-9.1 provides a maximum length of 105 and 61 microseconds for each half
  /// wave
  /// 
  /// NOTE: the values below are one higher than maximum to allow for modulo
  /// operation.
  static constexpr uint8_t DCC_RMT_MAX_ZERO_BIT_SPREAD =
    105 - CONFIG_DCC_RMT_TICKS_ZERO_PULSE + 1;
  static constexpr uint8_t DCC_RMT_MAX_ONE_BIT_SPREAD =
    61 - CONFIG_DCC_RMT_TICKS_ONE_PULSE + 1;
#endif // CONFIG_DCC_RMT_EMC_SPREAD

  void encode_next_packet(BaseType_t *woken)
  {
    // Bit mask constants used as part of the packet translation layer.
    const uint8_t PACKET_BIT_MASK[] =
    {
      0x80, 0x40, 0x20, 0x10, //
      0x08, 0x04, 0x02, 0x01  //
    };

    // Check if we need to encode the next packet or if we still have at least
    // one repeat left of the current packet.
    if (--pktRepeatCount_ >= 0)
    {
      return;
    }
    // attempt to fetch a packet from the queue or use an idle packet
    Notifiable* n = nullptr;
    dcc::Packet packet{dcc::Packet::DCC_IDLE()};
    
    if (xQueueReceiveFromISR(packetQueueHandle_, &packet, woken))
    {
      // since we removed a packet from the queue, swap the notifiable handle
      // so it can be woken up if needed.
      std::swap(n, notifiable_);
    }
    if (n)
    {
      n->notify_from_isr();
    }
    if (packet.packet_header.is_marklin)
    {
      packet = dcc::Packet::DCC_IDLE();
    }
#if CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY
    uint32_t preableBitCount = HW::DCC_SERVICE_MODE_PREAMBLE_BITS;
#else
    uint32_t preableBitCount = HW::DCC_PREAMBLE_BITS;
#ifndef CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY
    if (packet.packet_header.send_long_preamble)
    {
      preableBitCount = HW::DCC_SERVICE_MODE_PREAMBLE_BITS;
    }
#endif // CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY
#endif // CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY
    // encode the preamble bits
    for (pktLength_ = 0; pktLength_ < preableBitCount; pktLength_++)
    {
      packet_[pktLength_].val = DCC_RMT_ONE_BIT.val;
    }
    // start of payload marker
    packet_[pktLength_++].val = DCC_RMT_ZERO_BIT.val;
    // encode the packet bits
    for (uint8_t dlc = 0; dlc < packet.dlc; dlc++)
    {
      for(uint8_t bit = 0; bit < 8; bit++, pktLength_++)
      {
        packet_[pktLength_].val = PACKET_BIT_MASK[bit] ?
          DCC_RMT_ONE_BIT.val : DCC_RMT_ZERO_BIT.val;
      }
      // end of byte marker
      packet_[pktLength_++].val = DCC_RMT_ZERO_BIT.val;
    }
    // set the last bit of the encoded payload to be an end of packet marker
    packet_[pktLength_ - 1].val = DCC_RMT_ONE_BIT.val;
    // add an extra ONE bit to the end to prevent mangling of the last bit by
    // the RMT
    packet_[pktLength_++].val = DCC_RMT_ONE_BIT.val;
    // Add marker to the end of the DCC packet data to allow the RMT to know it
    // can stop transmitting at this point.
    packet_[pktLength_++].val = 0;

#if CONFIG_DCC_RMT_EMC_SPREAD
    // If the EMC spectrum spreading option is enabled, modify the DCC bit time
    // to ensure no two sequential bits are identical. The modification of the
    // bits is done in the simplest manner possible by adding a few
    // microseconds to the two parts of the signal based on the packet bit
    // index modulo the maximum bit spread (5usec for zero, 3usec for one).
    // The first bit of the preamble is skipped as is the last entry in the
    // packet which is an end-of-packet marker for the RMT peripheral and is
    // not transmitted to the rails.
    for (uint8_t idx = 1 idx < pktLength_; idx++)
    {
      if (packet_[idx].val == packet_[idx - 1].val)
      {
        if (packet_[idx].val == DCC_RMT_ZERO_BIT.val)
        {
          packet_[idx].duration0 += (idx % DCC_RMT_MAX_ZERO_BIT_SPREAD);
          packet_[idx].duration1 += (idx % DCC_RMT_MAX_ZERO_BIT_SPREAD);
        }
        else
        {
          packet_[idx].duration0 += (idx % DCC_RMT_MAX_ONE_BIT_SPREAD);
          packet_[idx].duration1 += (idx % DCC_RMT_MAX_ONE_BIT_SPREAD);
        }
      }
    }
#endif
    // record the repeat count
    pktRepeatCount_ = packet.packet_header.rept_count;

    railcomDriver_->set_feedback_key(packet.feedback_key);
  }

  DISALLOW_COPY_AND_ASSIGN(RMTTrackDevice);
};

} // namespace esp32cs

#endif // _RMT_TRACK_DEVICE_H_
