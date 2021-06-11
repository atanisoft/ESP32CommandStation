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
template<class HW>
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
    rmt_item32_t value =
    {{{
      CONFIG_DCC_RMT_TICKS_ONE_PULSE,  // number of microseconds for first half
      RMT_TRACK_DEVICE_DCC_TOP_HALF,   // of the square wave.
      CONFIG_DCC_RMT_TICKS_ONE_PULSE,  // number of microseconds for second
      RMT_TRACK_DEVICE_DCC_BOTTOM_HALF // half of the square wave.
    }}};
    // send one bit to kickstart the signal, remaining data will come from the
    // packet queue. We intentionally do not wait for the RMT TX complete here.
    rmt_write_items(HW::RMT_CHANNEL, &value, 1, false);
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
    railcomDriver_->start_cutout();

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

  /// Marklin Motorola bit timing
  /// https://people.zeelandnet.nl/zondervan/digispan.html
  /// http://www.drkoenig.de/digital/motorola.htm
  static constexpr uint32_t MARKLIN_ZERO_BIT_PULSE_HIGH_USEC = 182;
  static constexpr uint32_t MARKLIN_ZERO_BIT_PULSE_LOW_USEC = 26;
  static constexpr uint32_t MARKLIN_ONE_BIT_PULSE_HIGH_USEC = 26;
  static constexpr uint32_t MARKLIN_ONE_BIT_PULSE_LOW_USEC = 182;
  static constexpr uint32_t MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC = 104;
  static constexpr uint32_t MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC = 104;
  #if CONFIG_DCC_RMT_EMC_SPREAD
  // NOTE: the value below is one higher than maximum to allow for modulo
  // operation.
  static constexpr uint8_t MARKLIN_RMT_MAX_BIT_SPREAD = 3;
  #endif // CONFIG_DCC_RMT_EMC_SPREAD

  ///////////////////////////////////////////////////////////////////////////////
  // Marklin Motorola ZERO bit pre-encoded in RMT format, sent as HIGH then LOW.
  ///////////////////////////////////////////////////////////////////////////////
  static constexpr rmt_item32_t MARKLIN_RMT_ZERO_BIT =
  {{{
      MARKLIN_ZERO_BIT_PULSE_HIGH_USEC  // number of microseconds for TOP half
    , 1                                 // of the square wave.
    , MARKLIN_ZERO_BIT_PULSE_LOW_USEC   // number of microseconds for BOTTOM half
    , 0                                 // of the square wave.
  }}};

  ///////////////////////////////////////////////////////////////////////////////
  // Marklin Motorola ONE bit pre-encoded in RMT format, sent as HIGH then LOW.
  ///////////////////////////////////////////////////////////////////////////////
  static constexpr rmt_item32_t MARKLIN_RMT_ONE_BIT =
  {{{
      MARKLIN_ZERO_BIT_PULSE_HIGH_USEC  // number of microseconds for TOP half
    , 1                                 // of the square wave.
    , MARKLIN_ZERO_BIT_PULSE_LOW_USEC   // number of microseconds for BOTTOM half
    , 0                                 // of the square wave.
  }}};

  ///////////////////////////////////////////////////////////////////////////////
  // Marklin Motorola preamble bit pre-encoded in RMT format, both top and bottom
  // half of the wave are LOW.
  ///////////////////////////////////////////////////////////////////////////////
  static constexpr rmt_item32_t MARKLIN_RMT_PREAMBLE_BIT =
  {{{
      MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC // number of microseconds for TOP half
    , 0                                    // of the square wave.
    , MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC  // number of microseconds for BOTTOM
    , 0                                    // half of the square wave.
  }}};

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
  rmt_item32_t packet_[SOC_RMT_MEM_WORDS_PER_CHANNEL * MAX_RMT_MEMORY_BLOCKS];

  ///////////////////////////////////////////////////////////////////////////////
  // Generates the DCC bit timing, optionally with EMC spectrum spreading.
  //
  // This method provides an optional feature that can help with passing EMC
  // certification. The observation is that if the output signal has may repeats
  // of a certain period, then in the measured spectrum there will be a big spike
  // in energy that might exceed the thresholds for compliance. However, by
  // slightly varying the timing of the output signal, the energy will be spread
  // across a wider spectrum, thus the peak of emission will be smaller.
  //
  // This feature is disabled by default and can be enabled via menuconfig under
  // DCC Signal -> Advanced options -> EMC spectrum spreading.
  ///////////////////////////////////////////////////////////////////////////////
  static inline uint32_t packet_bit_time_dcc(uint32_t index, bool one)
  {
    if (one)
    {
      rmt_item32_t value =
      {{{
        CONFIG_DCC_RMT_TICKS_ONE_PULSE,  // number of microseconds for first half
        RMT_TRACK_DEVICE_DCC_TOP_HALF,   // of the square wave.
        CONFIG_DCC_RMT_TICKS_ONE_PULSE,  // number of microseconds for second
        RMT_TRACK_DEVICE_DCC_BOTTOM_HALF // half of the square wave.
      }}};
  #if CONFIG_DCC_RMT_EMC_SPREAD
      value.duration0 += index % DCC_RMT_MAX_ONE_BIT_SPREAD;
      value.duration1 += index % DCC_RMT_MAX_ONE_BIT_SPREAD;
  #endif // CONFIG_DCC_RMT_EMC_SPREAD
      return value.val;
    }
    else
    {
      rmt_item32_t value =
      {{{
          CONFIG_DCC_RMT_TICKS_ZERO_PULSE, // number of microseconds for first
          RMT_TRACK_DEVICE_DCC_TOP_HALF,   // half of the square wave.
          CONFIG_DCC_RMT_TICKS_ZERO_PULSE, // number of microseconds for second
          RMT_TRACK_DEVICE_DCC_BOTTOM_HALF // half of the square wave.
      }}};
  #if CONFIG_DCC_RMT_EMC_SPREAD
      value.duration0 += index % DCC_RMT_MAX_ZERO_BIT_SPREAD;
      value.duration1 += index % DCC_RMT_MAX_ZERO_BIT_SPREAD;
  #endif // CONFIG_DCC_RMT_EMC_SPREAD
      return value.val;
    }
    // end of packet marker
    return 0;
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Generates the Marklin Motorola bit timing, optionally with EMC spectrum
  // spreading.
  //
  // This method provides an optional feature that can help with passing EMC
  // certification. The observation is that if the output signal has may repeats
  // of a certain period, then in the measured spectrum there will be a big spike
  // in energy that might exceed the thresholds for compliance. However, by
  // slightly varying the timing of the output signal, the energy will be spread
  // across a wider spectrum, thus the peak of emission will be smaller.
  //
  // This feature is disabled by default and can be enabled via menuconfig under
  // RMT DCC Signal -> Advanced options -> EMC spectrum spreading.
  ///////////////////////////////////////////////////////////////////////////////
  static inline uint32_t packet_bit_time_marklin(uint32_t index, bool one)
  {
    rmt_item32_t value = one ? MARKLIN_RMT_ONE_BIT : MARKLIN_RMT_ZERO_BIT;
  #if CONFIG_DCC_RMT_EMC_SPREAD
    value.duration0 += (index % MARKLIN_RMT_MAX_BIT_SPREAD);
    value.duration1 += (index % MARKLIN_RMT_MAX_BIT_SPREAD);
  #endif // CONFIG_DCC_RMT_EMC_SPREAD

    return value.val;
  }

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
    /*if (packet.packet_header.is_marklin)
    {
      // TODO: add Marklin encoding
    }
    else*/
    {
      uint32_t preableBitCount = HW::DCC_PREAMBLE_BITS;
      if (packet.packet_header.send_long_preamble)
      {
        preableBitCount = HW::DCC_SERVICE_MODE_PREAMBLE_BITS;
      }
      // encode the preamble bits
      for (pktLength_ = 0; pktLength_ < preableBitCount; pktLength_++)
      {
        packet_[pktLength_].val = packet_bit_time_dcc(pktLength_, true);
      }
      // start of payload marker
      packet_[pktLength_].val = packet_bit_time_dcc(pktLength_, false);
      pktLength_++;
      // encode the packet bits
      for (uint8_t dlc = 0; dlc < packet.dlc; dlc++)
      {
        for(uint8_t bit = 0; bit < 8; bit++, pktLength_++)
        {
          packet_[pktLength_].val =
            packet_bit_time_dcc(pktLength_,
                                packet.payload[dlc] & PACKET_BIT_MASK[bit]);
        }
        // end of byte marker
        packet_[pktLength_].val = packet_bit_time_dcc(pktLength_, false);
        pktLength_++;
      }
      // set the last bit of the encoded payload to be an end of packet marker
      packet_[pktLength_ - 1].val = packet_bit_time_dcc(pktLength_, true);
      // add an extra ONE bit to the end to prevent mangling of the last bit by
      // the RMT
      packet_[pktLength_].val = packet_bit_time_dcc(pktLength_, true);
      // Add marker to the end of the DCC packet data to allow the RMT to know it
      // can stop transmitting at this point.
      packet_[++pktLength_].val = 0;
    }
    // record the repeat count
    pktRepeatCount_ = packet.packet_header.rept_count;

    railcomDriver_->set_feedback_key(packet.feedback_key);
  }

  DISALLOW_COPY_AND_ASSIGN(RMTTrackDevice);
};

} // namespace esp32cs

#endif // _RMT_TRACK_DEVICE_H_
