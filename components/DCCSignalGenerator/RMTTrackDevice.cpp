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

#include "RMTTrackDevice.h"
#include "sdkconfig.h"

#include <dcc/DccDebug.hxx>
#include <soc/gpio_struct.h>


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
//    |      96      |
// ---|     usec     |      96      ---
//                   |     usec     |
//                   ----------------
// DCC ONE:
//    --------
//    |  58  |      
// ---| usec |  58  ---
//           | usec |
//           --------
//
// The timing can be adjusted via menuconfig with the above being the default
// values when using the APB clock.
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// DCC ZERO bit pre-encoded in RMT format.
///////////////////////////////////////////////////////////////////////////////
#if CONFIG_DCC_RMT_HIGH_FIRST
static constexpr rmt_item32_t DCC_RMT_ZERO_BIT =
{{{
    CONFIG_DCC_RMT_TICKS_ZERO_PULSE // number of microseconds for TOP half
  , 1                               // of the square wave.
  , CONFIG_DCC_RMT_TICKS_ZERO_PULSE // number of microseconds for BOTTOM half
  , 0                               // of the square wave.
}}};
#else
{{{
    CONFIG_DCC_RMT_TICKS_ZERO_PULSE // number of microseconds for TOP half
  , 0                               // of the square wave.
  , CONFIG_DCC_RMT_TICKS_ZERO_PULSE // number of microseconds for BOTTOM half
  , 1                               // of the square wave.
}}};
#endif // CONFIG_DCC_RMT_HIGH_FIRST

///////////////////////////////////////////////////////////////////////////////
// DCC ONE bit pre-encoded in RMT format.
///////////////////////////////////////////////////////////////////////////////
#if CONFIG_DCC_RMT_HIGH_FIRST
static constexpr rmt_item32_t DCC_RMT_ONE_BIT =
{{{
    CONFIG_DCC_RMT_TICKS_ONE_PULSE  // number of microseconds for TOP half
  , 1                               // of the square wave.
  , CONFIG_DCC_RMT_TICKS_ONE_PULSE  // number of microseconds for BOTTOM half
  , 0                               // of the square wave.
}}};
#else
static constexpr rmt_item32_t DCC_RMT_ONE_BIT =
{{{
    CONFIG_DCC_RMT_TICKS_ONE_PULSE  // number of microseconds for TOP half
  , 0                               // of the square wave.
  , CONFIG_DCC_RMT_TICKS_ONE_PULSE  // number of microseconds for BOTTOM half
  , 1                               // of the square wave.
}}};
#endif // CONFIG_DCC_RMT_HIGH_FIRST

///////////////////////////////////////////////////////////////////////////////
// Marklin Motorola bit timing (WIP)
// https://people.zeelandnet.nl/zondervan/digispan.html
// http://www.drkoenig.de/digital/motorola.htm
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t MARKLIN_ZERO_BIT_PULSE_HIGH_USEC = 182;
static constexpr uint32_t MARKLIN_ZERO_BIT_PULSE_LOW_USEC = 26;
static constexpr uint32_t MARKLIN_ONE_BIT_PULSE_HIGH_USEC = 26;
static constexpr uint32_t MARKLIN_ONE_BIT_PULSE_LOW_USEC = 182;
static constexpr uint32_t MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC = 104;
static constexpr uint32_t MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC = 104;

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
    ESP_INTR_FLAG_LOWMED              // ISR is implemented in C code
  | ESP_INTR_FLAG_SHARED              // ISR is shared across multiple handlers
);

///////////////////////////////////////////////////////////////////////////////
// Bit mask constants used as part of the packet translation layer.
///////////////////////////////////////////////////////////////////////////////
static constexpr DRAM_ATTR uint8_t PACKET_BIT_MASK[] =
{
  0x80, 0x40, 0x20, 0x10, //
  0x08, 0x04, 0x02, 0x01  //
};

///////////////////////////////////////////////////////////////////////////////
// RMTTrackDevice constructor.
//
// This creates a VFS interface for the packet queue which can be used by
// the LocalTrackIf implementation.
//
// The VFS mount point is /dev/track. This must be opened by the caller before
// the LocalTrackIf is able to route dcc::Packets to the track.
//
// This also allocates two h-bridge monitoring state flows, these will check
// for short circuits and disable the track output from the h-bridge
// independently from the RMT signal being generated.
///////////////////////////////////////////////////////////////////////////////
RMTTrackDevice::RMTTrackDevice(const char *name
                             , const rmt_channel_t channel
                             , const uint8_t dccPreambleBitCount
                             , size_t packet_queue_len
                             , gpio_num_t pin
                             , RailcomDriver *railcomDriver)
                             : name_(name)
                             , channel_(channel)
                             , dccPreambleBitCount_(dccPreambleBitCount)
                             , railcomDriver_(railcomDriver)
                             , packetQueue_(DeviceBuffer<dcc::Packet>::create(packet_queue_len))
{
  uint16_t maxBitCount = dccPreambleBitCount_             // preamble bits
                        + 1                               // packet start bit
                        + (dcc::Packet::MAX_PAYLOAD * 8)  // payload bits
                        +  dcc::Packet::MAX_PAYLOAD       // end of byte bits
                        + 1                               // end of packet bit
                        + 1;                              // RMT extra bit
  HASSERT(maxBitCount <= MAX_RMT_BITS);

  uint8_t memoryBlocks = (maxBitCount / RMT_MEM_ITEM_NUM) + 1;
  HASSERT(memoryBlocks <= MAX_RMT_MEMORY_BLOCKS);

  LOG(INFO
    , "[%s] DCC config: zero: %duS, one: %duS, preamble-bits: %d, wave: %s"
    , name_, CONFIG_DCC_RMT_TICKS_ZERO_PULSE, CONFIG_DCC_RMT_TICKS_ONE_PULSE
    , dccPreambleBitCount_
#if CONFIG_DCC_RMT_HIGH_FIRST
    , "high, low"
#else
    , "low, high"
#endif
    );
  /*LOG(INFO
    , "[%s] Marklin-Motorola config: zero: %duS (high), zero: %duS (low), "
      "one: %duS (high), one: %duS (low), preamble: %duS (low)",
    , name_, MARKLIN_ZERO_BIT_PULSE_HIGH_USEC, MARKLIN_ZERO_BIT_PULSE_LOW_USEC
    , MARKLIN_ONE_BIT_PULSE_HIGH_USEC, MARKLIN_ONE_BIT_PULSE_LOW_USEC
    , MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC + MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC);*/
  LOG(INFO
    , "[%s] signal pin: %d, RMT(ch:%d,mem:%d[%d],clk-div:%d,clk-src:%s)"
    , name_, pin, channel_, maxBitCount, memoryBlocks
    , CONFIG_DCC_RMT_CLOCK_DIVIDER
#if defined(CONFIG_DCC_RMT_USE_REF_CLOCK)
    , "REF"
#else
    , "APB"
#endif // CONFIG_DCC_RMT_USE_REF_CLOCK
  );
  rmt_config_t config =
  {
    .rmt_mode = RMT_MODE_TX,
    .channel = channel_,
    .clk_div = CONFIG_DCC_RMT_CLOCK_DIVIDER,
    .gpio_num = pin,
    .mem_block_num = memoryBlocks,
    {
      .tx_config =
      {
        .loop_en = false,
        .carrier_freq_hz = 0,
        .carrier_duty_percent = 0,
        .carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW,
        .carrier_en = false,
        .idle_level = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW,
        .idle_output_en = false
      }
    }
  };
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(channel_, 0, RMT_ISR_FLAGS));

#if defined(CONFIG_DCC_RMT_USE_REF_CLOCK)
  LOG(INFO, "[%s] Configuring RMT to use REF_CLK", name_);
  ESP_ERROR_CHECK(rmt_set_source_clk(channel_, RMT_BASECLK_REF));
#endif // CONFIG_DCC_RMT_USE_APB_CLOCK

  LOG(INFO, "[%s] Starting signal generator", name_);
  // send one bit to kickstart the signal, remaining data will come from the
  // packet queue. We intentionally do not wait for the RMT TX complete here.
  rmt_write_items(channel_, &DCC_RMT_ONE_BIT, 1, false);
}

///////////////////////////////////////////////////////////////////////////////
// ESP VFS callback for ::write()
//
// This will write *ONE* dcc::Packet to either the OPS or PROG packet queue. If
// there is no space in the packet queue the packet will be rejected and errno
// set to ENOSPC.
//
// NOTE: At this time Marklin packets will be actively rejected.
///////////////////////////////////////////////////////////////////////////////
ssize_t RMTTrackDevice::write(int fd, const void * data, size_t size)
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

  {
    AtomicHolder l(&packetQueueLock_);
    dcc::Packet* writePacket;
    if (packetQueue_->space() &&
        packetQueue_->data_write_pointer(&writePacket))
    {
      memcpy(writePacket, data, size);
      packetQueue_->advance(1);
      return 1;
    }
  }
  // packet queue is full!
  errno = ENOSPC;
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// ESP VFS callback for ::ioctl()
//
// When the cmd is CAN_IOC_WRITE_ACTIVE the packet queue will be checked. When
// there is no space in the queue the Notifiable will be stored to be called
// after the next DCC packet has been transmitted. Any existing Notifiable will
// be called to requeue themselves if necessary.
///////////////////////////////////////////////////////////////////////////////
int RMTTrackDevice::ioctl(int fd, int cmd, va_list args)
{
  // Attempt to write a Packet to the queue
  if (IOC_TYPE(cmd) == CAN_IOC_MAGIC && IOC_SIZE(cmd) == NOTIFIABLE_TYPE &&
      cmd == CAN_IOC_WRITE_ACTIVE)
  {
    Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
    HASSERT(n);
    {
      AtomicHolder l(&packetQueueLock_);
      if (!packetQueue_->space())
      {
        // stash the notifiable so we can call it later when there is space
        std::swap(n, notifiable_);
      }
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

///////////////////////////////////////////////////////////////////////////////
// RMT transmit complete callback.
//
// When RailCom is enabled this will poll for RailCom data before transmission
// of the next dcc::Packet from the queue.
//
// Note: This does not use ESP-IDF provided rmt_write_items to increase
// performance by avoiding various FreeRTOS functions used by the API.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::rmt_transmit_complete()
{
  encode_next_packet();
  railcomDriver_->start_cutout();

  // send the packet to the RMT, note not using memcpy for the packet as this
  // directly accesses hardware registers.
  RMT.apb_conf.fifo_mask = RMT_DATA_MODE_MEM;
  for(uint32_t index = 0; index < pktLength_; index++)
  {
    RMTMEM.chan[channel_].data32[index].val = packet_[index].val;
  }
  // RMT marker for "end of data"
  RMTMEM.chan[channel_].data32[pktLength_].val = 0;
  // start transmit
  RMT.conf_ch[channel_].conf1.mem_rd_rst = 1;
  RMT.conf_ch[channel_].conf1.mem_owner = RMT_MEM_OWNER_TX;
  RMT.conf_ch[channel_].conf1.tx_start = 1;
}

///////////////////////////////////////////////////////////////////////////////
// Transfers a dcc::Packet to the OPS packet queue.
//
// NOTE: At this time Marklin packets will be actively discarded.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::send(Buffer<dcc::Packet> *b, unsigned prio)
{
  // if it is not a Marklin Motorola packet put it in the queue, otherwise
  // discard.
  if (!b->data()->packet_header.is_marklin)
  {
    AtomicHolder l(&packetQueueLock_);
    dcc::Packet* writePacket;
    if (packetQueue_->space() &&
        packetQueue_->data_write_pointer(&writePacket))
    {
      memcpy(writePacket, b->data(), b->size());
      packetQueue_->advance(1);
    }
  }
  b->unref();
}

///////////////////////////////////////////////////////////////////////////////
// Encode the next packet or reuse the existing packet.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::encode_next_packet()
{
  // Check if we need to encode the next packet or if we still have at least
  // one repeat left of the current packet.
  if (--pktRepeatCount_ >= 0)
  {
    return;
  }
  // attempt to fetch a packet from the queue or use an idle packet
  Notifiable* n = nullptr;
  dcc::Packet packet{dcc::Packet::DCC_IDLE()};
  {
    AtomicHolder l(&packetQueueLock_);
    if (packetQueue_->get(&packet, 1))
    {
      // since we removed a packet from the queue, check if we have a pending
      // notifiable to wake up.
      std::swap(n, notifiable_);
    }
  }
  if (n)
  {
    n->notify_from_isr();
  }
  // TODO: add encoding for Marklin-Motorola

  // encode the preamble bits
  for (pktLength_ = 0; pktLength_ < dccPreambleBitCount_; pktLength_++)
  {
    packet_[pktLength_].val = DCC_RMT_ONE_BIT.val;
  }
  // start of payload marker
  packet_[pktLength_++].val = DCC_RMT_ZERO_BIT.val;
  // encode the packet bits
  for (uint8_t dlc = 0; dlc < packet.dlc; dlc++)
  {
    for(uint8_t bit = 0; bit < 8; bit++)
    {
      packet_[pktLength_++].val =
        packet.payload[dlc] & PACKET_BIT_MASK[bit] ?
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
  // record the repeat count
  pktRepeatCount_ = packet.packet_header.rept_count;

  railcomDriver_->set_feedback_key(packet.feedback_key);
}

} // namespace esp32cs
