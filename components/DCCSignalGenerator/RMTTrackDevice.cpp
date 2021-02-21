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

#include "RMTTrackDevice.h"
#include "sdkconfig.h"

#include <dcc/DccDebug.hxx>

namespace esp32cs
{

#if CONFIG_DCC_RMT_USE_REF_CLOCK
// Use the REF (1Mhz) clock for the RMT
static constexpr const char * const RMT_TRACK_DEVICE_CLOCK_SOURCE = "REF";
#else
// Use the APB (80Mhz) clock for the RMT.
static constexpr const char * const RMT_TRACK_DEVICE_CLOCK_SOURCE = "APB";
#endif // CONFIG_DCC_RMT_USE_REF_CLOCK

#if CONFIG_DCC_RMT_HIGH_FIRST
// This configures the first half of the bit to be sent as a high and second
// half as low.
static constexpr const char * const RMT_TRACK_DEVICE_DCC_WAVE_FMT = "high,low";
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_TOP_HALF = 1;
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_BOTTOM_HALF = 0;
#else
// This configures the first half of the bit to be sent as a low and second
// half as high.
static constexpr const char * const RMT_TRACK_DEVICE_DCC_WAVE_FMT = "low,high";
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_TOP_HALF = 0;
static constexpr uint8_t RMT_TRACK_DEVICE_DCC_BOTTOM_HALF = 1;
#endif // CONFIG_DCC_RMT_HIGH_FIRST

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
// The timing can be adjusted via menuconfig with the above being the default
// values when using the APB clock.
//
// NMRA S-9.1 reference:
// https://www.nmra.org/sites/default/files/standards/sandrp/pdf/s-9.1_electrical_standards_2020.pdf
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// DCC ZERO bit pre-encoded in RMT format.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t DCC_RMT_ZERO_BIT =
{{{
    CONFIG_DCC_RMT_TICKS_ZERO_PULSE  // number of microseconds for TOP half
  , RMT_TRACK_DEVICE_DCC_TOP_HALF    // of the square wave.
  , CONFIG_DCC_RMT_TICKS_ZERO_PULSE  // number of microseconds for BOTTOM half
  , RMT_TRACK_DEVICE_DCC_BOTTOM_HALF // of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// DCC ONE bit pre-encoded in RMT format.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t DCC_RMT_ONE_BIT =
{{{
    CONFIG_DCC_RMT_TICKS_ONE_PULSE   // number of microseconds for TOP half
  , RMT_TRACK_DEVICE_DCC_TOP_HALF    // of the square wave.
  , CONFIG_DCC_RMT_TICKS_ONE_PULSE   // number of microseconds for BOTTOM half
  , RMT_TRACK_DEVICE_DCC_BOTTOM_HALF // of the square wave.
}}};

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
// The ESP32 RMT peripheral will continue transmitting bits until it reaches a
// special marker bit which is all zeros.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t RMT_END_OF_PACKET_BIT = 
{{{
    0,0,0,0
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
// Maximum amount of time to wait for the packet queue to have space available
// for another packet before giving up. Currently configured to not wait.
///////////////////////////////////////////////////////////////////////////////
static constexpr BaseType_t MAX_PACKET_QUEUE_WAIT_TIME = 0;

///////////////////////////////////////////////////////////////////////////////
// malloc() capabilities to use for the DCC packet queue. This is configured to
// use internal 8-bit capable memory only.
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t RMT_MALLOC_CAPS = MALLOC_CAP_INTERNAL |
                                            MALLOC_CAP_8BIT;

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
                             , gpio_num_t signal_pin
                             , RailcomDriver *railcomDriver)
                             : name_(name)
                             , channel_(channel)
                             , dccPreambleBitCount_(dccPreambleBitCount)
                             , railcomDriver_(railcomDriver)
{
  // allocate buffer space for the packet queue items.
  packetQueueBuf_ =
    heap_caps_calloc(packet_queue_len, sizeof(dcc::Packet), RMT_MALLOC_CAPS);
  HASSERT(packetQueueBuf_ != nullptr);

  // create the packet queue using the pre-allocated memory.
  packetQueueHandle_ =
    xQueueCreateStatic(packet_queue_len, sizeof(dcc::Packet)
                     , (uint8_t*)packetQueueBuf_, &packetQueue_);
  HASSERT(packetQueueHandle_ != NULL);

  // calculate the maximum number of bits that will be transmitted in a single
  // dcc packet, with the current configuration the maximum number of bits in a
  // single dcc packet is 192.
  uint16_t maxBitCount = dccPreambleBitCount_             // preamble bits
                        + 1                               // packet start bit
                        + (dcc::Packet::MAX_PAYLOAD * 8)  // payload bits
                        +  dcc::Packet::MAX_PAYLOAD       // end of byte bits
                        + 1                               // end of packet bit
                        + 1                               // RMT extra bit
                        + 1;                              // RMT "end of data" marker
  HASSERT(maxBitCount <= MAX_RMT_BITS);
  uint8_t memoryBlocks = (maxBitCount / RMT_MEM_ITEM_NUM) + 1;
  HASSERT(memoryBlocks <= MAX_RMT_MEMORY_BLOCKS);

  LOG(INFO, "[%s] DCC config: zero:%duS, one:%duS, preamble-bits:%d, wave:%s"
    , name_, CONFIG_DCC_RMT_TICKS_ZERO_PULSE, CONFIG_DCC_RMT_TICKS_ONE_PULSE
    , dccPreambleBitCount_, RMT_TRACK_DEVICE_DCC_WAVE_FMT);
#if 0
  LOG(INFO
    , "[%s] Marklin-Motorola config: zero: %duS (high), zero: %duS (low), "
      "one: %duS (high), one: %duS (low), preamble: %duS (low)",
    , name_, MARKLIN_ZERO_BIT_PULSE_HIGH_USEC, MARKLIN_ZERO_BIT_PULSE_LOW_USEC
    , MARKLIN_ONE_BIT_PULSE_HIGH_USEC, MARKLIN_ONE_BIT_PULSE_LOW_USEC
    , MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC + MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC);
#endif
  LOG(INFO, "[%s] signal pin: %d, RMT(ch:%d,mem:%d[%d],clk-div:%d,clk-src:%s)"
    , name_, signal_pin, channel_, maxBitCount, memoryBlocks
    , CONFIG_DCC_RMT_CLOCK_DIVIDER, RMT_TRACK_DEVICE_CLOCK_SOURCE);

  // Configure the RMT channel for TX
  rmt_config_t config;
  bzero(&config, sizeof(rmt_config_t));
  config.rmt_mode = RMT_MODE_TX;
  config.channel = channel_;
  config.clk_div = CONFIG_DCC_RMT_CLOCK_DIVIDER;
  config.gpio_num = signal_pin;
  config.mem_block_num = memoryBlocks;
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(channel_, 0, RMT_ISR_FLAGS));

#if CONFIG_DCC_RMT_USE_REF_CLOCK
  LOG(INFO, "[%s] Configuring RMT to use REF_CLK", name_);
  ESP_ERROR_CHECK(rmt_set_source_clk(channel_, RMT_BASECLK_REF));
#endif // CONFIG_DCC_RMT_USE_APB_CLOCK

  LOG(INFO, "[%s] Starting signal generator", name_);
  // send one bit to kickstart the signal, remaining data will come from the
  // packet queue. We intentionally do not wait for the RMT TX complete here.
  rmt_write_items(channel_, &DCC_RMT_ONE_BIT, 1, false);
}

///////////////////////////////////////////////////////////////////////////////
// RMTTrackDevice destructor.
//
// Frees resources allocated for this RMT track device.
//
///////////////////////////////////////////////////////////////////////////////
RMTTrackDevice::~RMTTrackDevice()
{
  LOG(INFO, "[%s] Shutting down signal generator", name_);
  rmt_driver_uninstall(channel_);
  if (packetQueueHandle_ != NULL)
  {
    vQueueDelete(packetQueueHandle_);
  }
  if (packetQueueBuf_ != nullptr)
  {
    free(packetQueueBuf_);
  }
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

  if (xQueueSendToBack(packetQueueHandle_, sourcePacket
                     , MAX_PACKET_QUEUE_WAIT_TIME) == pdTRUE)
  {
    return 1;
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
  BaseType_t woken = pdFALSE;
  encode_next_packet(&woken);
  railcomDriver_->start_cutout();

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,2,0)
  // NOTE: This is not using rmt_write_items as it is not safe within an ISR
  // context which this callback is invoked from.
  // NOTE: rmt_fill_tx_items will truncate the packet length to 64 bits in
  // IDF v4.1, this has been fixed in IDF v4.2.
  rmt_fill_tx_items(channel_, packet_, pktLength_, 0);

  // start the transmit using the rmt_tx_start method which is ISR safe as of
  // IDF v4.1.
  rmt_tx_start(channel_, true);
#else
  // send the packet to the RMT.
  for(uint32_t index = 0; index < pktLength_; index++)
  {
    RMTMEM.chan[channel_].data32[index].val = packet_[index].val;
  }
  // reset the TX memory read offset and trigger TX start.
  RMT.conf_ch[channel_].conf1.mem_rd_rst = 1;
  RMT.conf_ch[channel_].conf1.mem_owner = RMT_MEM_OWNER_TX;
  RMT.conf_ch[channel_].conf1.tx_start = 1;
#endif // IDF 4.2+

  // if we need to wake up another task we can safely do it after sending the
  // packet off to be transmitted.
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
  portYIELD_FROM_ISR(woken);
#else
  if (woken == pdTRUE)
  {
    portYIELD_FROM_ISR();
  }
#endif
}

///////////////////////////////////////////////////////////////////////////////
// Encode the next packet or reuse the existing packet.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::encode_next_packet(BaseType_t *woken)
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
  // Add marker to the end of the DCC packet data to allow the RMT to know it
  // can stop transmitting at this point.
  packet_[pktLength_++].val = RMT_END_OF_PACKET_BIT.val;
  // record the repeat count
  pktRepeatCount_ = packet.packet_header.rept_count;

  railcomDriver_->set_feedback_key(packet.feedback_key);
}

} // namespace esp32cs
