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

// APB/REF clock divider to use for the RMT module
static constexpr uint8_t RMT_CLOCK_DIVIDER = 80;

// number of microseconds for each half of the DCC signal for a zero
static constexpr uint32_t ZERO_BIT_PULSE = 98;
//static constexpr uint32_t ZERO_BIT_PULSE = 8640;

// number of microseconds for each half of the DCC signal for a one
static constexpr uint32_t ONE_BIT_PULSE = 58;
//static constexpr uint32_t ONE_BIT_PULSE = 4640;

static constexpr TickType_t PREAMBLE_MAX_DELAY = 1000000000L;

static constexpr rmt_item32_t DCC_ZERO_BIT = {{{ ZERO_BIT_PULSE, 1, ZERO_BIT_PULSE, 0 }}};
static constexpr rmt_item32_t DCC_ONE_BIT = {{{ ONE_BIT_PULSE, 1, ONE_BIT_PULSE, 0 }}};

// pre-encoded DCC preamble in RMT format
static constexpr rmt_item32_t DCC_PREAMBLE[] = {
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT
};

constexpr uint8_t MAX_DCC_PACKET_BITS = 64;

// TODO: remove preamble bits from Packet
// this skips the preamble bits that come pre-encoded in the packet by the DCCSignalGenerator code
#define CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
    for(int index = 22; \
        index < packet->numberOfBits; \
        index++, encodedBitCount++) { \
        if(packet->buffer[index / 8] & DCC_PACKET_BIT_MASK[index % 8]) { \
            encodedPacket[encodedBitCount].val = DCC_ONE_BIT.val; \
        } else { \
            encodedPacket[encodedBitCount].val = DCC_ZERO_BIT.val; \
        } \
    }

#define RMT_TRANSMIT_BITS(signal, bits, count, wait) \
    ESP_ERROR_CHECK(rmt_write_items(signal->_rmtChannel, bits, count, wait));

#define RMT_WAIT_FOR_TRANSMIT_COMPLETE(signal, time) \
    ESP_ERROR_CHECK(rmt_wait_tx_done(signal->_rmtChannel, time));

#define RMT_TRANSMIT_DCC(signal, preambleBitCount) \
    while(!signal->_stopRequested) { \
        auto packet = signal->getPacket(); \
        if(packet) { \
            uint8_t encodedBitCount = 0; \
            rmt_item32_t encodedPacket[MAX_DCC_PACKET_BITS]; \
            CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
            RMT_TRANSMIT_BITS(signal, DCC_PREAMBLE, preambleBitCount, true) \
            RMT_TRANSMIT_BITS(signal, encodedPacket, encodedBitCount, true) \
            packet->currentBit = packet->numberOfBits; \
        } \
    }

#define RMT_TRANSMIT_DCC_WITH_RAILCOM(signal, preambleBitCount) \
    while(!signal->_stopRequested) { \
        auto packet = signal->getPacket(); \
        if(packet) { \
            uint8_t encodedBitCount = 0; \
            rmt_item32_t encodedPacket[MAX_DCC_PACKET_BITS]; \
            CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
            RMT_TRANSMIT_BITS(signal, DCC_PREAMBLE, preambleBitCount, false) \
            /* TODO: RailCom Cutout, 488uS */ \
            RMT_WAIT_FOR_TRANSMIT_COMPLETE(signal, PREAMBLE_MAX_DELAY) \
            RMT_TRANSMIT_BITS(signal, encodedPacket, encodedBitCount, true) \
            packet->currentBit = packet->numberOfBits; \
        } \
    }

static void RMT_task_entry(void *param) {
    SignalGenerator_RMT *signal = static_cast<SignalGenerator_RMT *>(param);
    signal->_stopCompleted = false;
    if(signal->_rmtChannel == DCC_SIGNAL_PROGRAMMING) {
        // for PROG track we need to use a longer preamble
        RMT_TRANSMIT_DCC(signal, 22)
    } else {
        // for OPS track we can use a shorter preamble
        RMT_TRANSMIT_DCC_WITH_RAILCOM(signal, 16)
    }
    signal->_stopCompleted = true;
    vTaskDelete(NULL);
}

SignalGenerator_RMT::SignalGenerator_RMT(String name, uint16_t maxPackets, uint8_t signalID, uint8_t signalPin) :
    SignalGenerator(name, maxPackets, signalID, signalPin), _rmtChannel((rmt_channel_t)signalID) {

    rmt_config_t rmtConfig = {
        .rmt_mode = RMT_MODE_TX,
        .channel = _rmtChannel,
        .clk_div = RMT_CLOCK_DIVIDER,
        .gpio_num = (gpio_num_t)signalPin,
        .mem_block_num = 2,
        {
            .tx_config = {
                .loop_en = false,
                .carrier_freq_hz = 0,
                .carrier_duty_percent = 0,
                .carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW,
                .carrier_en = 0,
                .idle_level = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW,
                .idle_output_en = 0
            }
        }
    };
    ESP_ERROR_CHECK(rmt_config(&rmtConfig));
    ESP_ERROR_CHECK(rmt_driver_install(_rmtChannel, 0, 0));
}

void SignalGenerator_RMT::enable() {
    _stopRequested = false;
    xTaskCreate(RMT_task_entry, _name.c_str(), DEFAULT_THREAD_STACKSIZE, this, DEFAULT_THREAD_PRIO, nullptr);
}

void SignalGenerator_RMT::disable() {
    _stopRequested = true;
    while(!_stopCompleted) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
