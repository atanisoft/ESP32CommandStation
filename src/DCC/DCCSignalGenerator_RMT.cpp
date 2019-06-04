/**********************************************************************
ESP32 COMMAND STATION

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

#include "ESP32CommandStation.h"

// APB/REF clock divider to use for the RMT module
static constexpr uint8_t RMT_CLOCK_DIVIDER = 80;

// number of microseconds for each half of the DCC signal for a zero
static constexpr uint32_t ZERO_BIT_PULSE_USEC = 98;

// number of microseconds for each half of the DCC signal for a one
static constexpr uint32_t ONE_BIT_PULSE_USEC = 58;

static constexpr TickType_t MAX_PACKET_TX_TIME = 100000L;

static constexpr rmt_item32_t DCC_ZERO_BIT = {{{ ZERO_BIT_PULSE_USEC, 1, ZERO_BIT_PULSE_USEC, 0 }}};
static constexpr rmt_item32_t DCC_ONE_BIT = {{{ ONE_BIT_PULSE_USEC, 1, ONE_BIT_PULSE_USEC, 0 }}};
/*
// pre-encoded DCC preamble in RMT format
static constexpr rmt_item32_t DCC_PREAMBLE[] = {
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT
};
*/

constexpr uint8_t MAX_DCC_PACKET_BITS = 128;

constexpr uint32_t RMT_TASK_STACK_SIZE = 3084;
constexpr BaseType_t RMT_TASK_PRIORITY = ESP_TASK_TCPIP_PRIO;
constexpr BaseType_t RMT_TASK_CORE = 0;


#define CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
    for(int index = 0; \
        index < packet->numberOfBits; \
        index++, encodedBitCount++) { \
        if(packet->buffer[index / 8] & DCC_PACKET_BIT_MASK[index % 8]) { \
            encodedPacket[encodedBitCount].val = DCC_ONE_BIT.val; \
        } else { \
            encodedPacket[encodedBitCount].val = DCC_ZERO_BIT.val; \
        } \
    } \
    encodedPacket[encodedBitCount++].val = DCC_ONE_BIT.val;
// NOTE: the above extra one bit is to ensure the RMT sends the last bit of the packet without
// stretching the LOW wave portion. This extra bit will be ignored by the decoders as an extra
// preamble bit.

#define RMT_TRANSMIT_BITS(signal, bits, count, wait) \
    ESP_ERROR_CHECK(rmt_write_items(signal->_rmtChannel, bits, count, wait));

#define RMT_WAIT_FOR_TRANSMIT_COMPLETE(signal, time) \
    ESP_ERROR_CHECK(rmt_wait_tx_done(signal->_rmtChannel, time));

#define RMT_TRANSMIT_DCC(signal, preambleBitCount) \
    while(xSemaphoreTake(signal->_stopRequest, 0) != pdTRUE) { \
        auto packet = signal->getPacket(); \
        if(packet) { \
            uint8_t encodedBitCount = 0; \
            rmt_item32_t encodedPacket[MAX_DCC_PACKET_BITS]; \
            CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
            RMT_TRANSMIT_BITS(signal, encodedPacket, encodedBitCount, true) \
            packet->currentBit = packet->numberOfBits; \
        } \
    }
constexpr uint8_t RAILCOM_PACKET_END_DELAY = 1;
constexpr uint8_t RAILCOM_BRAKE_ENABLE_DELAY = 1;
constexpr uint8_t RAILCOM_BRAKE_DISABLE_DELAY = 10;

#define RMT_TRANSMIT_DCC_WITH_RAILCOM(signal, preambleBitCount) \
    while(xSemaphoreTake(signal->_stopRequest, 0) != pdTRUE) { \
        auto packet = signal->getPacket(); \
        if (packet) { \
            uint8_t encodedBitCount = 0; \
            rmt_item32_t encodedPacket[MAX_DCC_PACKET_BITS]; \
            CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
            RMT_TRANSMIT_BITS(signal, encodedPacket, encodedBitCount, true) \
            packet->currentBit = packet->numberOfBits; \
            digitalWrite(signal->_signalPin, LOW); \
            delayMicroseconds(RAILCOM_PACKET_END_DELAY); \
            digitalWrite(signal->_brakeEnablePin, HIGH); \
            delayMicroseconds(RAILCOM_BRAKE_ENABLE_DELAY); \
            digitalWrite(signal->_outputEnablePin, LOW); \
            digitalWrite(signal->_railComEnablePin, HIGH); \
            signal->receiveRailComData(); \
            digitalWrite(signal->_railComEnablePin, LOW); \
            if(digitalRead(signal->_railComShortPin)) { \
                /* TBD */ \
            } \
            digitalWrite(signal->_outputEnablePin, HIGH); \
            delayMicroseconds(RAILCOM_BRAKE_DISABLE_DELAY); \
            digitalWrite(signal->_brakeEnablePin, LOW); \
            /* signal pin will reset with next packet automatically */ \
        } \
    }

static void RMT_task_entry(void *param) {
    SignalGenerator_RMT *signal = static_cast<SignalGenerator_RMT *>(param);
    xSemaphoreTake(signal->_stopComplete, portMAX_DELAY);
    LOG(INFO, "[%s] RMT feeder task starting up", signal->getName());
    if(signal->_rmtChannel == DCC_SIGNAL_PROGRAMMING) {
        // for PROG track there is no RailCom support
        RMT_TRANSMIT_DCC(signal, PROG_TRACK_PREAMBLE_BITS)
    } else if(signal->_outputEnablePin != NOT_A_PIN && signal->_brakeEnablePin != NOT_A_PIN && 
              signal->_railComEnablePin != NOT_A_PIN && signal->_railComShortPin != NOT_A_PIN) {
        LOG(INFO, "[%s] Enabling RailCom detection", signal->getName());
        // Enable RailCom detection on OPS output
        RMT_TRANSMIT_DCC_WITH_RAILCOM(signal, OPS_TRACK_PREAMBLE_BITS)
    } else {
        // No RailCom support
        RMT_TRANSMIT_DCC(signal, OPS_TRACK_PREAMBLE_BITS)
    }
    LOG(INFO, "[%s] RMT feeder task shut down", signal->getName());
    xSemaphoreGive(signal->_stopComplete);
    vTaskDelete(NULL);
}

SignalGenerator_RMT::SignalGenerator_RMT(String name, uint16_t maxPackets, uint8_t signalID, uint8_t signalPin,
    uint8_t outputEnablePin, uint8_t brakeEnablePin, uint8_t railComEnablePin, uint8_t railComShortPin,
    uint8_t railComUART, uint8_t railComReceivePin) : SignalGenerator(name, maxPackets, signalID, signalPin),
    _rmtChannel((rmt_channel_t)signalID), _signalPin(signalPin), _outputEnablePin(outputEnablePin),
    _brakeEnablePin(brakeEnablePin), _railComEnablePin(railComEnablePin), _railComShortPin(railComShortPin) {

    LOG(INFO, "[%s] Configuring RMT-%d using pin %d and bit timing: zero: %duS, one: %duS",
        _name.c_str(), _rmtChannel, _signalPin, ZERO_BIT_PULSE_USEC, ONE_BIT_PULSE_USEC);

    if(_railComEnablePin != NOT_A_PIN && _brakeEnablePin != NOT_A_PIN &&
       railComReceivePin != NOT_A_PIN && railComUART != NOT_A_PIN) {
        pinMode(_railComEnablePin, OUTPUT);
        digitalWrite(_railComEnablePin, LOW);
        pinMode(_brakeEnablePin, OUTPUT);
        digitalWrite(_brakeEnablePin, LOW);
        _railComUART = uartBegin(railComUART, 250000L, SERIAL_8N1, railComReceivePin, -1, 128, false);
    }

    _stopRequest = xSemaphoreCreateBinary();
    _stopComplete = xSemaphoreCreateBinary();
    xSemaphoreGive(_stopComplete);

    rmt_config_t rmtConfig = {
        .rmt_mode = RMT_MODE_TX,
        .channel = _rmtChannel,
        .clk_div = RMT_CLOCK_DIVIDER,
        .gpio_num = (gpio_num_t)_signalPin,
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
    LOG(INFO, "[%s] Creating RMT feeder task", _name.c_str());
    xSemaphoreGive(_stopComplete);
    xTaskCreatePinnedToCore(RMT_task_entry, _name.c_str(), RMT_TASK_STACK_SIZE,
                            this, RMT_TASK_PRIORITY, nullptr, RMT_TASK_CORE);
}

void SignalGenerator_RMT::disable() {
    LOG(INFO, "[%s] Requesting RMT feeder task to stop", _name.c_str());
    xSemaphoreGive(_stopRequest);
    while(xSemaphoreTake(_stopComplete, pdMS_TO_TICKS(250)) != pdTRUE) {
        LOG(INFO, "[%s] RMT feeder task still running...", _name.c_str());
    }
    LOG(INFO, "[%s] RMT Feeder task stopped", _name.c_str());
}

void SignalGenerator_RMT::receiveRailComData() {
    std::vector<uint8_t> data(8);
    while(uartAvailable(_railComUART)) {
        data.push_back(uartRead(_railComUART));
    }
    if(data.size() < 2 || data.size() > 8) {
        LOG_ERROR("[%s] Invalid RailCom data length of %d received.", _name.c_str(), data.size());
    } else {
        dcc::Feedback feedback;
        auto dataPtr = data.begin();
        feedback.add_ch1_data(*++dataPtr);
        feedback.add_ch1_data(*++dataPtr);
        while(dataPtr != data.end()) {
            feedback.add_ch2_data(*++dataPtr);
        }
        //_railComData.push_back(feedback);
        LOG(INFO, "[%s] RailCom: %s", _name.c_str(), railcom_debug(feedback).c_str());
    }
}