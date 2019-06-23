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

#if LCC_ENABLED
#include <dcc/RailcomHub.hxx>
extern dcc::RailcomHubFlow railComHub;
#endif

// APB/REF clock divider to use for the RMT module
static constexpr uint8_t RMT_CLOCK_DIVIDER = 80;

// number of microseconds for each half of the DCC signal for a zero
static constexpr uint32_t ZERO_BIT_PULSE_USEC = 98;

// number of microseconds for each half of the DCC signal for a one
static constexpr uint32_t ONE_BIT_PULSE_USEC = 58;

static constexpr rmt_item32_t DCC_ZERO_BIT = {{{ ZERO_BIT_PULSE_USEC, 1, ZERO_BIT_PULSE_USEC, 0 }}};
static constexpr rmt_item32_t DCC_ONE_BIT = {{{ ONE_BIT_PULSE_USEC, 1, ONE_BIT_PULSE_USEC, 0 }}};

// pre-encoded DCC IDLE packet in RMT format
static constexpr rmt_item32_t DCC_IDLE_PACKET[] = {
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,     // 0xFF
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,     // 0xFF
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ZERO_BIT, DCC_ZERO_BIT,   // 0xFD
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ZERO_BIT,    // 0xFE
    DCC_ZERO_BIT, DCC_ZERO_BIT, DCC_ZERO_BIT, DCC_ZERO_BIT,
    DCC_ZERO_BIT, DCC_ZERO_BIT, DCC_ZERO_BIT, DCC_ZERO_BIT, // 0x00
    DCC_ZERO_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,
    DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT, DCC_ONE_BIT,     // 0x7F
    DCC_ONE_BIT,                                            // 0x01
    DCC_ONE_BIT
    // NOTE: The above extra bit is to ensure the RMT sends the
    // final bit of the DCC IDLE packet without stretching the
    // final LOW wave portion of the bit.
};

constexpr uint8_t MAX_DCC_PACKET_BITS = 128;

constexpr uint32_t RMT_TASK_STACK_SIZE = 4096;
constexpr BaseType_t RMT_TASK_PRIORITY = ESP_TASK_TCPIP_PRIO;
constexpr BaseType_t RMT_TASK_CORE = 0;

constexpr uint8_t RAILCOM_PACKET_END_DELAY_USEC = 1;
constexpr uint8_t RAILCOM_BRAKE_ENABLE_DELAY_USEC = 1;
constexpr uint8_t RAILCOM_BRAKE_DISABLE_DELAY_USEC = 10;

#define CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
    for(encodedBitCount = 0; \
        encodedBitCount < packet->numberOfBits; \
        encodedBitCount++) { \
        if(packet->buffer[encodedBitCount / 8] & DCC_PACKET_BIT_MASK[encodedBitCount % 8]) { \
            encodedPacket[encodedBitCount].val = DCC_ONE_BIT.val; \
        } else { \
            encodedPacket[encodedBitCount].val = DCC_ZERO_BIT.val; \
        } \
    } \
    encodedPacket[encodedBitCount++].val = DCC_ONE_BIT.val;
// NOTE: The above extra one bit is to ensure the RMT sends the final bit of the packet without
// stretching the LOW wave portion of the bit.

// Allow for up to 100 zero bits before reporting that a DCC packet was transmitted slowly,
// this should not normally be reported unless there is an issue with the RMT peripheral.
constexpr uint64_t MAX_DCC_PACKET_TIME = (ZERO_BIT_PULSE_USEC * 2) * 100;

#define RMT_TRANSMIT_BITS(signal, bits, count) \
    uint64_t ts_start = esp_timer_get_time(); \
    ESP_ERROR_CHECK(rmt_write_items(signal->_rmtChannel, bits, count, true)); \
    uint64_t ts_end = esp_timer_get_time(); \
    if ((ts_end - ts_start) > MAX_DCC_PACKET_TIME) { \
        LOG(WARNING, "[%s] SLOW DCC transmit! %s:%s, %d bits", \
            signal->getName(), uint64_to_string(ts_start).c_str(), \
            uint64_to_string(ts_end).c_str(), count); \
    }

#define RMT_TRANSMIT_DCC(signal, preambleBitCount) \
    while(xSemaphoreTake(signal->_stopRequest, 0) != pdTRUE) { \
        esp_task_wdt_reset(); \
        auto packet = signal->getNextPacket(); \
        if(packet) { \
            uint8_t encodedBitCount = 0; \
            rmt_item32_t encodedPacket[MAX_DCC_PACKET_BITS]; \
            CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
            RMT_TRANSMIT_BITS(signal, encodedPacket, encodedBitCount) \
        } else { \
            RMT_TRANSMIT_BITS(signal, DCC_IDLE_PACKET, 50) \
        } \
    }

#define RMT_TRANSMIT_DCC_WITH_RAILCOM(signal, preambleBitCount) \
    while(xSemaphoreTake(signal->_stopRequest, 0) != pdTRUE) { \
        esp_task_wdt_reset(); \
        auto packet = signal->getNextPacket(); \
        if (packet) { \
            uint8_t encodedBitCount = 0; \
            rmt_item32_t encodedPacket[MAX_DCC_PACKET_BITS]; \
            CONVERT_DCC_PACKET_TO_RMT(packet, encodedPacket, encodedBitCount) \
            RMT_TRANSMIT_BITS(signal, encodedPacket, encodedBitCount) \
        } else { \
            RMT_TRANSMIT_BITS(signal, DCC_IDLE_PACKET, 50) \
        } \
        digitalWrite(signal->_signalPin, LOW); \
        delayMicroseconds(RAILCOM_PACKET_END_DELAY_USEC); \
        digitalWrite(signal->_brakeEnablePin, HIGH); \
        delayMicroseconds(RAILCOM_BRAKE_ENABLE_DELAY_USEC); \
        digitalWrite(signal->_outputEnablePin, LOW); \
        digitalWrite(signal->_railComEnablePin, HIGH); \
        signal->receiveRailComData(); \
        digitalWrite(signal->_railComEnablePin, LOW); \
        if(digitalRead(signal->_railComShortPin)) { \
            /* TBD */ \
        } \
        digitalWrite(signal->_outputEnablePin, HIGH); \
        delayMicroseconds(RAILCOM_BRAKE_DISABLE_DELAY_USEC); \
        digitalWrite(signal->_brakeEnablePin, LOW); \
        /* signal pin will reset with next packet automatically */ \
    }

static void RMT_task_entry(void *param) {
    SignalGenerator_RMT *signal = static_cast<SignalGenerator_RMT *>(param);
    esp_task_wdt_add(NULL);
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
    int8_t outputEnablePin, int8_t brakeEnablePin, int8_t railComEnablePin, int8_t railComShortPin,
    int8_t railComUART, int8_t railComReceivePin) : SignalGenerator(name, maxPackets, signalID, signalPin),
    _rmtChannel((rmt_channel_t)signalID), _signalPin(signalPin), _outputEnablePin(outputEnablePin),
    _brakeEnablePin(brakeEnablePin), _railComEnablePin(railComEnablePin), _railComShortPin(railComShortPin) {

    InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "%s RMT Init", getName());

    LOG(INFO, "[%s] Configuring RMT-%d using pin %d and bit timing: zero: %duS, one: %duS",
        getName(), _rmtChannel, _signalPin, ZERO_BIT_PULSE_USEC, ONE_BIT_PULSE_USEC);

    if(_outputEnablePin != NOT_A_PIN && _railComEnablePin != NOT_A_PIN && _brakeEnablePin != NOT_A_PIN &&
       railComReceivePin != NOT_A_PIN && railComUART != NOT_A_PIN) {
        InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "%s RailCom Init", getName());
        LOG(INFO, "[%s] Configuring RailCom detector (hb-en: %d, rc-en: %d, br-en: %d, rc: %d, uart: %d)",
            getName(), _outputEnablePin, _railComEnablePin, _brakeEnablePin, railComReceivePin, railComUART);
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
                .carrier_en = false,
                .idle_level = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW,
                .idle_output_en = false
            }
        }
    };
    ESP_ERROR_CHECK(rmt_config(&rmtConfig));
    ESP_ERROR_CHECK(rmt_driver_install(_rmtChannel, 0, 0));
}

void SignalGenerator_RMT::enable() {
    LOG(INFO, "[%s] Creating RMT feeder task", getName());
    xSemaphoreGive(_stopComplete);
    xTaskCreatePinnedToCore(RMT_task_entry, getName(), RMT_TASK_STACK_SIZE,
                            this, RMT_TASK_PRIORITY, nullptr, RMT_TASK_CORE);
}

void SignalGenerator_RMT::disable() {
    LOG(INFO, "[%s] Requesting RMT feeder task to stop", getName());
    xSemaphoreGive(_stopRequest);
    while(xSemaphoreTake(_stopComplete, pdMS_TO_TICKS(250)) != pdTRUE) {
        LOG(INFO, "[%s] RMT feeder task still running...", getName());
    }
    LOG(INFO, "[%s] RMT Feeder task stopped", getName());
}

void SignalGenerator_RMT::receiveRailComData() {
    std::vector<uint8_t> data(8);
    while(uartAvailable(_railComUART)) {
        data.push_back(uartRead(_railComUART));
    }
    if(data.size() < 2 || data.size() > 8) {
        LOG_ERROR("[%s] Invalid RailCom data length of %d received.", getName(), data.size());
    } else {
        dcc::Feedback feedback;
        feedback.reset(0);
        auto dataPtr = data.begin();
        feedback.add_ch1_data(*dataPtr++);
        feedback.add_ch1_data(*dataPtr++);
        while(dataPtr != data.end()) {
            feedback.add_ch2_data(*dataPtr++);
        }
#if LCC_ENABLED
        auto buf = railComHub.alloc();
        memcpy(buf->data()->data(), &feedback, sizeof(dcc::Feedback));
        railComHub.send(buf);
#else
        //_railComData.push_back(feedback);
        LOG(VERBOSE, "[%s] RailCom: %s", getName(), railcom_debug(feedback).c_str());
#endif
    }
}