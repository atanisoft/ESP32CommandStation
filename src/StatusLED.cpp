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

#include "ESP32CommandStation.h"

#if STATUS_LED_ENABLED
constexpr uint8_t STATUS_LED_RMT_DIVIDER = 2;
constexpr rmt_channel_t STATUS_LED_RMT_CHANNEL = RMT_CHANNEL_3;
constexpr gpio_num_t STATUS_LED_GPIO = (gpio_num_t)STATUS_LED_DATA_PIN;

static constexpr uint32_t ZERO_BIT_PULSE_HIGH = 14;
static constexpr uint32_t ONE_BIT_PULSE_HIGH = 52;
static constexpr uint32_t BIT_PULSE_LOW = 52;
static constexpr uint32_t RESET_PULSE = 52;

static constexpr rmt_item32_t WS2811_ZERO_BIT = {{{ ZERO_BIT_PULSE_HIGH, 1, BIT_PULSE_LOW, 0 }}};
static constexpr rmt_item32_t WS2811_ONE_BIT = {{{ ONE_BIT_PULSE_HIGH, 1, BIT_PULSE_LOW, 0 }}};
static constexpr rmt_item32_t WS2811_RESET_BIT = {{{ RESET_PULSE, 1, RESET_PULSE, 0 }}};

constexpr rmt_item32_t LED_RED_DATA[] = {
    /* RED */
    WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT,
    WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT,
    /* GREEN */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    /* BLUE */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT
};

constexpr rmt_item32_t LED_GREEN_DATA[] = {
    /* RED */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    /* GREEN */
    WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT,
    WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT,
    /* BLUE */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT
};

constexpr rmt_item32_t LED_YELLOW_DATA[] = {
    /* RED */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    /* GREEN */
    WS2811_ZERO_BIT, WS2811_ONE_BIT, WS2811_ONE_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ONE_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    /* BLUE */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT
};

constexpr rmt_item32_t LED_OFF_DATA[] = {
    /* RED */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    /* GREEN */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    /* BLUE */
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT,
    WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT, WS2811_ZERO_BIT
};

STATUS_LED_COLOR statusLEDs[STATUS_LED::MAX_STATUS_LED] = {STATUS_LED_COLOR::LED_OFF, STATUS_LED_COLOR::LED_OFF, STATUS_LED_COLOR::LED_OFF};
bool statusLEDOn[STATUS_LED::MAX_STATUS_LED] = {false, false, false};
rmt_item32_t statusLEDPacket[(24*STATUS_LED::MAX_STATUS_LED)+1];

void updateStatusLEDs(void *arg) {
    while(true) {
        for(int led = 0; led < STATUS_LED::MAX_STATUS_LED; led++) {
            switch(statusLEDs[led]) {
                case LED_OFF:
                    memcpy(&statusLEDPacket[led * 24], LED_OFF_DATA, 24);
                    break;
                case LED_RED:
                    memcpy(&statusLEDPacket[led * 24], LED_RED_DATA, 24);
                    break;
                case LED_GREEN:
                    memcpy(&statusLEDPacket[led * 24], LED_GREEN_DATA, 24);
                    break;
                case LED_YELLOW:
                    memcpy(&statusLEDPacket[led * 24], LED_YELLOW_DATA, 24);
                    break;
                case LED_RED_BLINK:
                    if(statusLEDOn[led]) {
                        memcpy(&statusLEDPacket[led * 24], LED_OFF_DATA, 24);
                    } else {
                        memcpy(&statusLEDPacket[led * 24], LED_RED_DATA, 24);
                    }
                    statusLEDOn[led] = !statusLEDOn[led];
                    break;
                case LED_GREEN_BLINK:
                    if(statusLEDOn[led]) {
                        memcpy(&statusLEDPacket[led * 24], LED_OFF_DATA, 24);
                    } else {
                        memcpy(&statusLEDPacket[led * 24], LED_GREEN_DATA, 24);
                    }
                    statusLEDOn[led] = !statusLEDOn[led];
                    break;
                case LED_YELLOW_BLINK:
                    if(statusLEDOn[led]) {
                        memcpy(&statusLEDPacket[led * 24], LED_OFF_DATA, 24);
                    } else {
                        memcpy(&statusLEDPacket[led * 24], LED_YELLOW_DATA, 24);
                    }
                    statusLEDOn[led] = !statusLEDOn[led];
                    break;
            }
        }
        statusLEDPacket[24*STATUS_LED::MAX_STATUS_LED].val = WS2811_RESET_BIT.val;
        rmt_write_items(STATUS_LED_RMT_CHANNEL, statusLEDPacket, (24*STATUS_LED::MAX_STATUS_LED)+1, true);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void setStatusLED(const STATUS_LED led, const STATUS_LED_COLOR color) {
    statusLEDs[led] = color;
}

void initStatusLEDs() {
    rmt_config_t rmtConfig = {
        .rmt_mode = RMT_MODE_TX,
        .channel = STATUS_LED_RMT_CHANNEL,
        .clk_div = STATUS_LED_RMT_DIVIDER,
        .gpio_num = STATUS_LED_GPIO,
        .mem_block_num = 1,
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
    ESP_ERROR_CHECK(rmt_driver_install(STATUS_LED_RMT_CHANNEL, 0, 0));
    xTaskCreate(updateStatusLEDs, "LED", 2048, nullptr, 1, nullptr);
}

#endif
