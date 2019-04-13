/** \copyright
 * Copyright (c) 2019, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file Esp32HardwareCanAdapter.hxx
 *
 * ESP32 Hardware CAN adapter code. This utilizes the built in CAN controller
 * to translate the can_frame in OpenMRN to the ESP32 can_message_t used by the
 * ESP-IDF CAN controller code. The ESP32 will still require an external CAN
 * transceiver (MCP2551 or SN65HVD230 as example).
 *
 * @author Mike Dunston
 * @date 19 January 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32HWCAN_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32HWCAN_HXX_

#include "freertos_drivers/arduino/Can.hxx"
#include <driver/can.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>

namespace openmrn_arduino {

/// ESP32 CAN bus status strings, used for periodic status reporting
static const char *ESP32_CAN_STATUS_STRINGS[] = {
    "STOPPED",               // CAN_STATE_STOPPED
    "RUNNING",               // CAN_STATE_RUNNING
    "OFF / RECOVERY NEEDED", // CAN_STATE_BUS_OFF
    "RECOVERY UNDERWAY"      // CAN_STATE_RECOVERING
};

class Esp32HardwareCan : public Can
{
public:
    /// Constructor.
    ///
    /// @param name is the name for the CAN adapter, this is currently not used.
    /// @param rxPin is the ESP32 pin that is connected to the external
    /// transceiver RX.
    /// @param txPin is the ESP32 pin that is connected to the external
    /// transceiver TX.
    Esp32HardwareCan(const char *name, gpio_num_t rxPin, gpio_num_t txPin,
        bool reportStats = true)
        : Can(name)
        , reportStats_(reportStats)
        , overrunWarningPrinted_(false)
    {
        // Configure the ESP32 CAN driver to use 125kbps.
        can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_125KBITS();
        // By default we accept all CAN frames.
        can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
        // Note: not using the CAN_GENERAL_CONFIG_DEFAULT macro due to a missing
        // cast for CAN_IO_UNUSED.
        can_general_config_t can_general_config = {.mode = CAN_MODE_NORMAL,
            .tx_io = txPin,
            .rx_io = rxPin,
            .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
            .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
            .tx_queue_len = (uint32_t)config_can_tx_buffer_size() / 2,
            .rx_queue_len = (uint32_t)config_can_rx_buffer_size() / 2,
            .alerts_enabled = CAN_ALERT_NONE,
            .clkout_divider = 0};

        LOG(VERBOSE,
            "ESP32-CAN driver configured using RX: %d, TX: %d, RX-Q: %d, "
            "TX-Q: %d",
            can_general_config.rx_io, can_general_config.tx_io,
            can_general_config.rx_queue_len, can_general_config.tx_queue_len);

        ESP_ERROR_CHECK(can_driver_install(
            &can_general_config, &can_timing_config, &can_filter_config));

        xTaskCreatePinnedToCore(rx_task, "ESP32-CAN RX", OPENMRN_STACK_SIZE,
            this, RX_TASK_PRIORITY, &rxTaskHandle_, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(tx_task, "ESP32-CAN TX", OPENMRN_STACK_SIZE,
            this, TX_TASK_PRIORITY, &txTaskHandle_, tskNO_AFFINITY);
    }

    ~Esp32HardwareCan()
    {
    }

    /// Enables the ESP32 CAN driver
    virtual void enable()
    {
        ESP_ERROR_CHECK(can_start());
        LOG(VERBOSE, "ESP32-CAN driver enabled");
    }

    /// Disables the ESP32 CAN driver
    virtual void disable()
    {
        ESP_ERROR_CHECK(can_stop());
        LOG(VERBOSE, "ESP32-CAN driver disabled");
    }

protected:
    /// function to try and transmit a message
    void tx_msg() override
    {
        // wake up the tx_task so it can consume any can_frames from txBuf.
        xTaskNotifyGive(txTaskHandle_);
    }

private:
    /// Default constructor.
    Esp32HardwareCan();

    /// Enables/Disables the periodic reporting of CAN bus statistics to the
    /// default serial stream.
    bool reportStats_ : 1;
    /// Set to true if the 'frame dropped' warning is printed.
    bool overrunWarningPrinted_ : 1;

    /// Handle for the tx_task that converts and transmits can_frame to the
    /// native can driver.
    TaskHandle_t txTaskHandle_;

    /// Handle for the rx_task that receives and converts the native can driver
    /// frames to can_frame.
    TaskHandle_t rxTaskHandle_;

    /// Interval at which to print the ESP32 CAN bus status.
    static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

    /// Interval to wait between iterations when the bus is recovering, a
    /// transmit failure or there is nothing to transmit.
    static constexpr TickType_t TX_DEFAULT_DELAY = pdMS_TO_TICKS(250);

    /// Priority to use for the rx_task. This needs to be higher than the
    /// tx_task and lower than @ref OPENMRN_TASK_PRIORITY.
    static constexpr UBaseType_t RX_TASK_PRIORITY = ESP_TASK_TCPIP_PRIO - 1;

    /// Priority to use for the tx_task. This should be lower than
    /// @ref RX_TASK_PRIORITY and @ref OPENMRN_TASK_PRIORITY.
    static constexpr UBaseType_t TX_TASK_PRIORITY = ESP_TASK_TCPIP_PRIO - 2;

    /// Background task that takes care of the conversion of the @ref can_frame
    /// provided by the @ref txBuf into an ESP32 can_message_t which can be
    /// processed by the native CAN driver. This task also covers the periodic
    /// status reporting and BUS recovery when necessary.
    static void tx_task(void *can)
    {
        /// Get handle to our parent Esp32HardwareCan object to access the
        /// txBuf.
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);

        // Add this task to the WDT
        esp_task_wdt_add(parent->txTaskHandle_);

        /// Tracks the last time that we displayed the CAN driver status.
        TickType_t next_status_display_tick_count = 0;

        while (true)
        {
            // Feed the watchdog so it doesn't reset the ESP32
            esp_task_wdt_reset();

            // periodic CAN driver monitoring and reporting, this takes care of
            // bus recovery when the CAN driver disables the bus due to error
            // conditions exceeding thresholds.
            can_status_info_t status;
            can_get_status_info(&status);
            auto current_tick_count = xTaskGetTickCount();
            if (next_status_display_tick_count == 0 ||
                current_tick_count >= next_status_display_tick_count)
            {
                next_status_display_tick_count =
                    current_tick_count + STATUS_PRINT_INTERVAL;
                if (parent->reportStats_)
                {
                    LOG(INFO,
                        "ESP32-CAN: %s rx-q:%d, tx-q:%d, rx-err:%d, tx-err:%d, "
                        "ovr:%d arb-lost:%d, bus-err:%d, state: %s",
                        parent->overrunWarningPrinted_ ? "!!OVERRUN!! " : "",
                        status.msgs_to_rx, status.msgs_to_tx,
                        status.rx_error_counter, status.tx_error_counter,
                        parent->overrunCount, status.arb_lost_count,
                        status.bus_error_count,
                        ESP32_CAN_STATUS_STRINGS[status.state]);
                }
                parent->overrunWarningPrinted_ = false;
            }
            if (status.state == CAN_STATE_BUS_OFF)
            {
                // When the bus is OFF we need to initiate recovery, transmit is
                // not possible when in this state.
                LOG(WARNING, "ESP32-CAN: initiating recovery");
                can_initiate_recovery();
                continue;
            }
            else if (status.state == CAN_STATE_RECOVERING)
            {
                // when the bus is in recovery mode transmit is not possible.
                vTaskDelay(TX_DEFAULT_DELAY);
                continue;
            }

            // check txBuf for any message to transmit.
            unsigned count;
            struct can_frame *can_frame = nullptr;
            {
                AtomicHolder h(parent);
                count = parent->txBuf->data_read_pointer(&can_frame);
            }
            if (!count || !can_frame)
            {
                // tx Buf empty; wait for tx_msg to be called.
                ulTaskNotifyTake(pdTRUE, // clear on exit
                    TX_DEFAULT_DELAY);
                continue;
            }

            /// ESP32 native CAN driver frame
            can_message_t msg = {0};

            msg.flags = CAN_MSG_FLAG_NONE;
            msg.identifier = can_frame->can_id;
            msg.data_length_code = can_frame->can_dlc;
            for (int i = 0; i < can_frame->can_dlc; i++)
            {
                msg.data[i] = can_frame->data[i];
            }
            if (IS_CAN_FRAME_EFF(*can_frame))
            {
                msg.flags |= CAN_MSG_FLAG_EXTD;
            }
            if (IS_CAN_FRAME_RTR(*can_frame))
            {
                msg.flags |= CAN_MSG_FLAG_RTR;
            }

            // Pass the converted CAN frame to the native driver
            // for transmit, if the TX queue is full this will
            // return ESP_ERR_TIMEOUT which will result in the
            // the message being left in txBuf for the next iteration.
            // if this call returns ESP_OK we consider the frame as
            // transmitted by the driver and remove it from txBuf.
            esp_err_t tx_res = can_transmit(&msg, pdMS_TO_TICKS(100));
            if (tx_res == ESP_OK)
            {
                LOG(VERBOSE,
                    "ESP32-CAN-TX OK id:%08x, flags:%04x, dlc:%02d, "
                    "data:%02x%02x%02x%02x%02x%02x%02x%02x",
                    msg.identifier, msg.flags, msg.data_length_code,
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                    msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                AtomicHolder h(parent);
                parent->txBuf->consume(1);
                parent->txBuf->signal_condition();
            }
            else if (tx_res != ESP_ERR_TIMEOUT)
            {
                LOG(WARNING, "ESP32-CAN-TX: %s", esp_err_to_name(tx_res));
                vTaskDelay(TX_DEFAULT_DELAY);
            }
        } // loop on task
    }

    /// Background task that takes care of receiving can_message_t objects from
    /// the ESP32 native CAN driver, when they are available, converting them to
    /// a @ref can_frame and pushing them to the @ref rxBuf.
    static void rx_task(void *can)
    {
        /// Get handle to our parent Esp32HardwareCan object to access the rxBuf
        Esp32HardwareCan *parent = reinterpret_cast<Esp32HardwareCan *>(can);

        // Add this task to the WDT
        esp_task_wdt_add(parent->rxTaskHandle_);

        while (true)
        {
            // Feed the watchdog so it doesn't reset the ESP32
            esp_task_wdt_reset();

            /// ESP32 native CAN driver frame
            can_message_t msg = {0};
            if (can_receive(&msg, pdMS_TO_TICKS(250)) != ESP_OK)
            {
                // native CAN driver did not give us a frame.
                continue;
            }
            // we have received a frame from the native CAN driver, verify if
            // it is a standard frame, if not we drop it.
            if (msg.flags & CAN_MSG_FLAG_DLC_NON_COMP)
            {
                LOG(WARNING,
                    "ESP32-CAN-RX: received non-compliant CAN frame, frame "
                    "dropped!");
                continue;
            }
            LOG(VERBOSE,
                "ESP32-CAN-RX id:%08x, flags:%04x, dlc:%02d, "
                "data:%02x%02x%02x%02x%02x%02x%02x%02x",
                msg.identifier, msg.flags, msg.data_length_code,
                msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
            AtomicHolder h(parent);
            struct can_frame *can_frame = nullptr;
            // verify if we have space in the rxBuf, if not drop the frame and
            // record the overrun.
            if (!parent->rxBuf->data_write_pointer(&can_frame) ||
                can_frame == nullptr)
            {
                if (!parent->overrunWarningPrinted_)
                {
                    parent->overrunWarningPrinted_ = true;
                    LOG(WARNING,
                        "ESP32-CAN-RX: buffer overrun, frame dropped!");
                }
                parent->overrunCount++;
                continue;
            }
            // we have space in the rxBuf, start conversion
            LOG(VERBOSE, "ESP32-CAN-RX: converting to can_frame");
            memset(can_frame, 0, sizeof(struct can_frame));
            can_frame->can_id = msg.identifier;
            can_frame->can_dlc = msg.data_length_code;
            for (int i = 0; i < msg.data_length_code; i++)
            {
                can_frame->data[i] = msg.data[i];
            }
            if (msg.flags & CAN_MSG_FLAG_EXTD)
            {
                SET_CAN_FRAME_EFF(*can_frame);
            }
            if (msg.flags & CAN_MSG_FLAG_RTR)
            {
                SET_CAN_FRAME_RTR(*can_frame);
            }
            parent->rxBuf->advance(1);
            parent->rxBuf->signal_condition();
        }
    }
    DISALLOW_COPY_AND_ASSIGN(Esp32HardwareCan);
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32HardwareCan;

#endif /* _FREERTOS_DRIVERS_ARDUINO_ESP32HWCAN_HXX_ */
