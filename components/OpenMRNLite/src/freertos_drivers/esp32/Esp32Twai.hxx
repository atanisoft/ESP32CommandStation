/** \copyright
 * Copyright (c) 2020, Mike Dunston
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
 * \file Esp32Twai.hxx
 *
 * ESP32 Hardware TWAI adapter code. This utilizes the built in TWAI controller
 * to translate the can_frame in OpenMRN to the ESP32 twai_message_t used by 
 * the ESP-IDF TWAI controller code. The ESP32 will still require an external
 * TWAI transceiver (MCP2551 or SN65HVD230 as example).
 *
 * @author Mike Dunston
 * @date 20 September 2020
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32TWAI_HXX_

// Target platform check via the sdkconfig.h file which is only present on the
// ESP32 platform.
#if defined __has_include
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif // defined __has_include

// Only define the class if we are compiling for a supported platform
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32S3)

#if defined __has_include
#if __has_include(<soc/soc_caps.h>)
// This needs to be included before other includes so that SOC_TWAI_SUPPORTED
// is defined early.
#include <soc/soc_caps.h>
#endif
#endif // defined __has_include

#include <algorithm>
#include <atomic>
#include <cstring>
#include <driver/gpio.h>
#if SOC_TWAI_SUPPORTED
#include <driver/twai.h>
#else
#include <driver/can.h>
#define twai_driver_install can_driver_install
#define twai_driver_uninstall can_driver_uninstall
#define twai_start can_start
#define twai_stop can_stop
#define twai_transmit can_transmit
#define twai_receive can_receive
#define twai_initiate_recovery can_initiate_recovery
#define twai_get_status_info can_get_status_info
#define TWAI_GENERAL_CONFIG_DEFAULT CAN_GENERAL_CONFIG_DEFAULT
#define TWAI_TIMING_CONFIG_125KBITS CAN_TIMING_CONFIG_125KBITS
#define TWAI_FILTER_CONFIG_ACCEPT_ALL CAN_FILTER_CONFIG_ACCEPT_ALL
#define twai_timing_config_t can_timing_config_t
#define twai_filter_config_t can_filter_config_t
#define twai_general_config_t can_general_config_t
#define twai_status_info_t can_status_info_t
#define twai_message_t can_message_t
#define TWAI_MODE_NORMAL CAN_MODE_NORMAL
#define TWAI_STATE_STOPPED CAN_STATE_STOPPED
#define TWAI_STATE_RUNNING CAN_STATE_RUNNING
#define TWAI_STATE_BUS_OFF CAN_STATE_BUS_OFF
#define TWAI_STATE_RECOVERING CAN_STATE_RECOVERING
#define TWAI_MSG_FLAG_DLC_NON_COMP CAN_MSG_FLAG_DLC_NON_COMP
#define TWAI_MSG_FLAG_EXTD CAN_MSG_FLAG_EXTD
#define TWAI_MSG_FLAG_RTR CAN_MSG_FLAG_RTR
#define TWAI_MSG_FLAG_NONE CAN_MSG_FLAG_NONE
#endif // SOC_TWAI_SUPPORTED
#include <esp_log.h>
#include <esp_vfs.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <mutex>

#include "can_frame.h"
#include "freertos_drivers/arduino/DeviceBuffer.hxx"
#include "nmranet_config.h"
#include "utils/logging.h"
#include "utils/Singleton.hxx"

//#define USE_PORT_MUX 1
//#define USE_ESP_TIMER 1

#if USE_PORT_MUX
#define MUTEX_INIT(mux) {mux = portMUX_INITIALIZER_UNLOCKED}
#define MUTEX_TYPE portMUX_TYPE
#define LOCK_BUFFER() vPortEnterCritical(&mux_);
#define UNLOCK_BUFFER() vPortExitCritical(&mux_);
#else
#define MUTEX_INIT(mux)
#define MUTEX_TYPE std::mutex
#define LOCK_BUFFER() const std::lock_guard<std::mutex> lock(mux_);
#define UNLOCK_BUFFER()
#endif // USE_PORT_MUX

#if USE_ESP_TIMER
#define TIMER_ARG_TYPE void *
#define TIMER_HANDLE esp_timer_handle_t
#define TIMER_START(timer, interval, max_wait) esp_timer_start_once(timer, interval)
#define TIMER_RESET(timer, interval, max_wait) esp_timer_start_once(timer, interval)
#define TIMER_STOP(timer, max_wait) esp_timer_stop(timer)
#define TIMER_DELETE(timer, max_wait) esp_timer_delete(timer)
#else
#define TIMER_ARG_TYPE xTimerHandle
#define TIMER_HANDLE xTimerHandle
#define TIMER_START(timer, interval, max_wait) xTimerStart(timer, max_wait)
#define TIMER_RESET(timer, interval, max_wait) xTimerReset(timer, max_wait)
#define TIMER_STOP(timer, max_wait) {       \
if (xTimerIsTimerActive(timer) == pdTRUE)   \
{                                           \
    xTimerStop(timer, max_wait);            \
}}
#define TIMER_DELETE(timer, max_wait) xTimerDelete(timer, max_wait)
#endif // USE_ESP_TIMER

namespace openmrn_arduino 
{

// VFS adapter for write().
ssize_t twai_vfs_write(int fd, const void *buf, size_t size);

// VFS adapter for read().
ssize_t twai_vfs_read(int fd, void *buf, size_t size);

// VFS adapter for open().
int twai_vfs_open(const char * path, int flags, int mode);

// VFS adapter for close().
int twai_vfs_close(int fd);

// VFS adapter for fcntl().
int twai_vfs_fcntl(int fd, int cmd, int arg);

// VFS adapter for ioctl().
int twai_vfs_ioctl(int fd, int cmd, va_list args);

// VFS adapter for select().
esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds, fd_set *writefds
                              , fd_set *exceptfds, esp_vfs_select_sem_t sem
                              , void **end_select_args);

// VFS adapter for select().
esp_err_t twai_vfs_end_select(void *end_select_args);

// Periodic timer tick callback.
void twai_timer_tick(TimerHandle_t xTimer);

/// ESP32 TWAI Driver adapter.
///
/// Exposes a VFS device interface that can be used for async TWAI RX/TX
/// operations.
///
/// NOTE: Only one instance of this class is supported since there is only one
/// built-in TWAI controller.
class Esp32Twai : public Singleton<Esp32Twai>
{
public:
    /// Constructor.
    ///
    /// @param vfsPath is the VFS path to register the TWAI driver to.
    /// @param rxPin is the pin connected to the external transceiver RX pin.
    /// @param txPin is the pin connected to the external transceiver TX pin.
    Esp32Twai(const char *vfsPath, gpio_num_t rxPin, gpio_num_t txPin
            , bool reportStats = true)
            : vfsPath_(vfsPath)
            , rxPin_(rxPin)
            , txPin_(txPin)
            , txBuf_(DeviceBuffer<struct can_frame>::create(
                config_can_tx_buffer_size(), config_can_tx_buffer_size() / 2))
            , rxBuf_(DeviceBuffer<struct can_frame>::create(
                config_can_rx_buffer_size()))
            , reportStats_(reportStats)
            , vfsRegistered_(0)
            , twaiInitialized_(0)
    {
        HASSERT(GPIO_IS_VALID_GPIO(rxPin_));
        HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(txPin_));
        MUTEX_INIT(mux_);
    }

    /// Destructor.
    ///
    /// Cleans up resources allocated by the TWAI driver.
    ~Esp32Twai()
    {
        if (vfsRegistered_)
        {
            ESP_ERROR_CHECK(esp_vfs_unregister(vfsPath_));
        }
        if (twaiInitialized_)
        {
            TIMER_DELETE(timer_, TWAI_TIMER_MAX_WAIT);
            ESP_ERROR_CHECK(twai_driver_uninstall());
        }
        txBuf_->destroy();
        rxBuf_->destroy();
    }

    /// Initializes the VFS adapter and TWAI driver
    void hw_init()
    {
        init_vfs();
        init_twai_driver();
    }

    /// Enables the TWAI driver and starts the periodic timer.
    void enable()
    {
        LOG(INFO, "[TWAI] Starting TWAI low-level driver");
        ESP_ERROR_CHECK_WITHOUT_ABORT(twai_start());
        LOG(INFO, "[TWAI] Starting periodic timer");
        TIMER_START(timer_, TWAI_TIMER_TICK_INTERVAL, TWAI_TIMER_MAX_WAIT);
    }

    /// Disables the TWAI driver and periodic timer (if active).
    void disable()
    {
        LOG(INFO, "[TWAI] Stopping periodic timer");
        TIMER_STOP(timer_, TWAI_TIMER_MAX_WAIT);
        LOG(INFO, "[TWAI] Stopping TWAI low-level driver");
        ESP_ERROR_CHECK_WITHOUT_ABORT(twai_stop());
    }

    /// VFS interface helper for write()
    ///
    /// @param fd is the file descriptor being written to.
    /// @param buf is the buffer containing the data to be written.
    /// @param size is the size of the buffer.
    /// @return number of bytes written or -1 if there is the write would be a
    /// blocking operation.
    ssize_t vfs_write(int fd, const void *buf, size_t size)
    {
        LOG(VERBOSE, "[TWAI] vfs_write(%d, %p, %zu)", fd, buf, size);
        ssize_t result = 0;
        const struct can_frame *data = (const struct can_frame *)buf;
        size /= sizeof(struct can_frame);
        while (size)
        {
            size_t frames_written = 0;
            {
                UNLOCK_BUFFER()
                frames_written = txBuf_->put(data, size < 8 ? size : 8);
                UNLOCK_BUFFER()
            }
            if (frames_written == 0)
            {
                break;
            }
            size -= frames_written;
            result += frames_written;
            data += frames_written;
        }
        if (!result)
        {
            errno = EWOULDBLOCK;
            return -1;
        }
        LOG(VERBOSE, "[TWAI] vfs_write ret: %zu", result * sizeof(struct can_frame));
        return result * sizeof(struct can_frame);
    }

    /// VFS interface helper for read()
    ///
    /// @param fd is the file descriptor being read from.
    /// @param buf is the buffer to write into.
    /// @param size is the size of the buffer.
    /// @return number of bytes read or -1 if there is the read would be a
    /// blocking operation.
    ssize_t vfs_read(int fd, void *buf, size_t size)
    {
        LOG(VERBOSE, "[TWAI] vfs_read(%d, %p, %zu)", fd, buf, size);
        ssize_t result = 0;
        struct can_frame *data = (struct can_frame *)buf;
        size /= sizeof(struct can_frame);
        while (size)
        {
            size_t frames_read = 0;
            {
                LOCK_BUFFER()
                frames_read = rxBuf_->get(data, size < 8 ? size : 8);
                UNLOCK_BUFFER()
            }
            if (frames_read == 0)
            {
                break;
            }
            size -= frames_read;
            result += frames_read;
            data += frames_read;
        }
        if (!result)
        {
            errno = EWOULDBLOCK;
            return -1;
        }
        LOG(VERBOSE, "[TWAI] vfs_read ret: %zu", result * sizeof(struct can_frame));
        return result * sizeof(struct can_frame);
    }

    /// VFS interface helper for select()
    ///
    /// @param nfds is the number of FDs being checked.
    /// @param readfds is the set of FDs being checked for ready to read.
    /// @param writefds is the set of FDs being checked for ready to write.
    /// @param exceptfds is the set of FDs being checked for exception.
    /// @param sem is the semaphore to use for waking up the select() call.
    /// @param end_select_args is any arguments to pass into vfs_end_select().
    /// @return ESP_OK for success.
    esp_err_t vfs_start_select(int nfds, fd_set *readfds, fd_set *writefds
                             , fd_set *exceptfds, esp_vfs_select_sem_t sem
                             , void **end_select_args)
    {
        size_t tx_space = 0;
        size_t rx_ready = 0;
        {
            LOCK_BUFFER()
            tx_space = txBuf_->space();
            rx_ready = rxBuf_->pending();
            UNLOCK_BUFFER()
        }
        *end_select_args = nullptr;
        // check if we can/should wake up now.
        if (tx_space || rx_ready)
        {
            LOG(VERBOSE
              , "[TWAI] invoking esp_vfs_select_triggered as there is at least "
                "one frame available in the RX/TX buffers");
            esp_vfs_select_triggered(sem);
        }
        else
        {
            LOCK_BUFFER()
            selectSem_ = sem;
            selectPending_ = true;
            UNLOCK_BUFFER()
        }
        return ESP_OK;
    }

    /// VFS interface helper invoked when select() is woken up.
    ///
    /// @param end_select_args is any arguments provided in vfs_start_select().
    /// @return ESP_OK for success.
    esp_err_t vfs_end_select(void *end_select_args)
    {
        LOCK_BUFFER()
        selectPending_ = false;
        UNLOCK_BUFFER()
        return ESP_OK;
    }

    /// Periodic callback used to interact with the low-level TWAI driver.
    void timer_tick()
    {
        size_t tx_rx_count = 0;
        twai_status_info_t status;
        size_t rx_space = 0;
        size_t tx_ready = 0;
        {
            LOCK_BUFFER()
            rx_space = rxBuf_->space();
            tx_ready = txBuf_->pending();
            UNLOCK_BUFFER()
        }
        ESP_ERROR_CHECK(twai_get_status_info(&status));
        if (status.state == TWAI_STATE_BUS_OFF)
        {
            LOG(INFO, "[TWAI] Bus is off, initiating recovery.");
            ESP_ERROR_CHECK(twai_initiate_recovery());
            busOffCount_++;
        }
        else if (status.state == TWAI_STATE_RECOVERING)
        {
            LOG(INFO, "[TWAI] Bus is recovering from an off condition.");
        }
        else if (status.state == TWAI_STATE_STOPPED)
        {
            LOG(WARNING, "[TWAI] Bus is in a stopped state!");
            ESP_ERROR_CHECK(twai_start());
        }
        else
        {
            if (status.msgs_to_rx)
            {
                struct can_frame can_frame;
                if (rx_space)
                {
                    // calculate the number of frames we can receive based on
                    // the available RX buffer space and the number of frames
                    // in the RX queue.
                    size_t to_rx =
                        std::min((size_t)status.msgs_to_rx, rx_space);
                    LOG(VERBOSE, "[TWAI] RX Q:%zu, ll-Q:%d/%d, to_rx:%zu"
                      , rx_space, status.msgs_to_rx
                      , config_can_rx_buffer_size(), to_rx);
                    for (size_t idx = 0; idx < to_rx; ++idx)
                    {
                        twai_message_t msg;
                        memset(&msg, 0, sizeof(twai_message_t));
                        esp_err_t res = twai_receive(&msg, MAX_TWAI_RX_TIME);
                        if (res != ESP_OK)
                        {
                            if (res != ESP_ERR_TIMEOUT)
                            {
                                LOG_ERROR("[TWAI] RX error: %s (%d)"
                                        , esp_err_to_name(res), res);
                            }
                            break;
                        }
                        // we have received a frame from the native CAN driver,
                        // verify if it is a standard frame, if not we drop it.
                        if (msg.flags & TWAI_MSG_FLAG_DLC_NON_COMP)
                        {
                            LOG(WARNING
                              , "[TWAI] dropping non-compliant frame!");
                            continue;
                        }
                        LOG(VERBOSE
                          , "[TWAI] id:%08x, flags:%04x, dlc:%02d, "
                            "data:%02x%02x%02x%02x%02x%02x%02x%02x"
                          , msg.identifier, msg.flags, msg.data_length_code
                          , msg.data[0], msg.data[1], msg.data[2], msg.data[3]
                          , msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                        memset(&can_frame, 0, sizeof(struct can_frame));
                        can_frame.can_id = msg.identifier;
                        can_frame.can_dlc = msg.data_length_code;
                        memcpy(can_frame.data, msg.data, msg.data_length_code);
                        if (msg.flags & TWAI_MSG_FLAG_EXTD)
                        {
                            SET_CAN_FRAME_EFF(can_frame);
                        }
                        if (msg.flags & TWAI_MSG_FLAG_RTR)
                        {
                            SET_CAN_FRAME_RTR(can_frame);
                        }
                        {
                            LOCK_BUFFER()
                            rxBuf_->put(&can_frame, 1);
                            UNLOCK_BUFFER()
                        }
                        numReceivedPackets_++;
                        tx_rx_count++;
                    }
                }
                else
                {
                    overrunCount_++;
                    if (!overrunWarningPrinted_)
                    {
                        overrunWarningPrinted_ = true;
                        LOG(WARNING
                          , "[TWAI] RX buffer overrun, frame dropped!");
                    }
                }
            }
            if (tx_ready &&
                status.msgs_to_tx < config_can_tx_buffer_size())
            {
                struct can_frame *can_frame = nullptr;
                size_t available_tx_space =
                    config_can_tx_buffer_size() - status.msgs_to_tx;
                size_t to_tx = std::min(available_tx_space, tx_ready);
                LOG(VERBOSE, "[TWAI] TX Q:%zu, ll-Q:%d/%d, to_tx:%zu"
                  , tx_ready, status.msgs_to_tx, config_can_tx_buffer_size()
                  , to_tx);
                for (size_t idx = 0; idx < to_tx; ++idx)
                {
                    {
                        LOCK_BUFFER()
                        txBuf_->data_read_pointer(&can_frame);
                        UNLOCK_BUFFER()
                    }
                    if (can_frame == nullptr)
                    {
                        LOG_ERROR("[TWAI] TX ERROR: can_frame is null!");
                        break;
                    }
                    twai_message_t msg;
                    memset(&msg, 0, sizeof(twai_message_t));
                    msg.flags = TWAI_MSG_FLAG_NONE;
                    msg.identifier = can_frame->can_id;
                    msg.data_length_code = can_frame->can_dlc;
                    memcpy(msg.data, can_frame->data, can_frame->can_dlc);
                    if (IS_CAN_FRAME_EFF(*can_frame))
                    {
                        msg.flags |= TWAI_MSG_FLAG_EXTD;
                    }
                    if (IS_CAN_FRAME_RTR(*can_frame))
                    {
                        msg.flags |= TWAI_MSG_FLAG_RTR;
                    }
                    esp_err_t res = twai_transmit(&msg, MAX_TWAI_TX_TIME);
                    if (res != ESP_OK)
                    {
                        if (res != ESP_ERR_TIMEOUT)
                        {
                            LOG_ERROR("[TWAI] TX error: %s (%d)"
                                    , esp_err_to_name(res), res);
                        }
                        // there was an error sending the frame to the driver,
                        // give up and retry on next interval.
                        break;
                    }
                    LOG(VERBOSE,
                        "[TWAI] OK id:%08x, flags:%04x, dlc:%02d, "
                        "data:%02x%02x%02x%02x%02x%02x%02x%02x",
                        msg.identifier, msg.flags, msg.data_length_code,
                        msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                        msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    {
                        LOCK_BUFFER()
                        txBuf_->consume(1);
                        UNLOCK_BUFFER()
                    }
                    numTransmittedPackets_++;
                    tx_rx_count++;
                }
            }
        }

        // if we have a select() pending and we transfered at least one frame
        // wake up the select() call.
        if (tx_rx_count)
        {
            LOCK_BUFFER()
            if (selectPending_)
            {
                LOG(VERBOSE
                  , "[TWAI] invoking esp_vfs_select_triggered as %zu frames "
                    "were transfered to/from the low-level driver"
                  , tx_rx_count);
                esp_vfs_select_triggered(selectSem_);
                selectPending_ = false;
            }
            UNLOCK_BUFFER()
        }

        if (reportStats_)
        {
            auto tick = xTaskGetTickCount();
            if (nextStatusDisplayTick_ == 0 || tick >= nextStatusDisplayTick_)
            {
                nextStatusDisplayTick_ = tick + STATUS_PRINT_INTERVAL;
                LOG(INFO
                  , "[TWAI] %sRX:%d (ll-q:%d,err:%d,overrun:%d,miss:%d) "
                    "TX:%d (ll-q:%d,err:%d,fail:%d) "
                    "bus (arb-err:%d,err:%d,off:%d,st:%s)"
                  , overrunWarningPrinted_ ? "!!OVERRUN!! " : ""
                  , numReceivedPackets_, status.msgs_to_rx
                  , status.rx_error_counter, overrunCount_
                  , status.rx_missed_count, numTransmittedPackets_
                  , status.msgs_to_tx, status.tx_error_counter
                  , status.tx_failed_count, status.arb_lost_count
                  , status.bus_error_count, busOffCount_
                  , status.state == TWAI_STATE_STOPPED ? "Stopped"
                  : status.state == TWAI_STATE_RUNNING ? "Running"
                  : status.state == TWAI_STATE_BUS_OFF ? "Bus-Off"
                  : "Recovery in progress");
            }
            overrunWarningPrinted_ = false;
        }
        TIMER_RESET(timer_, TWAI_TIMER_TICK_INTERVAL, TWAI_TIMER_MAX_WAIT);
    }
private:
    DISALLOW_COPY_AND_ASSIGN(Esp32Twai);

    /// VFS Mount point.
    const char *vfsPath_;

    /// GPIO pin connected to the external transceiver RX pin.
    const gpio_num_t rxPin_;

    /// GPIO pin connected to the external transceiver TX pin.
    const gpio_num_t txPin_;
    
    /// Buffer for can_frames pending transfer to the low level TWAI driver.
    DeviceBuffer<struct can_frame> *txBuf_;

    /// Buffer for can_frames available to be read into teh stack.
    DeviceBuffer<struct can_frame> *rxBuf_;

    /// Enables/Disables the periodic reporting of CAN bus statistics to the
    /// default serial stream.
    bool reportStats_{false};

    /// Internal flag used for tracking if the VFS driver has been registered.
    std::atomic_uint_least8_t vfsRegistered_;

    /// Internal flag used for tracking if the TWAI driver has been started.
    std::atomic_uint_least8_t twaiInitialized_;

    /// Internal flag used for tracking if @ref selectSem_ is valid.
    bool selectPending_{false};

    /// Set to true if the 'frame dropped' warning is printed.
    bool overrunWarningPrinted_{false};

    /// Tracks the last time that we displayed the TWAI driver status.
    TickType_t nextStatusDisplayTick_{0};

    /// Tracks the number of RX buffer overruns.
    uint32_t overrunCount_{0};

    /// Tracks the number of times the TWAI bus was detected as off.
    uint32_t busOffCount_{0};

    /// Tracks the number of packets that have been received from the TWAI bus.
    uint32_t numReceivedPackets_{0};

    /// Tracks the number of packets that have been sent to the TWAI bus.
    uint32_t numTransmittedPackets_{0};

    /// Temporary holder for the VFS select() semaphore.
    esp_vfs_select_sem_t selectSem_;

    /// Periodic timer handle.
    TIMER_HANDLE timer_;

    /// Mutex protecting txBuf_, rxBuf_ and selectSem_.
    MUTEX_TYPE mux_;

    /// Interval at which the TWAI timer will be invoked.
    static constexpr TickType_t TWAI_TIMER_TICK_INTERVAL = pdMS_TO_TICKS(5);

    /// Maximum time that a Timer API call can take before timeout.
    static constexpr TickType_t TWAI_TIMER_MAX_WAIT = pdMS_TO_TICKS(1);

    /// Maximum time to allow for receiving a frame from the TWAI driver.
    static constexpr TickType_t MAX_TWAI_RX_TIME = pdMS_TO_TICKS(5);

    /// Maximum time to allow for sending a frame to the TWAI driver.
    static constexpr TickType_t MAX_TWAI_TX_TIME = pdMS_TO_TICKS(5);

    /// Interval at which to print the ESP32 CAN bus status.
    static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

    /// Registers the VFS adapter for the TWAI driver.
    void init_vfs()
    {
        if (!vfsRegistered_)
        {
            esp_vfs_t vfs;
            memset(&vfs, 0, sizeof(esp_vfs_t));
            vfs.flags = ESP_VFS_FLAG_DEFAULT;
            vfs.write = twai_vfs_write;
            vfs.read = twai_vfs_read;
            vfs.open = twai_vfs_open;
            vfs.close = twai_vfs_close;
            vfs.fcntl = twai_vfs_fcntl;
            vfs.ioctl = twai_vfs_ioctl;
            vfs.start_select = twai_vfs_start_select;
            vfs.end_select = twai_vfs_end_select;
            ESP_ERROR_CHECK(esp_vfs_register(vfsPath_, &vfs, nullptr));
            vfsRegistered_ = true;
        }
    }

    /// Initializes the low-level TWAI driver.
    void init_twai_driver()
    {
        if (!twaiInitialized_)
        {
            twai_timing_config_t timingCfg = TWAI_TIMING_CONFIG_125KBITS();
            twai_filter_config_t filterCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
            twai_general_config_t generalCfg =
                TWAI_GENERAL_CONFIG_DEFAULT(txPin_, rxPin_, TWAI_MODE_NORMAL);
            generalCfg.rx_queue_len = config_can_rx_buffer_size();
            generalCfg.tx_queue_len = config_can_tx_buffer_size();
            ESP_ERROR_CHECK(
                twai_driver_install(&generalCfg, &timingCfg, &filterCfg));
            LOG(INFO, "[TWAI] using RX: %d, TX: %d, RX-Q: %d, TX-Q: %d"
              , rxPin_, txPin_, config_can_rx_buffer_size()
              , config_can_tx_buffer_size());
#if USE_ESP_TIMER
            esp_timer_create_args_t timer_args =
            {
                .callback = twai_timer_tick                 /* callback     */
              , .arg = nullptr                              /* callback arg */
              , .dispatch_method = ESP_TIMER_TASK           /* timer mode   */
              , .name = vfsPath_                            /* timer name   */
            };
            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_));
#else
            timer_ = xTimerCreate(vfsPath_                  /* timer name  */
                                , TWAI_TIMER_TICK_INTERVAL  /* interval    */
                                , pdFALSE                   /* auto reload */
                                , nullptr                   /* timer id    */
                                , twai_timer_tick);         /* callback    */
#endif
            HASSERT(timer_ != nullptr);
            twaiInitialized_ = true;
        }
    }
};

/// VFS adapter for write(fd, buf, size).
///
/// @param fd is the file descriptor being written to.
/// @param buf is the buffer being written.
/// @param size is the size of the buffer.
///
/// @return number of bytes written upon success, -1 upon failure with errno
/// containing the cause.
ssize_t twai_vfs_write(int fd, const void *buf, size_t size)
{
    return Singleton<Esp32Twai>::instance()->vfs_write(fd, buf, size);
}

/// VFS adapter for read(fd, buf, size).
///
/// @param fd is the file descriptor being read fromo.
/// @param buf is the buffer being read into.
/// @param size is the size of the buffer.
///
/// @return number of bytes read upon success, -1 upon failure with errno
/// containing the cause.
ssize_t twai_vfs_read(int fd, void *buf, size_t size)
{
    return Singleton<Esp32Twai>::instance()->vfs_read(fd, buf, size);
}

/// VFS adapter for open(path, flags, mode).
///
/// @param path is the path to the file being opened.
/// @param flags are the flags to use for opened file.
/// @param mode is the mode to use for the opened file.
///
/// When this method is invoked it will enable the TWAI driver and start the
/// periodic timer used for RX/TX of frame data.
///
/// @return 0 upon success, -1 upon failure with errno containing the cause.
int twai_vfs_open(const char * path, int flags, int mode)
{
    LOG(INFO, "[TWAI] Opening TWAI path:%s", path);
    Singleton<Esp32Twai>::instance()->enable();
    return 0;
}

/// VFS adapter for close(fd).
///
/// @param path is the path to the file being opened.
/// @param flags are the flags to use for opened file.
/// @param mode is the mode to use for the opened file.
///
/// When this method is invoked it will disable the TWAI driver and stop the
/// periodic timer used for RX/TX of frame data if it is running.
///
/// @return zero upon success, negative value with errno for failure.
int twai_vfs_close(int fd)
{
    LOG(INFO, "[TWAI] Closing TWAI fd:%d", fd);
    Singleton<Esp32Twai>::instance()->disable();
    return 0;
}

/// VFS adapter for fcntl(fd, cmd, arg).
///
/// @param fd to operate on.
/// @param cmd to be executed.
/// @param arg arg to be used for the operation.
///
/// This method is currently a NO-OP.
///
/// @return zero upon success, negative value with errno for failure.
int twai_vfs_fcntl(int fd, int cmd, int arg)
{
    return 0;
}

/// VFS adapter for ioctl(fd, cmd, args).
///
/// @param fd to operate on.
/// @param cmd to be executed.
/// @param args args to be used for the operation.
///
/// This method is currently a NO-OP.
///
/// @return zero upon success, negative value with errno for failure.
int twai_vfs_ioctl(int fd, int cmd, va_list args)
{
    return 0;
}

/// VFS adapter called when select() is invoked.
///
/// @param nfds number of FDs to operate on.
/// @param readfds FDs to check if they can be read from without blocking.
/// @param writefds FDs to check if they can be written to without blocking.
/// @param exceptfds FDs to check for errors.
/// @param sem ESP VFS semaphore that can be used to wake up select() early.
/// @param end_select_args any arguments to pass into vfs_end_select().
///
/// @return ESP_OK upon success, ESP_FAIL on error.
esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds, fd_set *writefds
                              , fd_set *exceptfds, esp_vfs_select_sem_t sem
                              , void **end_select_args)
{
    return Singleton<Esp32Twai>::instance()->vfs_start_select(nfds, readfds
                                                            , writefds
                                                            , exceptfds, sem
                                                            , end_select_args);
}

/// VFS adapter called when select() has returned or was interrupted.
///
/// @param end_select_args are the arguments provided to the VFS layer when
/// twai_vfs_start_select was invoked.
///
/// @return ESP_OK upon success, ESP_FAIL on error.
esp_err_t twai_vfs_end_select(void *end_select_args)
{
    return Singleton<Esp32Twai>::instance()->vfs_end_select(end_select_args);
}

/// Periodic timer tick callback.
///
/// @param arg Argument passed into the periodic timer (not used).
void twai_timer_tick(TIMER_ARG_TYPE arg)
{
    Singleton<Esp32Twai>::instance()->timer_tick();
}

} // namespace openmrn_arduino

using openmrn_arduino::Esp32Twai;

#endif // CONFIG_IDF_TARGET:ESP32, ESP32S2, ESP32S3

#endif // _FREERTOS_DRIVERS_ESP32_ESP32TWAI_HXX_