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
 * \file Esp32HardwareTwai.hxx
 *
 * TWAI driver implementation for OpenMRN. This leverages a clone the ESP-IDF
 * HAL API rather than the TWAI driver to allow for faster access to received
 * frames and support for ESP-IDF v4.0 and above.
 *
 * @author Mike Dunston
 * @date 5 December 2020
 */
#ifndef _FREERTOS_DRIVERS_ESP32_ESP32HARDWARETWAI_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32HARDWARETWAI_HXX_

#ifdef ESP32

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <esp_vfs.h>
#include <fcntl.h>

#include "freertos_drivers/arduino/DeviceBuffer.hxx"
#include "freertos_drivers/esp32/Esp32TwaiHal.hxx"
#include "can_frame.h"
#include "can_ioctl.h"
#include "nmranet_config.h"
#include "executor/Notifiable.hxx"
#include "utils/Atomic.hxx"
#include "utils/logging.h"
#include "utils/Singleton.hxx"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
/// Data type used internally by ESP-IDF select() calls.
typedef SemaphoreHandle_t * esp_vfs_select_sem_t;
#endif // IDF < 4.0

static ssize_t twai_vfs_write(int fd, const void *buf, size_t size);
static ssize_t twai_vfs_read(int fd, void *buf, size_t size);
static int twai_vfs_open(const char *path, int flags, int mode);
static int twai_vfs_close(int fd);
static int twai_vfs_ioctl(int fd, int cmd, va_list args);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
static int twai_vfs_fcntl(int fd, int cmd, int arg);
static esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds
                                     , fd_set *writefds, fd_set *exceptfds
                                     , esp_vfs_select_sem_t sem
                                     , void **end_select_args);
static esp_err_t twai_vfs_end_select(void *end_select_args);
#else
static int twai_vfs_fcntl(int fd, int cmd, va_list arg);
static esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds
                                     , fd_set *writefds, fd_set *exceptfds
                                     , esp_vfs_select_sem_t sem);
static void twai_vfs_end_select();
#endif

static void twai_isr(void *arg);

class Esp32HardwareTwai : public Singleton<Esp32HardwareTwai>
{
public:
    Esp32HardwareTwai(int rx = GPIO_NUM_4, int tx = GPIO_NUM_5
                    , size_t rx_buffer_size = config_can_rx_buffer_size()
                    , size_t tx_buffer_size = config_can_tx_buffer_size()
                    , bool report = true, const char *path = "/dev/twai")
                    : rxPin_((gpio_num_t)rx)
                    , txPin_((gpio_num_t)tx)
                    , reportStats_(report)
                    , vfsPath_(path)
    {
        HASSERT(GPIO_IS_VALID_GPIO(rxPin_));
        HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(txPin_));

        rxBuf_ = DeviceBuffer<struct can_frame>::create(rx_buffer_size);
        HASSERT(rxBuf_ != nullptr);

        txBuf_ = DeviceBuffer<struct can_frame>::create(tx_buffer_size
                                                      , tx_buffer_size / 2);
        HASSERT(txBuf_ != nullptr);
    }

    void hw_init()
    {
        LOG(VERBOSE, "[TWAI] Configuring TWAI TX pin: %d", txPin_);
        gpio_set_pull_mode(txPin_, GPIO_FLOATING);
        gpio_matrix_out(txPin_, TWAI_TX_SIGNAL_IDX, false, false);
        gpio_pad_select_gpio(txPin_);

        LOG(VERBOSE, "[TWAI] Configuring TWAI RX pin: %d", rxPin_);
        gpio_set_pull_mode(rxPin_, GPIO_FLOATING);
        gpio_set_direction(rxPin_, GPIO_MODE_INPUT);
        gpio_matrix_in(rxPin_, TWAI_RX_SIGNAL_IDX, false);
        gpio_pad_select_gpio(rxPin_);

        esp_vfs_t vfs;
        memset(&vfs, 0, sizeof(esp_vfs_t));
        vfs.write = twai_vfs_write;
        vfs.read = twai_vfs_read;
        vfs.open = twai_vfs_open;
        vfs.close = twai_vfs_close;
        vfs.fcntl = twai_vfs_fcntl;
        vfs.ioctl = twai_vfs_ioctl;
        vfs.start_select = twai_vfs_start_select;
        vfs.end_select = twai_vfs_end_select;
        vfs.flags = ESP_VFS_FLAG_DEFAULT;
        ESP_ERROR_CHECK(esp_vfs_register(vfsPath_, &vfs, this));
        vfsRegistered_ = true;

        periph_module_reset(TWAI_PERIPHERAL);
        periph_module_enable(TWAI_PERIPHERAL);
        HASSERT(twai_hal_init(&twaiContext_));
        twai_timing_config_t timingCfg = TWAI_TIMING_CONFIG_125KBITS();
        twai_filter_config_t filterCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        LOG(VERBOSE, "[TWAI] Initiailizing TWAI peripheral");
        twai_hal_configure(&twaiContext_, &timingCfg, &filterCfg
                        , TWAI_DEFAULT_INTERRUPTS, 0);
        LOG(VERBOSE, "[TWAI] Allocating TWAI ISR");
        ESP_ERROR_CHECK(
            esp_intr_alloc(TWAI_ISR_SOURCE, TWAI_ISR_FLAGS, twai_isr, this
                        , &twaiISRHandle_));
        twaiConfigured_ = true;

        if (reportStats_)
        {
            os_thread_create(nullptr, "TWAI-STAT", 0, 0
                           , Esp32HardwareTwai::twai_stats_entry, this);
        }
    }

    ~Esp32HardwareTwai()
    {
        if (twaiConfigured_)
        {
            ESP_ERROR_CHECK(esp_intr_free(twaiISRHandle_));
            twai_hal_deinit(&twaiContext_);
        }

        if (vfsRegistered_)
        {
            ESP_ERROR_CHECK(esp_vfs_unregister(vfsPath_));
        }

        txBuf_->destroy();
        rxBuf_->destroy();
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
    int open(const char *path, int flags, int mode)
    {
        // skip past the '/' that is passed in as first character
        path++;
        nonBlocking_ = (flags & O_NONBLOCK);

        LOG(INFO, "[TWAI] Enabling TWAI device:%s mode:%x (%s)", path, mode
          , nonBlocking_ ? "non-blocking" : "blocking");
        {
            AtomicHolder h(&bufLock_);
            txBuf_->flush();
            rxBuf_->flush();
        }

        twai_hal_start(&twaiContext_, TWAI_MODE_NORMAL);
        return TWAI_VFS_FD;
    }

    /// VFS adapter for close(fd).
    ///
    /// @param fd is the file descriptor to close.
    ///
    /// When this method is invoked it will disable the TWAI driver and stop the
    /// periodic timer used for RX/TX of frame data if it is running.
    ///
    /// @return zero upon success, negative value with errno for failure.
    int close(int fd)
    {
        LOG(INFO, "[TWAI] Disabling TWAI fd:%d", fd);
        txBuf_->flush();

        twai_hal_stop(&twaiContext_);
        return 0;
    }

    /// VFS adapter for ioctl.
    ///
    /// @param fd is the file descriptor to operate on.
    /// @param cmd is the command to execute.
    /// @param args is the args for the command.
    ///
    /// @return zero upon success, negative value with errno for failure.
    int ioctl(int fd, int cmd, va_list args)
    {
        /* sanity check to be sure we have a valid key for this device */
        HASSERT(IOC_TYPE(cmd) == CAN_IOC_MAGIC);

        // Will be called at the end if non-null.
        Notifiable* n = nullptr;

        if (IOC_SIZE(cmd) == NOTIFIABLE_TYPE)
        {
            n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
        }

        switch (cmd)
        {
            default:
                return -EINVAL;
            case CAN_IOC_READ_ACTIVE:
                {
                    AtomicHolder h(&bufLock_);
                    if (!rxBuf_->pending())
                    {
                        std::swap(n, readableNotify_);
                    }
                }
                break;
            case CAN_IOC_WRITE_ACTIVE:
                {
                    AtomicHolder h(&bufLock_);
                    if (!txBuf_->space())
                    {
                        std::swap(n, readableNotify_);
                    }
                }
                break;
        }
        if (n)
        {
            n->notify();
        }
        return 0;
    }

    /// VFS interface helper for write()
    ///
    /// @param fd is the file descriptor being written to.
    /// @param buf is the buffer containing the data to be written.
    /// @param size is the size of the buffer.
    /// @return number of bytes written or -1 if there is the write would be a
    /// blocking operation.
    ssize_t write(int fd, const void *buf, size_t size)
    {
        DASSERT(fd == TWAI_VFS_FD);
        ssize_t sent = 0;
        const struct can_frame *data = (const struct can_frame *)buf;
        size /= sizeof(struct can_frame);
        while (size)
        {
            if (twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_BUS_OFF))
            {
                twai_hal_start_bus_recovery(&twaiContext_);
                txBuf_->flush();
                break;
            }
            else if (!twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_RUNNING))
            {
                break;
            }
            
            size_t frames_written = 0;
            {
                AtomicHolder h(&bufLock_);
                frames_written = txBuf_->put(data, size < 8 ? size : 8);
            }
            if (frames_written == 0)
            {
                // No space in the TX queue
                break;
            }
            if (twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_RUNNING) &&
               !twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED))
            {
                // since the TX buffer is not occupied, retrieve the first
                // frame and transmit it here.
                AtomicHolder h(&bufLock_);
                struct can_frame frame;
                twai_message_t tx_frame;
                twai_hal_frame_t hal_frame;
                HASSERT(txBuf_->get(&frame, 1));
                bzero(&tx_frame, sizeof(twai_message_t));
                tx_frame.identifier = frame.can_id;
                tx_frame.extd = frame.can_eff;
                tx_frame.rtr = frame.can_rtr;
                tx_frame.data_length_code = frame.can_dlc;
                memcpy(tx_frame.data, frame.data, frame.can_dlc);
                twai_hal_format_frame(&tx_frame, &hal_frame);
                twai_hal_set_tx_buffer_and_transmit(&twaiContext_, &hal_frame);
            }
            sent += frames_written;
            size -= frames_written;
        }
        if (!sent)
        {
            if (nonBlocking_)
            {
                return 0;
            }
            errno = EWOULDBLOCK;
            return -1;
        }
        return sent * sizeof(struct can_frame);
    }

    /// VFS interface helper for read()
    ///
    /// @param fd is the file descriptor being read from.
    /// @param buf is the buffer to write into.
    /// @param size is the size of the buffer.
    /// @return number of bytes read or -1 if there is the read would be a
    /// blocking operation.
    ssize_t read(int fd, void *buf, size_t size)
    {
        DASSERT(fd == TWAI_VFS_FD);

        ssize_t received = 0;
        struct can_frame *data = (struct can_frame *)buf;
        size /= sizeof(struct can_frame);
        while (size)
        {
            size_t received_frames = 0;
            {
                AtomicHolder h(&bufLock_);
                received_frames = rxBuf_->get(data, size < 8 ? size : 8);
            }
            if (received_frames == 0)
            {
                break;
            }
            rxProcessed_ += received_frames;
            size -= received_frames;
            received += received_frames;
            data += received_frames;
        }
        if (!received)
        {
            errno = EWOULDBLOCK;
            return -1;
        }

        return received * sizeof(struct can_frame);
    }

    /// VFS interface helper for select()
    ///
    /// @param nfds is the number of FDs being checked.
    /// @param readfds is the set of FDs being checked for ready to read.
    /// @param writefds is the set of FDs being checked for ready to write.
    /// @param exceptfds is the set of FDs being checked for exception.
    /// @param sem is the semaphore to use for waking up the select() call.
    void start_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds
                    , esp_vfs_select_sem_t sem)
    {
        // If our VFS FD is present in any of the FD sets we should process the
        // select.
        if (nfds >= 1 &&
            (FD_ISSET(TWAI_VFS_FD, readfds) || FD_ISSET(TWAI_VFS_FD, writefds) ||
             FD_ISSET(TWAI_VFS_FD, exceptfds)))
        {
            selectSem_ = sem;
            selectSemValid_ = true;
            readFds_ = readfds;
            origReadFds_ = *readfds;
            writeFds_ = writefds;
            origWriteFds_ = *writefds;
        }
        else
        {
            selectSemValid_ = false;
        }
    }

    /// VFS interface helper invoked when select() is woken up.
    ///
    /// @param end_select_args is any arguments provided in vfs_start_select().
    void end_select()
    {
        selectSemValid_ = false;
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
    int fcntl(int fd, int cmd, int arg)
    {
        HASSERT(fd == TWAI_VFS_FD);
        int result = 0;

        if (cmd == F_GETFL)
        {
            if (nonBlocking_)
            {
                result |= O_NONBLOCK;
            }
        }
        else if (cmd == F_SETFL)
        {
            nonBlocking_ = arg & O_NONBLOCK;
        }
        else
        {
            errno = ENOSYS;
            result = -1;
        }

        return result;
    }

    void isr()
    {
        BaseType_t wakeup = pdFALSE;
        bool wakeup_select = false;
        uint32_t events = twai_hal_decode_interrupt_events(&twaiContext_);
        
        // RX completed
        if (events & TWAI_HAL_EVENT_RX_BUFF_FRAME)
        {
            AtomicHolder h(&bufLock_);
            twai_hal_frame_t frame;
            struct can_frame *can_frame = nullptr;
            size_t space = rxBuf_->data_write_pointer(&can_frame);
            twai_hal_read_rx_buffer_and_clear(&twaiContext_, &frame);
            if (space)
            {
                if (frame.dlc > TWAI_FRAME_MAX_DLC)
                {
                    // DLC is longer than supported, discard the frame.
                    rxDiscard_++;
                }
                else
                {
                    twai_message_t rx_frame;
                    twai_hal_parse_frame(&frame, &rx_frame);
                    memcpy(can_frame->data, rx_frame.data, TWAI_FRAME_MAX_DLC);
                    can_frame->can_dlc = rx_frame.data_length_code;
                    can_frame->can_id = rx_frame.identifier;
                    can_frame->can_eff = rx_frame.extd;
                    can_frame->can_rtr = rx_frame.rtr;
                    rxBuf_->advance(1);
                    wakeup_select = true;
                    if (readableNotify_)
                    {
                        readableNotify_->notify_from_isr();
                        readableNotify_ = nullptr;
                    }
                }
            }
            else
            {
                rxOverrun_++;
            }
        }

        // TX completed
        if (events & TWAI_HAL_EVENT_TX_BUFF_FREE)
        {
            AtomicHolder h(&bufLock_);
            if (twai_hal_check_last_tx_successful(&twaiContext_))
            {
                txSuccess_++;
            }
            else
            {
                txFailed_++;
            }

            struct can_frame *can_frame = nullptr;
            size_t pending = txBuf_->data_read_pointer(&can_frame);
            if (pending)
            {
                twai_message_t tx_frame;
                twai_hal_frame_t hal_frame;
                bzero(&tx_frame, sizeof(twai_message_t));
                tx_frame.identifier = can_frame->can_id;
                tx_frame.extd = can_frame->can_eff;
                tx_frame.rtr = can_frame->can_rtr;
                tx_frame.data_length_code = can_frame->can_dlc;
                memcpy(tx_frame.data, can_frame->data, can_frame->can_dlc);
                twai_hal_format_frame(&tx_frame, &hal_frame);
                twai_hal_set_tx_buffer_and_transmit(&twaiContext_, &hal_frame);
                txBuf_->consume(1);
                wakeup_select = true;
                if (writableNotify_)
                {
                    writableNotify_->notify_from_isr();
                    writableNotify_ = nullptr;
                }
            }
        }

        // Bus recovery complete, trigger a restart
        if (events & TWAI_HAL_EVENT_BUS_RECOV_CPLT)
        {
            twai_hal_start(&twaiContext_, TWAI_MODE_NORMAL);
        }
        // Bus error detected
        if (events & TWAI_HAL_EVENT_BUS_ERR)
        {
            busError_++;
        }
        // Arbitration error detected
        if (events & TWAI_HAL_EVENT_ARB_LOST)
        {
            arbError_++;
        }

        // If we need to wake up select() do so now.
        if (wakeup_select && selectSemValid_)
        {
            if (FD_ISSET(TWAI_VFS_FD, &origReadFds_) &&
                (events & TWAI_HAL_EVENT_RX_BUFF_FRAME))
            {
                FD_SET(TWAI_VFS_FD, readFds_);
            }
            if (FD_ISSET(TWAI_VFS_FD, &origWriteFds_) &&
                (events & TWAI_HAL_EVENT_TX_BUFF_FREE))
            {
                FD_SET(TWAI_VFS_FD, writeFds_);
            }
            esp_vfs_select_triggered_isr(selectSem_, &wakeup);
        }

// In IDF v4.3 the portYIELD_FROM_ISR macro was extended to take the value of
// the wakeup flag and if true will yield as expected otherwise it is mostly a
// no-op.
#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(4,2,0)
        if (wakeup == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
#else
        portYIELD_FROM_ISR(wakeup);
#endif // ESP_IDF_VERSION > 4.2.0
    }

    static void *twai_stats_entry(void *param)
    {
        Esp32HardwareTwai *parent = (Esp32HardwareTwai *)param;
        parent->twai_stats_report();
        return nullptr;
    }

    /// Periodic TWAI statistics reporting task.
    void twai_stats_report()
    {
        while (twaiConfigured_)
        {
            LOG(INFO
            , "[TWAI] RX:%d (pending:%zu,overrun:%d,discard:%d)"
                " TX:%d (pending:%zu,suc:%d,fail:%d)"
                " bus (arb-err:%d,err:%d,state:%s)"
            , rxProcessed_, rxBuf_->pending(), rxOverrun_, rxDiscard_
            , txProcessed_, txBuf_->pending(), txSuccess_, txFailed_
            , arbError_, busError_
            , twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_RUNNING) ? "Running"
            : twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_RECOVERING) ? "Recovering"
            : twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_ERR_WARN) ? "Err-Warn"
            : twai_hal_check_state_flags(&twaiContext_, TWAI_HAL_STATE_FLAG_ERR_PASSIVE) ? "Err-Pasv"
            : "Bus Off");

            // delay until the next reporting interval, this is being used instead
            // of vTaskDelay to allow early wake up in the case of shutdown of the
            // TWAI driver.
            ulTaskNotifyTake(pdTRUE, STATUS_PRINT_INTERVAL);
        }
    }
private:
    /// GPIO pin connected to the external transceiver RX pin.
    const gpio_num_t rxPin_;

    /// GPIO pin connected to the external transceiver TX pin.
    const gpio_num_t txPin_;

    /// Flag to indicate that TWAI statistics should be periodically reported.
    const bool reportStats_;

    /// VFS Mount point.
    const char *vfsPath_;

    /// Receive buffer
    DeviceBuffer<struct can_frame> *rxBuf_;

    /// Transmit buffer
    DeviceBuffer<struct can_frame> *txBuf_;
    
    /// Lock protecting @ref txBuf_ and @ref rxBuf_.
    Atomic bufLock_;

    /// This will be notified if the device has data avilable for read.
    Notifiable* readableNotify_;

    /// This will be notified if the device has buffer avilable for write.
    Notifiable* writableNotify_;

    /// Flag indicating that the file descriptor has been opened with the
    /// O_NONBLOCK flag.
    bool nonBlocking_{false};

    /// VFS semaphore that can be used to prematurely wakeup a call to select.
    /// NOTE: This is only valid after VFS has called @ref start_select and is
    /// invalid after VFS calls @ref end_select.
    esp_vfs_select_sem_t selectSem_;

    /// Flag to indicate that @ref selectSem_ is valid or not.
    bool selectSemValid_{false};

    /// Pointer to the fd_set provided by the ESP32 VFS layer used to indicate
    /// the fd is ready to be read.
    fd_set *readFds_;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to check if
    /// the there is a read operation pending for the fd.
    fd_set origReadFds_;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to indicate the
    /// fd is ready to be written to.
    fd_set *writeFds_;

    /// Copy of the fd_set provided by the ESP32 VFS layer used to check if
    /// the there is a write operation pending for the fd.
    fd_set origWriteFds_;

    /// Number of frames have been removed from @ref rxBuf_ and sent to the
    /// OpenMRN stack.
    uint32_t rxProcessed_{0};

    /// Number of frames frames that could not be sent to @ref rxBuf_.
    uint32_t rxOverrun_{0};

    /// Number of frames that were discarded that had too large of a DLC count.
    uint32_t rxDiscard_{0};

    /// Number of frames that have been sent to the @ref twai_tx_queue by the
    /// OpenMRN stack successfully.
    uint32_t txProcessed_{0};

    /// Number of frames that have been transmitted successfully by the low-level
    /// TWAI driver.
    uint32_t txSuccess_{0};

    /// Number of frames that have been could not be transmitted successfully by
    /// the low-level TWAI driver.
    uint32_t txFailed_{0};

    /// Number of arbitration errors that have been observed on the TWAI bus.
    uint32_t arbError_{0};

    /// Number of general bus errors that have been observed on the TWAI bus.
    uint32_t busError_{0};

    /// Internal flag used for tracking if the VFS driver has been registered.
    bool vfsRegistered_{false};

    /// Internal flag used for tracking if the low-level TWAI driver has been
    /// configured.
    bool twaiConfigured_{false};

    /// TWAI HAL context object.
    twai_hal_context_t twaiContext_;

    /// Handle for the TWAI ISR.
    intr_handle_t twaiISRHandle_;

    /// Default file descriptor to return in the vfs_open() call.
    ///
    /// NOTE: The TWAI driver only supports one file descriptor at this time.
    static constexpr int TWAI_VFS_FD = 0;

    /// Interval at which to print the ESP32 TWAI bus status.
    static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

    /// TWAI peripheral ISR flags.
    ///
    /// Since the ISR is written in C/C++ code we can only specify the flag
    /// ESP_INTR_FLAG_LOWMED. We can optionally specify ESP_INTR_FLAG_IRAM but
    /// there are intermittent crashes when the ISR is in IRAM due to flash
    /// access (ie SPIFFS or LittleFS).
    static constexpr int TWAI_ISR_FLAGS = ESP_INTR_FLAG_LOWMED;

    /// TWAI default interrupt enable mask, excludes data overrun (bit[3]) and
    /// brp_div (bit[4]) since these are not supported on all models.
    static constexpr uint32_t TWAI_DEFAULT_INTERRUPTS = 0xE7;

// Starting in IDF v4.2 the CAN peripheral was renamed to TWAI.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,2,0)
    /// Pin matrix index for the TWAI TX peripheral pin.
    static constexpr uint32_t TWAI_TX_SIGNAL_IDX = TWAI_TX_IDX;

    /// Pin matrix index for the TWAI RX peripheral pin.
    static constexpr uint32_t TWAI_RX_SIGNAL_IDX = TWAI_RX_IDX;

    /// TWAI peripheral ISR source index.
    static constexpr int TWAI_ISR_SOURCE = ETS_TWAI_INTR_SOURCE;

    /// TWAI Peripheral module ID.
    static constexpr periph_module_t TWAI_PERIPHERAL = PERIPH_TWAI_MODULE;
#else
    /// Pin matrix index for the TWAI TX peripheral pin.
    static constexpr uint32_t TWAI_TX_SIGNAL_IDX = CAN_TX_IDX;

    /// Pin matrix index for the TWAI RX peripheral pin.
    static constexpr uint32_t TWAI_RX_SIGNAL_IDX = CAN_RX_IDX;

    /// TWAI peripheral ISR source index.
    static constexpr int TWAI_ISR_SOURCE = ETS_CAN_INTR_SOURCE;

    /// TWAI Peripheral module ID.
    static constexpr periph_module_t TWAI_PERIPHERAL = PERIPH_CAN_MODULE;
#endif // ESP_IDF_VERSION > 4.2.0
};

static ssize_t twai_vfs_write(int fd, const void *buf, size_t size)
{
    return Singleton<Esp32HardwareTwai>::instance()->write(fd, buf, size);
}

static ssize_t twai_vfs_read(int fd, void *buf, size_t size)
{
    return Singleton<Esp32HardwareTwai>::instance()->read(fd, buf, size);
}

static int twai_vfs_open(const char *path, int flags, int mode)
{
    return Singleton<Esp32HardwareTwai>::instance()->open(path, flags, mode);
}

static int twai_vfs_close(int fd)
{
    return Singleton<Esp32HardwareTwai>::instance()->close(fd);
}

static int twai_vfs_ioctl(int fd, int cmd, va_list args)
{
    return Singleton<Esp32HardwareTwai>::instance()->ioctl(fd, cmd, args);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
static int twai_vfs_fcntl(int fd, int cmd, int arg)
{
    return Singleton<Esp32HardwareTwai>::instance()->fcntl(fd, cmd, arg);
}
static esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds
                                     , fd_set *writefds, fd_set *exceptfds
                                     , esp_vfs_select_sem_t sem
                                     , void **end_select_args)
{
    Singleton<Esp32HardwareTwai>::instance()->start_select(nfds, readfds
                                                         , writefds, exceptfds
                                                         , sem);
    return ESP_OK;
}
static esp_err_t twai_vfs_end_select(void *end_select_args)
{
    Singleton<Esp32HardwareTwai>::instance()->end_select();
    return ESP_OK;
}
#else
static int twai_vfs_fcntl(int fd, int cmd, va_list arg)
{
    return Singleton<Esp32HardwareTwai>::instance()->fcntl(fd, cmd, va_arg(arg, int));
}
static esp_err_t twai_vfs_start_select(int nfds, fd_set *readfds
                                     , fd_set *writefds, fd_set *exceptfds
                                     , esp_vfs_select_sem_t sem)
{
    Singleton<Esp32HardwareTwai>::instance()->start_select(nfds, readfds
                                                         , writefds, exceptfds
                                                         , sem);
    return ESP_OK;
}
static void twai_vfs_end_select()
{
    Singleton<Esp32HardwareTwai>::instance()->end_select();
}
#endif

static void twai_isr(void *arg)
{
    Esp32HardwareTwai *parent = (Esp32HardwareTwai *)arg;
    parent->isr();
}

#endif // ESP32

#endif // _FREERTOS_DRIVERS_ESP32_ESP32HARDWARETWAI_HXX_