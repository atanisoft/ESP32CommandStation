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
 * TWAI driver implementation for OpenMRN. This leverages a clone the ESP-IDF
 * HAL API rather than the TWAI driver to allow for faster access to received
 * frames and support for ESP-IDF v4.0 and above.
 *
 * @author Mike Dunston
 * @date 24 September 2020
 */

#include "freertos_drivers/esp32/Esp32Twai.hxx"

#if ESP32_TWAI_DRIVER_SUPPORTED

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <esp_task.h>
#include <esp_vfs.h>
#include <esp_intr_alloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "can_frame.h"
#include "can_ioctl.h"
#include "Esp32TwaiHal.hxx"
#include "executor/Notifiable.hxx"
#include "freertos_drivers/arduino/DeviceBuffer.hxx"
#include "nmranet_config.h"
#include "utils/logging.h"

#define INCLUDE_VFS_SELECT_SUPPORT 1
//#define INCLUDE_VFS_IOCTL_SUPPORT 1

/// Default file descriptor to return in the vfs_open() call.
/// NOTE: The TWAI driver only supports one file descriptor at this time.
static constexpr int TWAI_VFS_FD = 0;

/// Interval at which to print the ESP32 TWAI bus status.
static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

/// Stack size to use for the periodic statistics reporting task.
static constexpr size_t STATUS_TASK_STACK_SIZE = 2048;

/// TWAI peripheral ISR flags.
static constexpr int TWAI_ISR_FLAGS =
    ESP_INTR_FLAG_LOWMED |  // ISR is written in C/C++ code.
    ESP_INTR_FLAG_IRAM;     // ISR can be called with cache disabled.

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
#endif

/// Internal flag used for tracking if the VFS driver has been registered.
static bool vfs_is_registered = false;

/// Internal flag used for tracking if the low-level TWAI driver has been
/// configured.
static bool twai_is_configured = false;

/// TWAI default interrupt enable mask, excludes data overrun (bit[3]) and
/// brp_div (bit[4]) since these are not supported on all models.
static constexpr uint32_t TWAI_DEFAULT_INTERRUPTS = 0xE7;

/// TWAI HAL context object.
static twai_hal_context_t twai_context;

/// Handle for the TWAI ISR.
static intr_handle_t twai_isr_handle;

/// TWAI Pending TX Queue.
static QueueHandle_t twai_tx_queue;

/// TWAI Pending RX Queue.
static QueueHandle_t twai_rx_queue;

/// Number of frames that are pending transmission by the low-level TWAI
/// driver.
/// NOTE: This saves calling FreeRTOS functions to check @ref twai_tx_queue.
static volatile uint32_t twai_tx_pending = 0;

/// Number of frames that have been received by the low-level TWAI driver but
/// have not been read from @ref twai_rx_queue.
/// NOTE: This saves calling FreeRTOS functions to check @ref twai_rx_queue.
static volatile uint32_t twai_rx_pending = 0;

/// Number of frames have been removed from @ref twai_rx_queue and sent to the
/// OpenMRN stack.
static volatile uint32_t twai_rx_processed = 0;

/// Number of frames frames that could not be sent to @ref twai_rx_queue.
static volatile uint32_t twai_rx_overrun_count = 0;

/// Number of frames that were discarded that had too large of a DLC count.
static volatile uint32_t twai_rx_discard_count = 0;

/// Number of frames that have been sent to the @ref twai_tx_queue by the
/// OpenMRN stack successfully.
static volatile uint32_t twai_tx_processed = 0;

/// Number of frames that have been transmitted successfully by the low-level
/// TWAI driver.
static volatile uint32_t twai_tx_success_count = 0;

/// Number of frames that have been could not be transmitted successfully by
/// the low-level TWAI driver.
static volatile uint32_t twai_tx_failed_count = 0;

/// Number of arbitration errors that have been observed on the TWAI bus.
static volatile uint32_t twai_arb_lost_count = 0;

/// Number of general bus errors that have been observed on the TWAI bus.
static volatile uint32_t twai_bus_error_count = 0;

/// Thread handle for the statistics reporting task.
static os_thread_t status_thread_handle;

#if INCLUDE_VFS_SELECT_SUPPORT
/// Semaphore provided by the VFS layer when select() is invoked on one of the
/// file descriptors handle by the TWAI VFS adapter.
static esp_vfs_select_sem_t twai_select_sem;

/// Internal flag indicating that the VFS layer has called
/// @ref vfs_start_select() and that @ref twai_select_sem should be used if/when
/// there is a change in @ref twai_tx_queue or @ref twai_rx_queue.
static volatile uint32_t twai_select_pending;
#endif // INCLUDE_VFS_SELECT_SUPPORT

#if INCLUDE_VFS_IOCTL_SUPPORT
/// Spinlock protecting the ioctl @ref Notifiable objects below.
static SemaphoreHandle_t twai_ioctl_spinlock;

/// @ref Notifable object used to wakeup when there is room for at least one
/// frame in @ref twai_tx_queue.
static Notifiable *twai_tx_notifiable = nullptr;

/// @ref Notifable object used to wakeup when there is at least one frame
/// available in @ref twai_rx_queue.
static Notifiable *twai_rx_notifiable = nullptr;
#endif // INCLUDE_VFS_IOCTL_SUPPORT

/// Flushes all frames in @ref twai_tx_queue.
static inline void twai_flush_tx_queue()
{
    xQueueReset(twai_tx_queue);
    twai_tx_pending = 0;
}

/// Flushes all frames in @ref twai_rx_queue.
static inline void twai_flush_rx_queue()
{
    xQueueReset(twai_rx_queue);
    twai_rx_pending = 0;
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
static int vfs_open(const char *path, int flags, int mode)
{
    // skip past the '/' that is passed in as first character
    path++;

    LOG(INFO, "[TWAI] Enabling TWAI device:%s", path);
    twai_flush_tx_queue();
    twai_flush_rx_queue();

    twai_hal_start(&twai_context, TWAI_MODE_NORMAL);
    return TWAI_VFS_FD;
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
static int vfs_close(int fd)
{
    LOG(INFO, "[TWAI] Disabling TWAI fd:%d", fd);
    twai_flush_tx_queue();

    twai_hal_stop(&twai_context);
    return 0;
}

/// VFS interface helper for write()
///
/// @param fd is the file descriptor being written to.
/// @param buf is the buffer containing the data to be written.
/// @param size is the size of the buffer.
/// @return number of bytes written or -1 if there is the write would be a
/// blocking operation.
static ssize_t vfs_write(int fd, const void *buf, size_t size)
{
    DASSERT(fd == TWAI_VFS_FD);
    ssize_t sent = 0;
    const struct can_frame *data = (const struct can_frame *)buf;
    size /= sizeof(struct can_frame);
    while (size)
    {
        if (twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_BUS_OFF))
        {
            twai_hal_start_bus_recovery(&twai_context);
            twai_flush_tx_queue();
            break;
        }
        else if (!twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING))
        {
            break;
        }

        twai_message_t tx_frame;
        twai_hal_frame_t hal_frame;
        bzero(&tx_frame, sizeof(twai_message_t));
        tx_frame.identifier = data->can_id;
        tx_frame.extd = data->can_eff;
        tx_frame.rtr = data->can_rtr;
        tx_frame.data_length_code = data->can_dlc;
        memcpy(tx_frame.data, data->data, data->can_dlc);
        twai_hal_format_frame(&tx_frame, &hal_frame);
        if (xQueueSend(twai_tx_queue, &hal_frame, 0) == pdTRUE)
        {
            twai_tx_pending++;
            if (twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING) &&
                !twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED) &&
                xQueueReceive(twai_tx_queue, &hal_frame, 0) == pdTRUE)
            {
                twai_hal_set_tx_buffer_and_transmit(&twai_context, &hal_frame);
            }
            sent++;
            size--;
        }
        else
        {
            // No space in the TX queue
            break;
        }
    }
    if (!sent)
    {
        errno = EWOULDBLOCK;
        return -1;
    }
    twai_tx_processed += sent;
    return sent * sizeof(struct can_frame);
}

/// VFS interface helper for read()
///
/// @param fd is the file descriptor being read from.
/// @param buf is the buffer to write into.
/// @param size is the size of the buffer.
/// @return number of bytes read or -1 if there is the read would be a
/// blocking operation.
static ssize_t vfs_read(int fd, void *buf, size_t size)
{
    DASSERT(fd == TWAI_VFS_FD);

    ssize_t received = 0;
    struct can_frame *data = (struct can_frame *)buf;
    size /= sizeof(struct can_frame);
    while (size)
    {
        twai_hal_frame_t hal_frame;
        memset(&hal_frame, 0, sizeof(twai_hal_frame_t));
        if (xQueueReceive(twai_rx_queue, &hal_frame, 0) != pdTRUE)
        {
            // no frames available to receive
            break;
        }
        twai_rx_pending--;
        twai_message_t rx_frame;
        twai_hal_parse_frame(&hal_frame, &rx_frame);
        memcpy(data->data, rx_frame.data, TWAI_FRAME_MAX_DLC);
        data->can_dlc = rx_frame.data_length_code;
        data->can_id = rx_frame.identifier;
        data->can_eff = rx_frame.extd;
        data->can_rtr = rx_frame.rtr;
        size--;
        received++;
        data++;
    }
    if (!received)
    {
        errno = EWOULDBLOCK;
        return -1;
    }
    twai_rx_processed += received;

    return received * sizeof(struct can_frame);
}

#if INCLUDE_VFS_SELECT_SUPPORT
/// VFS interface helper for select()
///
/// @param nfds is the number of FDs being checked.
/// @param readfds is the set of FDs being checked for ready to read.
/// @param writefds is the set of FDs being checked for ready to write.
/// @param exceptfds is the set of FDs being checked for exception.
/// @param sem is the semaphore to use for waking up the select() call.
/// @param end_select_args is any arguments to pass into vfs_end_select().
/// @return ESP_OK for success.
static esp_err_t vfs_start_select(int nfds, fd_set *readfds
                                , fd_set *writefds, fd_set *exceptfds
                                , esp_vfs_select_sem_t sem
                                , void **end_select_args)
{
    HASSERT(nfds >= 1);
    *end_select_args = nullptr;
    twai_select_sem = sem;
    twai_select_pending = 1;

    return ESP_OK;
}

/// VFS interface helper invoked when select() is woken up.
///
/// @param end_select_args is any arguments provided in vfs_start_select().
/// @return ESP_OK for success.
static esp_err_t vfs_end_select(void *end_select_args)
{
    twai_select_pending = 0;
    return ESP_OK;
}
#endif // INCLUDE_VFS_SELECT_SUPPORT

/// VFS adapter for fcntl(fd, cmd, arg).
///
/// @param fd to operate on.
/// @param cmd to be executed.
/// @param arg arg to be used for the operation.
///
/// This method is currently a NO-OP.
///
/// @return zero upon success, negative value with errno for failure.
static int vfs_fcntl(int fd, int cmd, int arg)
{
    DASSERT(fd == TWAI_VFS_FD);

    // NO OP

    return 0;
}

#if INCLUDE_VFS_IOCTL_SUPPORT
/// VFS adapter for ioctl(fd, cmd, args).
///
/// @param fd to operate on.
/// @param cmd to be executed.
/// @param args args to be used for the operation.
///
/// This method is currently a NO-OP.
///
/// @return zero upon success, negative value with errno for failure.
static int vfs_ioctl(int fd, int cmd, va_list args)
{
    DASSERT(fd == TWAI_VFS_FD);
    if (IOC_TYPE(cmd) != CAN_IOC_MAGIC)
    {
        errno = EINVAL;
        return -1;
    }
    else if (IOC_SIZE(cmd) == NOTIFIABLE_TYPE)
    {
        Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
        HASSERT(n);
        xSemaphoreTake(twai_ioctl_spinlock, portMAX_DELAY);
        if (cmd == CAN_IOC_WRITE_ACTIVE && twai_tx_pending == 0)
        {
            std::swap(n, twai_tx_notifiable);
        }
        else if (cmd == CAN_IOC_READ_ACTIVE && twai_rx_pending == 0)
        {
            std::swap(n, twai_rx_notifiable);
        }
        xSemaphoreGive(twai_ioctl_spinlock);

        if (n)
        {
            n->notify();
        }
    }
    return 0;
}
#endif // INCLUDE_VFS_IOCTL_SUPPORT

/// Interrupt routine that processing events raised by the low-level TWAI
/// driver.
///
/// @param arg Unused
static IRAM_ATTR void twai_isr(void *arg)
{
    BaseType_t wakeup = pdFALSE;
    uint32_t events = twai_hal_decode_interrupt_events(&twai_context);
    bool is_rx = (events & TWAI_HAL_EVENT_RX_BUFF_FRAME);
    bool is_tx = (events & TWAI_HAL_EVENT_TX_BUFF_FREE);

    if (is_rx)
    {
        uint32_t msg_count = twai_hal_get_rx_msg_count(&twai_context);
        for (int i = 0; i < msg_count; i++)
        {
            twai_hal_frame_t frame;
            twai_hal_read_rx_buffer_and_clear(&twai_context, &frame);
            if (frame.dlc > TWAI_FRAME_MAX_DLC)
            {
                twai_rx_discard_count++;
            }
            else if (xQueueSendFromISR(twai_rx_queue, &frame, &wakeup) == pdTRUE)
            {
                twai_rx_pending++;
            }
            else
            {
                twai_rx_overrun_count++;
            }
        }
    }

    if (is_tx)
    {
        twai_tx_pending--;
        if (twai_hal_check_last_tx_successful(&twai_context))
        {
            twai_tx_success_count++;
        }
        else
        {
            twai_tx_failed_count++;
        }

        twai_hal_frame_t tx_frame;
        if (xQueueReceiveFromISR(twai_tx_queue, &tx_frame, &wakeup) == pdTRUE)
        {
            twai_hal_set_tx_buffer_and_transmit(&twai_context, &tx_frame);
        }
    }

    if (events & TWAI_HAL_EVENT_BUS_RECOV_CPLT)
    {
        twai_hal_start(&twai_context, TWAI_MODE_NORMAL);
    }
    if (events & TWAI_HAL_EVENT_BUS_ERR)
    {
        twai_bus_error_count++;
    }
    if (events & TWAI_HAL_EVENT_ARB_LOST)
    {
        twai_arb_lost_count++;
    }

#if INCLUDE_VFS_SELECT_SUPPORT
    if ((is_rx || is_tx) && twai_select_pending)
    {
        esp_vfs_select_triggered_isr(twai_select_sem, &wakeup);
    }
#endif // INCLUDE_VFS_SELECT_SUPPORT

#if INCLUDE_VFS_IOCTL_SUPPORT
    if ((is_tx || is_rx) &&
        xSemaphoreTakeFromISR(twai_ioctl_spinlock, &wakeup) == pdTRUE)
    {
        if (twai_tx_notifiable)
        {
            twai_tx_notifiable->notify_from_isr();
            twai_tx_notifiable = nullptr;
        }
        if (twai_rx_notifiable)
        {
            twai_rx_notifiable->notify_from_isr();
            twai_rx_notifiable = nullptr;
        }
        xSemaphoreGiveFromISR(twai_ioctl_spinlock, &wakeup);
    }
#endif // INCLUDE_VFS_IOCTL_SUPPORT

#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    portYIELD_FROM_ISR(wakeup);
#else
    if (wakeup == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
#endif
}

/// Periodic TWAI statistics reporting task.
///
/// @param param (unused)
static void *report_stats(void *param)
{
    while (twai_is_configured)
    {
        LOG(INFO
          , "[TWAI] RX:%d (pending:%d,overrun:%d,discard:%d)"
            " TX:%d (pending:%d,suc:%d,fail:%d)"
            " bus (arb-err:%d,err:%d,state:%s)"
          , twai_rx_processed, twai_rx_pending, twai_rx_overrun_count
          , twai_rx_discard_count, twai_tx_processed, twai_tx_pending
          , twai_tx_success_count, twai_tx_failed_count, twai_arb_lost_count
          , twai_bus_error_count
          , twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING) ? "Running"
          : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RECOVERING) ? "Recovering"
          : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_ERR_WARN) ? "Err-Warn"
          : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_ERR_PASSIVE) ? "Err-Pasv"
          : "Bus Off");

        // delay until the next reporting interval, this is being used instead
        // of vTaskDelay to allow early wake up in the case of shutdown of the
        // TWAI driver.
        ulTaskNotifyTake(pdTRUE, STATUS_PRINT_INTERVAL);
    }
    return nullptr;
}

static inline void configure_twai_gpio(gpio_num_t tx, gpio_num_t rx)
{
    LOG(INFO, "[TWAI] Configuring TWAI TX pin: %d", tx);
    gpio_set_pull_mode(tx, GPIO_FLOATING);
    gpio_matrix_out(tx, TWAI_TX_SIGNAL_IDX, false, false);
    gpio_pad_select_gpio(tx);

    LOG(INFO, "[TWAI] Configuring TWAI RX pin: %d", rx);
    gpio_set_pull_mode(rx, GPIO_FLOATING);
    gpio_set_direction(rx, GPIO_MODE_INPUT);
    gpio_matrix_in(rx, TWAI_RX_SIGNAL_IDX, false);
    gpio_pad_select_gpio(rx);
}

static inline void create_twai_buffers()
{
    LOG(INFO, "[TWAI] Creating TWAI TX queue: %d"
      , config_can_tx_buffer_size());
    twai_tx_queue =
        xQueueCreate(config_can_tx_buffer_size(), sizeof(twai_hal_frame_t));
    HASSERT(twai_tx_queue != nullptr);
    LOG(INFO, "[TWAI] Creating TWAI RX queue: %d"
      , config_can_rx_buffer_size());
    twai_rx_queue =
        xQueueCreate(config_can_rx_buffer_size(), sizeof(twai_hal_frame_t));
    HASSERT(twai_rx_queue != nullptr);
}

static inline void initialize_twai()
{
    periph_module_reset(TWAI_PERIPHERAL);
    periph_module_enable(TWAI_PERIPHERAL);
    HASSERT(twai_hal_init(&twai_context));
    twai_timing_config_t timingCfg = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t filterCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    LOG(INFO, "[TWAI] Initiailizing TWAI peripheral");
    twai_hal_configure(&twai_context, &timingCfg, &filterCfg
                     , TWAI_DEFAULT_INTERRUPTS, 0);
    LOG(INFO, "[TWAI] Allocating TWAI ISR");
    ESP_ERROR_CHECK(
        esp_intr_alloc(TWAI_ISR_SOURCE, TWAI_ISR_FLAGS, twai_isr, NULL
                     , &twai_isr_handle));
}

// Constructor.
Esp32Twai::Esp32Twai(const char *vfsPath, int rxPin, int txPin, bool report)
    : vfsPath_(vfsPath), rxPin_((gpio_num_t)rxPin), txPin_((gpio_num_t)txPin)
    , reportStats_(report)
{
    HASSERT(GPIO_IS_VALID_GPIO(rxPin));
    HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(txPin));
}

// Destructor.
Esp32Twai::~Esp32Twai()
{
    if (vfs_is_registered)
    {
        ESP_ERROR_CHECK(esp_vfs_unregister(vfsPath_));
    }

    if (twai_is_configured)
    {
        ESP_ERROR_CHECK(esp_intr_free(twai_isr_handle));
        twai_hal_deinit(&twai_context);
        twai_is_configured = false;
        xTaskNotifyGive(status_thread_handle);
        vQueueDelete(twai_tx_queue);
        vQueueDelete(twai_rx_queue);
    }
}

// Initializes the VFS adapter and TWAI driver
void Esp32Twai::hw_init()
{
    esp_vfs_t vfs;
    memset(&vfs, 0, sizeof(esp_vfs_t));
    vfs.write = vfs_write;
    vfs.read = vfs_read;
    vfs.open = vfs_open;
    vfs.close = vfs_close;
    vfs.fcntl = vfs_fcntl;
#if INCLUDE_VFS_IOCTL_SUPPORT
    vfs.ioctl = vfs_ioctl;
#endif // INCLUDE_VFS_IOCTL_SUPPORT
    vfs.start_select = vfs_start_select;
    vfs.end_select = vfs_end_select;
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    ESP_ERROR_CHECK(esp_vfs_register(vfsPath_, &vfs, this));
    vfs_is_registered = true;

#if INCLUDE_VFS_IOCTL_SUPPORT
    twai_ioctl_spinlock = xSemaphoreCreateBinary();
#endif

    configure_twai_gpio(txPin_, rxPin_);
    create_twai_buffers();
    initialize_twai();

    twai_is_configured = true;
    if (reportStats_)
    {
        os_thread_create(&status_thread_handle, "TWAI-STATS"
                       , ESP_TASK_MAIN_PRIO, STATUS_TASK_STACK_SIZE
                       , report_stats, nullptr);
    }
}

#endif // INCLUDE_TWAI_DRIVER