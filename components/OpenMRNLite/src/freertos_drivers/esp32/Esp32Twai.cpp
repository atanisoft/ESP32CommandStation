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

//#define USE_FREERTOS_PORTMUX 1
#define USE_FREERTOS_BIN_SEMAPHORE 1

#if USE_FREERTOS_PORTMUX && USE_FREERTOS_BIN_SEMAPHORE
#error Only one of these can be enabled at one time: USE_FREERTOS_PORTMUX or USE_FREERTOS_BIN_SEMAPHORE.
#endif

//#define INCLUDE_VFS_IOCTL_SUPPORT 1

static constexpr int TWAI_VFS_FD = 0;

/// Interval at which to print the ESP32 TWAI bus status.
static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

static const size_t TWAI_RX_BUFFER_SIZE = 
    ((size_t)config_can_rx_buffer_size()) * 10;
static const size_t TWAI_TX_BUFFER_SIZE =
    (size_t)config_can_tx_buffer_size();

/// Periodic timer handle.
static xTimerHandle twai_status_timer = nullptr;

/// Internal flag used for tracking if the VFS driver has been registered.
static bool vfs_is_registered = false;

/// Internal flag used for tracking if the low-level TWAI driver has been
/// configured.
static bool twai_is_configured = false;

//Exclude data overrun (bit[3]) and brp_div (bit[4])
static constexpr uint32_t TWAI_DEFAULT_INTERRUPTS = 0xE7;
static twai_hal_context_t twai_context;
static intr_handle_t twai_isr_handle;
static QueueHandle_t twai_tx_queue;
static QueueHandle_t twai_rx_queue;

static volatile uint32_t twai_tx_pending = 0;
static volatile uint32_t twai_rx_pending = 0;
static volatile uint32_t twai_rx_processed = 0;
static volatile uint32_t twai_rx_overrun_count = 0;
static volatile uint32_t twai_rx_discard_count = 0;
static volatile uint32_t twai_tx_processed = 0;
static volatile uint32_t twai_tx_success_count = 0;
static volatile uint32_t twai_tx_failed_count = 0;
static volatile uint32_t twai_arb_lost_count = 0;
static volatile uint32_t twai_bus_error_count = 0;

#if USE_FREERTOS_PORTMUX
static portMUX_TYPE twai_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define LOCK_MUX() portENTER_CRITICAL(&twai_spinlock)
#define LOCK_MUX_ISR(wakeup) portENTER_CRITICAL_ISR(&twai_spinlock)
#define UNLOCK_MUX() portEXIT_CRITICAL(&twai_spinlock)
#define UNLOCK_MUX_ISR(wakeup) portEXIT_CRITICAL_ISR(&twai_spinlock)
#elif USE_FREERTOS_BIN_SEMAPHORE
static SemaphoreHandle_t twai_spinlock;
#define LOCK_MUX() xSemaphoreTake(twai_spinlock, pdMS_TO_TICKS(1))
#define LOCK_MUX_ISR(wakeup) xSemaphoreTakeFromISR(twai_spinlock, &wakeup)
#define UNLOCK_MUX() xSemaphoreGive(twai_spinlock)
#define UNLOCK_MUX_ISR(wakeup) xSemaphoreGiveFromISR(twai_spinlock, &wakeup)
#else
#error USE_FREERTOS_PORTMUX or USE_FREERTOS_BIN_SEMAPHORE must be defined to 1.
#endif
static uint32_t twai_select_pending = 0;
static esp_vfs_select_sem_t twai_select_sem;
#if INCLUDE_VFS_IOCTL_SUPPORT
static Notifiable* twai_tx_notifiable = nullptr;
static Notifiable* twai_rx_notifiable = nullptr;
#endif // INCLUDE_VFS_IOCTL_SUPPORT

static inline void twai_reset_tx_queue()
{
    xQueueReset(twai_tx_queue);
    twai_tx_pending = 0;
}

static inline void twai_reset_rx_queue()
{
    xQueueReset(twai_rx_queue);
    twai_rx_pending = 0;
}

/// Enables the low-level TWAI driver.
static inline void twai_enable()
{
    twai_reset_tx_queue();
    twai_reset_rx_queue();

    LOCK_MUX();
    twai_hal_start(&twai_context, TWAI_MODE_NORMAL);
    UNLOCK_MUX();
}

/// Disables the low-level TWAI driver.
static inline void twai_disable()
{
    LOCK_MUX();
    twai_hal_stop(&twai_context);
    UNLOCK_MUX();

    twai_reset_tx_queue();
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
    LOG(INFO, "[TWAI] Opening TWAI device:%s", path);
    twai_enable();
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
    LOG(INFO, "[TWAI] Closing TWAI fd:%d", fd);
    twai_disable();
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
        twai_message_t tx_frame;
        memset(&tx_frame, 0, sizeof(twai_message_t));
        tx_frame.identifier = data->can_id;
        tx_frame.extd = IS_CAN_FRAME_EFF(*data);
        tx_frame.rtr = IS_CAN_FRAME_RTR(*data);
        tx_frame.data_length_code = data->can_dlc;
        memcpy(tx_frame.data, data->data, data->can_dlc);

        twai_hal_frame_t hal_frame;
        if (twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_BUS_OFF))
        {
            LOCK_MUX();
            twai_hal_start_bus_recovery(&twai_context);
            UNLOCK_MUX();
            twai_reset_tx_queue();
            break;
        }
        else if (!twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING))
        {
            break;
        }
        memset(&hal_frame, 0, sizeof(twai_hal_frame_t));
        twai_hal_format_frame(&tx_frame, &hal_frame);

        if (xQueueSend(twai_tx_queue, &hal_frame, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            LOCK_MUX();
            twai_tx_pending++;
            if ((!twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED))
              && twai_tx_pending > 0)
            {
                if (twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING) &&
                    xQueueReceive(twai_tx_queue, &hal_frame, 0) == pdTRUE)
                {
                    twai_hal_set_tx_buffer_and_transmit(&twai_context, &hal_frame);
                }
            }
            sent++;
            UNLOCK_MUX();
            size--;
        }
        else
        {
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
        if (xQueueReceive(twai_rx_queue, &hal_frame, pdMS_TO_TICKS(1)) != pdTRUE)
        {
            break;
        }
        LOCK_MUX();
        twai_rx_pending--;
        UNLOCK_MUX();
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
    LOCK_MUX();
    twai_select_pending = 1;
    twai_select_sem = sem;
    UNLOCK_MUX();

    return ESP_OK;
}

/// VFS interface helper invoked when select() is woken up.
///
/// @param end_select_args is any arguments provided in vfs_start_select().
/// @return ESP_OK for success.
static esp_err_t vfs_end_select(void *end_select_args)
{
    LOCK_MUX();
    twai_select_pending = 0;
    UNLOCK_MUX();
    return ESP_OK;
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
        LOCK_MUX();
        if (cmd == CAN_IOC_WRITE_ACTIVE && twai_tx_pending == 0)
        {
            std::swap(n, twai_tx_notifiable);
        }
        else if (cmd == CAN_IOC_READ_ACTIVE && twai_rx_pending == 0)
        {
            std::swap(n, twai_rx_notifiable);
        }
        UNLOCK_MUX();

        if (n)
        {
            n->notify();
        }
    }
    return 0;
}
#endif // INCLUDE_VFS_IOCTL_SUPPORT

static IRAM_ATTR void twai_isr(void *arg)
{
    BaseType_t wakeup = pdFALSE;

    LOCK_MUX_ISR(wakeup);
    uint32_t init_twai_rx_pending = twai_rx_pending;
    uint32_t init_twai_tx_pending = twai_tx_pending;

    uint32_t event = twai_hal_decode_interrupt_events(&twai_context);
    if (event & TWAI_HAL_EVENT_RX_BUFF_FRAME)
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
    if (event & TWAI_HAL_EVENT_TX_BUFF_FREE)
    {
        if (twai_hal_check_last_tx_successful(&twai_context))
        {
            twai_tx_success_count++;
        }
        else
        {
            twai_tx_failed_count++;
        }

        twai_tx_pending--;
        twai_hal_frame_t frame;
        if (twai_tx_pending > 0 &&
            xQueueReceiveFromISR(twai_tx_queue, &frame, &wakeup) == pdTRUE)
        {
            twai_hal_set_tx_buffer_and_transmit(&twai_context, &frame);
        }
    }

    if (event & TWAI_HAL_EVENT_BUS_RECOV_CPLT)
    {
        twai_hal_start(&twai_context, TWAI_MODE_NORMAL);
    }
    if (event & TWAI_HAL_EVENT_BUS_ERR)
    {
        twai_bus_error_count++;
    }
    if (event & TWAI_HAL_EVENT_ARB_LOST)
    {
        twai_arb_lost_count++;
    }

    if (init_twai_tx_pending != twai_tx_pending ||
        init_twai_rx_pending != twai_rx_pending)
    {
        if (twai_select_pending)
        {
            esp_vfs_select_triggered_isr(twai_select_sem, &wakeup);
            twai_select_pending = 0;
        }
#if INCLUDE_VFS_IOCTL_SUPPORT
        if (twai_tx_notifiable && init_twai_tx_pending != twai_tx_pending)
        {
            twai_tx_notifiable->notify_from_isr();
            twai_tx_notifiable = nullptr;
        }
        if (twai_rx_notifiable && init_twai_rx_pending != twai_rx_pending)
        {
            twai_rx_notifiable->notify_from_isr();
            twai_rx_notifiable = nullptr;
        }
#endif // INCLUDE_VFS_IOCTL_SUPPORT
    }
    UNLOCK_MUX_ISR(wakeup);

#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    portYIELD_FROM_ISR(wakeup);
#else
    if (wakeup == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
#endif
}

static void report_stats(xTimerHandle handle)
{
    if (!twai_is_configured)
    {
        return;
    }
    LOCK_MUX();
    uint32_t tx_pending = twai_tx_pending;
    uint32_t rx_pending = twai_rx_pending;
    uint32_t rx_processed = twai_rx_processed;
    uint32_t rx_overrun_count = twai_rx_overrun_count;
    uint32_t rx_discard_count = twai_rx_discard_count;
    uint32_t tx_processed = twai_tx_processed;
    uint32_t tx_success_count = twai_tx_success_count;
    uint32_t tx_failed_count = twai_tx_failed_count;
    uint32_t arb_lost_count = twai_arb_lost_count;
    uint32_t bus_error_count = twai_bus_error_count;
    UNLOCK_MUX();

    LOG(INFO
      , "[TWAI] RX:%d (pending:%d,overrun:%d,discard:%d)"
        " TX:%d (pending:%d,suc:%d,fail:%d)"
        " bus (arb-err:%d,err:%d,state:%s)"
      , rx_processed, rx_pending, rx_overrun_count, rx_discard_count
      , tx_processed, tx_pending, tx_success_count, tx_failed_count
      , arb_lost_count, bus_error_count
      , twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RUNNING) ? "Running"
      : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_RECOVERING) ? "Recovering"
      : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_ERR_WARN) ? "Err-Warn"
      : twai_hal_check_state_flags(&twai_context, TWAI_HAL_STATE_FLAG_ERR_PASSIVE) ? "Err-Pasv"
      : "Bus Off");
}

static inline void configure_twai_gpio(gpio_num_t tx, gpio_num_t rx)
{
    LOG(INFO, "[TWAI] Configuring TWAI TX pin: %d", tx);

    gpio_set_pull_mode(tx, GPIO_FLOATING);
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    gpio_matrix_out(tx, TWAI_TX_IDX, false, false);
#else
    gpio_matrix_out(tx, CAN_TX_IDX, false, false);
#endif
    gpio_pad_select_gpio(tx);

    LOG(INFO, "[TWAI] Configuring TWAI RX pin: %d", rx);
    gpio_set_pull_mode(rx, GPIO_FLOATING);
    gpio_set_direction(rx, GPIO_MODE_INPUT);
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    gpio_matrix_in(rx, TWAI_RX_IDX, false);
#else
    gpio_matrix_in(rx, CAN_RX_IDX, false);
#endif
    gpio_pad_select_gpio(rx);
}

static inline void create_twai_buffers()
{
    LOG(INFO, "[TWAI] Creating TWAI TX queue: %d", TWAI_TX_BUFFER_SIZE);
    twai_tx_queue = xQueueCreate(TWAI_TX_BUFFER_SIZE, sizeof(twai_hal_frame_t));
    HASSERT(twai_tx_queue != nullptr);
    LOG(INFO, "[TWAI] Creating TWAI RX queue: %d", TWAI_RX_BUFFER_SIZE);
    twai_rx_queue = xQueueCreate(TWAI_RX_BUFFER_SIZE, sizeof(twai_hal_frame_t));
    HASSERT(twai_rx_queue != nullptr);
}

static void create_twai_isr(void *param)
{
    Notifiable *notif = static_cast<Notifiable *>(param);
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_TWAI_INTR_SOURCE
                  , ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, twai_isr, NULL
                  , &twai_isr_handle));
#else
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_CAN_INTR_SOURCE
                  , ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, twai_isr, NULL
                  , &twai_isr_handle));
#endif
    notif->notify();
    vTaskDelete(nullptr);
}

static inline void initialize_twai()
{
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    periph_module_reset(PERIPH_TWAI_MODULE);
    periph_module_enable(PERIPH_TWAI_MODULE);
#else
    periph_module_reset(PERIPH_CAN_MODULE);
    periph_module_enable(PERIPH_CAN_MODULE);
#endif
    HASSERT(twai_hal_init(&twai_context));
    twai_timing_config_t timingCfg = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t filterCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    LOG(INFO, "[TWAI] Initiailizing TWAI peripheral");
    twai_hal_configure(&twai_context, &timingCfg, &filterCfg
                     , TWAI_DEFAULT_INTERRUPTS, 0);
    LOG(INFO, "[TWAI] Allocating TWAI ISR");
    SyncNotifiable notif;
    xTaskCreatePinnedToCore(create_twai_isr, "TWAI-ISR", 2048, &notif, 15, nullptr, APP_CPU_NUM);
    notif.wait_for_notification();
}

// Constructor.
Esp32Twai::Esp32Twai(const char *vfsPath, int rxPin, int txPin, bool report)
    : vfsPath_(vfsPath), rxPin_((gpio_num_t)rxPin), txPin_((gpio_num_t)txPin)
{
    HASSERT(GPIO_IS_VALID_GPIO(rxPin));
    HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(txPin));
    if (report)
    {
        // Create periodic automatic reload timer to print out TWAI statistics.
        twai_status_timer = xTimerCreate(vfsPath_, STATUS_PRINT_INTERVAL
                                       , pdTRUE, nullptr, report_stats);
    }
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
        twai_disable();
        twai_hal_deinit(&twai_context);
        ESP_ERROR_CHECK(esp_intr_free(twai_isr_handle));
    }
    if (twai_status_timer != nullptr)
    {
        xTimerDelete(twai_status_timer, pdMS_TO_TICKS(1));
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

#if USE_FREERTOS_BIN_SEMAPHORE
    twai_spinlock = xSemaphoreCreateBinary();
#endif

    configure_twai_gpio(txPin_, rxPin_);
    create_twai_buffers();
    initialize_twai();

    twai_is_configured = true;
    if (twai_status_timer != nullptr)
    {
        LOG(INFO, "[TWAI] Starting TWAI periodic timer");
        xTimerStart(twai_status_timer, pdMS_TO_TICKS(1));
    }
}

#endif // INCLUDE_TWAI_DRIVER