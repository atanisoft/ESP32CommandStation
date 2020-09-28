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
 * TWAI driver implementation for OpenMRN. This leverages the ESP-IDF HAL API
 * rather than the TWAI driver to allow 
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
#include "executor/Notifiable.hxx"
#include "nmranet_config.h"
#include "utils/logging.h"

#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(4,1,0)
#include <soc/can_periph.h>
typedef can_dev_t ESP32_CAN_DEV_TYPE;
#else
#include <soc/twai_periph.h>
typedef twai_dev_t ESP32_CAN_DEV_TYPE;
#endif

/**
 * @brief   TWAI Constants
 */
#define TWAI_EXTD_ID_MASK               0x1FFFFFFF  /**< Bit mask for 29 bit Extended Frame Format ID */
#define TWAI_STD_ID_MASK                0x7FF       /**< Bit mask for 11 bit Standard Frame Format ID */
#define TWAI_FRAME_MAX_DLC              8           /**< Max data bytes allowed in TWAI */
#define TWAI_FRAME_EXTD_ID_LEN_BYTES    4           /**< EFF ID requires 4 bytes (29bit) */
#define TWAI_FRAME_STD_ID_LEN_BYTES     2           /**< SFF ID requires 2 bytes (11bit) */
#define TWAI_ERR_PASS_THRESH            128         /**< Error counter threshold for error passive */

/** @cond */    //Doxy command to hide preprocessor definitions from docs
/**
 * @brief   TWAI Message flags
 *
 * The message flags are used to indicate the type of message transmitted/received.
 * Some flags also specify the type of transmission.
 */
#define TWAI_MSG_FLAG_NONE              0x00        /**< No message flags (Standard Frame Format) */
#define TWAI_MSG_FLAG_EXTD              0x01        /**< Extended Frame Format (29bit ID) */
#define TWAI_MSG_FLAG_RTR               0x02        /**< Message is a Remote Frame */
#define TWAI_MSG_FLAG_SS                0x04        /**< Transmit as a Single Shot Transmission. Unused for received. */
#define TWAI_MSG_FLAG_SELF              0x08        /**< Transmit as a Self Reception Request. Unused for received. */
#define TWAI_MSG_FLAG_DLC_NON_COMP      0x10        /**< Message's Data length code is larger than 8. This will break compliance with TWAI */

/**
 * @brief Initializer macros for timing configuration structure
 *
 * The following initializer macros offer commonly found bit rates. These macros
 * place the sample point at 80% or 67% of a bit time.
 *
 * @note These timing values are based on the assumption APB clock is at 80MHz
 * @note The available bit rates are dependent on the chip target and revision.
 */
#if (TWAI_BRP_MAX > 256)
#define TWAI_TIMING_CONFIG_1KBITS()     {.brp = 4000, .tseg_1 = 15, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_5KBITS()     {.brp = 800, .tseg_1 = 15, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_10KBITS()    {.brp = 400, .tseg_1 = 15, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#endif
#if (TWAI_BRP_MAX > 128) || (CONFIG_ESP32_REV_MIN >= 2)
#define TWAI_TIMING_CONFIG_12_5KBITS()  {.brp = 256, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_16KBITS()    {.brp = 200, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_20KBITS()    {.brp = 200, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#endif
#define TWAI_TIMING_CONFIG_25KBITS()    {.brp = 128, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_50KBITS()    {.brp = 80, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_100KBITS()   {.brp = 40, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_125KBITS()   {.brp = 32, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_250KBITS()   {.brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_500KBITS()   {.brp = 8, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_800KBITS()   {.brp = 4, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_1MBITS()     {.brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}

/**
 * @brief   Initializer macro for filter configuration to accept all IDs
 */
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true}
/** @endcond */

/**
 * @brief   TWAI Controller operating modes
 */
typedef enum {
    TWAI_MODE_NORMAL,               /**< Normal operating mode where TWAI controller can send/receive/acknowledge messages */
    TWAI_MODE_NO_ACK,               /**< Transmission does not require acknowledgment. Use this mode for self testing */
    TWAI_MODE_LISTEN_ONLY,          /**< The TWAI controller will not influence the bus (No transmissions or acknowledgments) but can receive messages */
} twai_mode_t;

/**
 * @brief   Structure to store a TWAI message
 *
 * @note    The flags member is deprecated
 */
typedef struct {
    union {
        struct {
            //The order of these bits must match deprecated message flags for compatibility reasons
            uint32_t extd: 1;           /**< Extended Frame Format (29bit ID) */
            uint32_t rtr: 1;            /**< Message is a Remote Frame */
            uint32_t ss: 1;             /**< Transmit as a Single Shot Transmission. Unused for received. */
            uint32_t self: 1;           /**< Transmit as a Self Reception Request. Unused for received. */
            uint32_t dlc_non_comp: 1;   /**< Message's Data length code is larger than 8. This will break compliance with ISO 11898-1 */
            uint32_t reserved: 27;      /**< Reserved bits */
        };
        //Todo: Deprecate flags
        uint32_t flags;                 /**< Deprecated: Alternate way to set bits using message flags */
    };
    uint32_t identifier;                /**< 11 or 29 bit identifier */
    uint8_t data_length_code;           /**< Data length code */
    uint8_t data[TWAI_FRAME_MAX_DLC];    /**< Data bytes (not relevant in RTR frame) */
} twai_message_t;

/**
 * @brief   Structure for bit timing configuration of the TWAI driver
 *
 * @note    Macro initializers are available for this structure
 */
typedef struct {
    uint32_t brp;                   /**< Baudrate prescaler (i.e., APB clock divider). Any even number from 2 to 128 for ESP32, 2 to 32768 for ESP32S2.
                                         For ESP32 Rev 2 or later, multiples of 4 from 132 to 256 are also supported */
    uint8_t tseg_1;                 /**< Timing segment 1 (Number of time quanta, between 1 to 16) */
    uint8_t tseg_2;                 /**< Timing segment 2 (Number of time quanta, 1 to 8) */
    uint8_t sjw;                    /**< Synchronization Jump Width (Max time quanta jump for synchronize from 1 to 4) */
    bool triple_sampling;           /**< Enables triple sampling when the TWAI controller samples a bit */
} twai_timing_config_t;

/**
 * @brief   Structure for acceptance filter configuration of the TWAI driver (see documentation)
 *
 * @note    Macro initializers are available for this structure
 */
typedef struct {
    uint32_t acceptance_code;       /**< 32-bit acceptance code */
    uint32_t acceptance_mask;       /**< 32-bit acceptance mask */
    bool single_filter;             /**< Use Single Filter Mode (see documentation) */
} twai_filter_config_t;


/* ------------------------- Defines and Typedefs --------------------------- */

#define TWAI_LL_STATUS_RBS      (0x1 << 0)      //Receive Buffer Status
#define TWAI_LL_STATUS_DOS      (0x1 << 1)      //Data Overrun Status
#define TWAI_LL_STATUS_TBS      (0x1 << 2)      //Transmit Buffer Status
#define TWAI_LL_STATUS_TCS      (0x1 << 3)      //Transmission Complete Status
#define TWAI_LL_STATUS_RS       (0x1 << 4)      //Receive Status
#define TWAI_LL_STATUS_TS       (0x1 << 5)      //Transmit Status
#define TWAI_LL_STATUS_ES       (0x1 << 6)      //Error Status
#define TWAI_LL_STATUS_BS       (0x1 << 7)      //Bus Status

#define TWAI_LL_INTR_RI         (0x1 << 0)      //Receive Interrupt
#define TWAI_LL_INTR_TI         (0x1 << 1)      //Transmit Interrupt
#define TWAI_LL_INTR_EI         (0x1 << 2)      //Error Interrupt
//Data overrun interrupt not supported in SW due to HW peculiarities
#define TWAI_LL_INTR_EPI        (0x1 << 5)      //Error Passive Interrupt
#define TWAI_LL_INTR_ALI        (0x1 << 6)      //Arbitration Lost Interrupt
#define TWAI_LL_INTR_BEI        (0x1 << 7)      //Bus Error Interrupt

/*
 * The following frame structure has an NEARLY identical bit field layout to
 * each byte of the TX buffer. This allows for formatting and parsing frames to
 * be done outside of time critical regions (i.e., ISRs). All the ISR needs to
 * do is to copy byte by byte to/from the TX/RX buffer. The two reserved bits in
 * TX buffer are used in the frame structure to store the self_reception and
 * single_shot flags which in turn indicate the type of transmission to execute.
 */
typedef union {
    struct {
        struct {
            uint8_t dlc: 4;             //Data length code (0 to 8) of the frame
            uint8_t self_reception: 1;  //This frame should be transmitted using self reception command
            uint8_t single_shot: 1;     //This frame should be transmitted using single shot command
            uint8_t rtr: 1;             //This frame is a remote transmission request
            uint8_t frame_format: 1;    //Format of the frame (1 = extended, 0 = standard)
        };
        union {
            struct {
                uint8_t id[2];          //11 bit standard frame identifier
                uint8_t data[8];        //Data bytes (0 to 8)
                uint8_t reserved8[2];
            } standard;
            struct {
                uint8_t id[4];          //29 bit extended frame identifier
                uint8_t data[8];        //Data bytes (0 to 8)
            } extended;
        };
    };
    uint8_t bytes[13];
} __attribute__((packed)) twai_ll_frame_buffer_t;

_Static_assert(sizeof(twai_ll_frame_buffer_t) == 13, "TX/RX buffer type should be 13 bytes");

/* ---------------------------- Mode Register ------------------------------- */

/**
 * @brief   Enter reset mode
 *
 * When in reset mode, the TWAI controller is effectively disconnected from the
 * TWAI bus and will not participate in any bus activates. Reset mode is required
 * in order to write the majority of configuration registers.
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Reset mode is automatically entered on BUS OFF condition
 */
static inline void twai_ll_enter_reset_mode(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->mode_reg.rm = 1;
#else
    hw->mode_reg.reset = 1;
#endif
}

/**
 * @brief   Exit reset mode
 *
 * When not in reset mode, the TWAI controller will take part in bus activities
 * (e.g., send/receive/acknowledge messages and error frames) depending on the
 * operating mode.
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Reset mode must be exit to initiate BUS OFF recovery
 */
static inline void twai_ll_exit_reset_mode(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->mode_reg.rm = 0;
#else
    hw->mode_reg.reset = 0;
#endif
}

/**
 * @brief   Check if in reset mode
 * @param hw Start address of the TWAI registers
 * @return true if in reset mode
 */
static inline bool twai_ll_is_in_reset_mode(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    return hw->mode_reg.rm;
#else
    return hw->mode_reg.reset;
#endif
}

/**
 * @brief   Set operating mode of TWAI controller
 *
 * @param hw Start address of the TWAI registers
 * @param mode Operating mode
 *
 * @note Must be called in reset mode
 */
static inline void twai_ll_set_mode(ESP32_CAN_DEV_TYPE *hw, twai_mode_t mode)
{
    if (mode == TWAI_MODE_NORMAL) {           //Normal Operating mode
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        hw->mode_reg.lom = 0;
        hw->mode_reg.stm = 0;
#else
        hw->mode_reg.listen_only = 0;
        hw->mode_reg.self_test = 0;
#endif
    } else if (mode == TWAI_MODE_NO_ACK) {    //Self Test Mode (No Ack)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        hw->mode_reg.lom = 0;
        hw->mode_reg.stm = 1;
#else
        hw->mode_reg.listen_only = 0;
        hw->mode_reg.self_test = 1;
#endif
    } else if (mode == TWAI_MODE_LISTEN_ONLY) {       //Listen Only Mode
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        hw->mode_reg.lom = 1;
        hw->mode_reg.stm = 0;
#else
        hw->mode_reg.listen_only = 1;
        hw->mode_reg.self_test = 0;
#endif
    }
}

/* --------------------------- Command Register ----------------------------- */

/**
 * @brief   Set TX command
 *
 * Setting the TX command will cause the TWAI controller to attempt to transmit
 * the frame stored in the TX buffer. The TX buffer will be occupied (i.e.,
 * locked) until TX completes.
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Transmit commands should be called last (i.e., after handling buffer
 *       release and clear data overrun) in order to prevent the other commands
 *       overwriting this latched TX bit with 0.
 */
static inline void twai_ll_set_cmd_tx(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->command_reg.tr = 1;
#else
    hw->command_reg.tx_req = 1;
#endif
}

/**
 * @brief   Set single shot TX command
 *
 * Similar to setting TX command, but the TWAI controller will not automatically
 * retry transmission upon an error (e.g., due to an acknowledgement error).
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Transmit commands should be called last (i.e., after handling buffer
 *       release and clear data overrun) in order to prevent the other commands
 *       overwriting this latched TX bit with 0.
 */
static inline void twai_ll_set_cmd_tx_single_shot(ESP32_CAN_DEV_TYPE *hw)
{
    hw->command_reg.val = 0x03;     //Writing to TR and AT simultaneously
}

/**
 * @brief   Aborts TX
 *
 * Frames awaiting TX will be aborted. Frames already being TX are not aborted.
 * Transmission Complete Status bit is automatically set to 1.
 * Similar to setting TX command, but the TWAI controller will not automatically
 * retry transmission upon an error (e.g., due to acknowledge error).
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Transmit commands should be called last (i.e., after handling buffer
 *       release and clear data overrun) in order to prevent the other commands
 *       overwriting this latched TX bit with 0.
 */
static inline void twai_ll_set_cmd_abort_tx(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->command_reg.at = 1;
#else
    hw->command_reg.abort_tx = 1;
#endif
}

/**
 * @brief   Release RX buffer
 *
 * Rotates RX buffer to the next frame in the RX FIFO.
 *
 * @param hw Start address of the TWAI registers
 */
static inline void twai_ll_set_cmd_release_rx_buffer(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->command_reg.rrb = 1;
#else
    hw->command_reg.release_rx_buff = 1;
#endif
}

/**
 * @brief   Clear data overrun
 *
 * Clears the data overrun status bit
 *
 * @param hw Start address of the TWAI registers
 */
static inline void twai_ll_set_cmd_clear_data_overrun(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->command_reg.cdo = 1;
#else
    hw->command_reg.clear_data_overrun = 1;
#endif
}

/**
 * @brief   Set self reception single shot command
 *
 * Similar to setting TX command, but the TWAI controller also simultaneously
 * receive the transmitted frame and is generally used for self testing
 * purposes. The TWAI controller will not ACK the received message, so consider
 * using the NO_ACK operating mode.
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Transmit commands should be called last (i.e., after handling buffer
 *       release and clear data overrun) in order to prevent the other commands
 *       overwriting this latched TX bit with 0.
 */
static inline void twai_ll_set_cmd_self_rx_request(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->command_reg.srr = 1;
#else
    hw->command_reg.self_rx_req = 1;
#endif
}

/**
 * @brief   Set self reception request command
 *
 * Similar to setting the self reception request, but the TWAI controller will
 * not automatically retry transmission upon an error (e.g., due to and
 * acknowledgement error).
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Transmit commands should be called last (i.e., after handling buffer
 *       release and clear data overrun) in order to prevent the other commands
 *       overwriting this latched TX bit with 0.
 */
static inline void twai_ll_set_cmd_self_rx_single_shot(ESP32_CAN_DEV_TYPE *hw)
{
    hw->command_reg.val = 0x12;
}

/* --------------------------- Status Register ------------------------------ */

/**
 * @brief   Get all status bits
 *
 * @param hw Start address of the TWAI registers
 * @return Status bits
 */
static inline uint32_t twai_ll_get_status(ESP32_CAN_DEV_TYPE *hw)
{
    return hw->status_reg.val;
}

/**
 * @brief   Check if RX FIFO overrun status bit is set
 *
 * @param hw Start address of the TWAI registers
 * @return Overrun status bit
 */
static inline bool twai_ll_is_fifo_overrun(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    return hw->status_reg.dos;
#else
    return hw->status_reg.data_overrun;
#endif
}

/**
 * @brief   Check if previously TX was successful
 *
 * @param hw Start address of the TWAI registers
 * @return Whether previous TX was successful
 */
static inline bool twai_ll_is_last_tx_successful(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    return hw->status_reg.tcs;
#else
    return hw->status_reg.tx_complete;
#endif
}

/* -------------------------- Interrupt Register ---------------------------- */

/**
 * @brief   Get currently set interrupts
 *
 * Reading the interrupt registers will automatically clear all interrupts
 * except for the Receive Interrupt.
 *
 * @param hw Start address of the TWAI registers
 * @return Bit mask of set interrupts
 */
static inline uint32_t twai_ll_get_and_clear_intrs(ESP32_CAN_DEV_TYPE *hw)
{
    return hw->interrupt_reg.val;
}

/* ----------------------- Interrupt Enable Register ------------------------ */

/**
 * @brief   Set which interrupts are enabled
 *
 * @param hw Start address of the TWAI registers
 * @param Bit mask of interrupts to enable
 *
 * @note Must be called in reset mode
 */
static inline void twai_ll_set_enabled_intrs(ESP32_CAN_DEV_TYPE *hw, uint32_t intr_mask)
{
#if (CONFIG_ESP32_REV_MIN >= 2)
    //ESP32 Rev 2 or later has brp div field. Need to mask it out
    hw->interrupt_enable_reg.val = (hw->interrupt_enable_reg.val & 0x10) | intr_mask;
#else
    hw->interrupt_enable_reg.val = intr_mask;
#endif
}

/* ------------------------ Bus Timing Registers --------------------------- */

/**
 * @brief   Set bus timing
 *
 * @param hw Start address of the TWAI registers
 * @param brp Baud Rate Prescaler
 * @param sjw Synchronization Jump Width
 * @param tseg1 Timing Segment 1
 * @param tseg2 Timing Segment 2
 * @param triple_sampling Triple Sampling enable/disable
 *
 * @note Must be called in reset mode
 * @note ESP32 rev 2 or later can support a x2 brp by setting a brp_div bit,
 *       allowing the brp to go from a maximum of 128 to 256.
 */
static inline void twai_ll_set_bus_timing(ESP32_CAN_DEV_TYPE *hw, uint32_t brp, uint32_t sjw, uint32_t tseg1, uint32_t tseg2, bool triple_sampling)
{
#if (CONFIG_ESP32_REV_MIN >= 2)
    if (brp > TWAI_BRP_DIV_THRESH) {
        //Need to set brp_div bit
        hw->interrupt_enable_reg.brp_div = 1;
        brp /= 2;
    } else {
        hw->interrupt_enable_reg.brp_div = 0;
    }
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->bus_timing_0_reg.brp = (brp / 2) - 1;
    hw->bus_timing_0_reg.sjw = sjw - 1;
    hw->bus_timing_1_reg.tseg1 = tseg1 - 1;
    hw->bus_timing_1_reg.tseg2 = tseg2 - 1;
    hw->bus_timing_1_reg.sam = triple_sampling;
#else
    hw->bus_timing_0_reg.baud_rate_prescaler = (brp / 2) - 1;
    hw->bus_timing_0_reg.sync_jump_width = sjw - 1;
    hw->bus_timing_1_reg.time_seg_1 = tseg1 - 1;
    hw->bus_timing_1_reg.time_seg_2 = tseg2 - 1;
    hw->bus_timing_1_reg.sampling = triple_sampling;
#endif
}

/* ----------------------------- ALC Register ------------------------------- */

/**
 * @brief   Clear Arbitration Lost Capture Register
 *
 * Reading the ALC register rearms the Arbitration Lost Interrupt
 *
 * @param hw Start address of the TWAI registers
 */
static inline void twai_ll_clear_arb_lost_cap(ESP32_CAN_DEV_TYPE *hw)
{
    (void)hw->arbitration_lost_captue_reg.val;
}

/* ----------------------------- ECC Register ------------------------------- */

/**
 * @brief   Clear Error Code Capture register
 *
 * Reading the ECC register rearms the Bus Error Interrupt
 *
 * @param hw Start address of the TWAI registers
 */
static inline void twai_ll_clear_err_code_cap(ESP32_CAN_DEV_TYPE *hw)
{
    (void)hw->error_code_capture_reg.val;
}

/* ----------------------------- EWL Register ------------------------------- */

/**
 * @brief   Set Error Warning Limit
 *
 * @param hw Start address of the TWAI registers
 * @param ewl Error Warning Limit
 *
 * @note Must be called in reset mode
 */
static inline void twai_ll_set_err_warn_lim(ESP32_CAN_DEV_TYPE *hw, uint32_t ewl)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->error_warning_limit_reg.ewl = ewl;
#else
    hw->error_warning_limit_reg.val = ewl;
#endif
}

/**
 * @brief   Get Error Warning Limit
 *
 * @param hw Start address of the TWAI registers
 * @return Error Warning Limit
 */
static inline uint32_t twai_ll_get_err_warn_lim(ESP32_CAN_DEV_TYPE *hw)
{
    return hw->error_warning_limit_reg.val;
}

/* ------------------------ RX Error Count Register ------------------------- */

/**
 * @brief   Get RX Error Counter
 *
 * @param hw Start address of the TWAI registers
 * @return REC value
 *
 * @note REC is not frozen in reset mode. Listen only mode will freeze it. A BUS
 *       OFF condition automatically sets the REC to 0.
 */
static inline uint32_t twai_ll_get_rec(ESP32_CAN_DEV_TYPE *hw)
{
    return hw->rx_error_counter_reg.val;
}

/**
 * @brief   Set RX Error Counter
 *
 * @param hw Start address of the TWAI registers
 * @param rec REC value
 *
 * @note Must be called in reset mode
 */
static inline void twai_ll_set_rec(ESP32_CAN_DEV_TYPE *hw, uint32_t rec)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->rx_error_counter_reg.rxerr = rec;
#else
    hw->rx_error_counter_reg.val = rec;
#endif
}

/* ------------------------ TX Error Count Register ------------------------- */

/**
 * @brief   Get TX Error Counter
 *
 * @param hw Start address of the TWAI registers
 * @return TEC value
 *
 * @note A BUS OFF condition will automatically set this to 128
 */
static inline uint32_t twai_ll_get_tec(ESP32_CAN_DEV_TYPE *hw)
{
    return hw->tx_error_counter_reg.val;
}

/**
 * @brief   Set TX Error Counter
 *
 * @param hw Start address of the TWAI registers
 * @param tec TEC value
 *
 * @note Must be called in reset mode
 */
static inline void twai_ll_set_tec(ESP32_CAN_DEV_TYPE *hw, uint32_t tec)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->tx_error_counter_reg.txerr = tec;
#else
    hw->tx_error_counter_reg.val = tec;
#endif
}

/* ---------------------- Acceptance Filter Registers ----------------------- */

/**
 * @brief   Set Acceptance Filter
 * @param hw Start address of the TWAI registers
 * @param code Acceptance Code
 * @param mask Acceptance Mask
 * @param single_filter Whether to enable single filter mode
 *
 * @note Must be called in reset mode
 */
static inline void twai_ll_set_acc_filter(ESP32_CAN_DEV_TYPE* hw, uint32_t code, uint32_t mask, bool single_filter)
{
    uint32_t code_swapped = __builtin_bswap32(code);
    uint32_t mask_swapped = __builtin_bswap32(mask);
    for (int i = 0; i < 4; i++) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        hw->acceptance_filter.acr[i].byte = ((code_swapped >> (i * 8)) & 0xFF);
        hw->acceptance_filter.amr[i].byte = ((mask_swapped >> (i * 8)) & 0xFF);
#else
        hw->acceptance_filter.code_reg[i].byte = ((code_swapped >> (i * 8)) & 0xFF);
        hw->acceptance_filter.mask_reg[i].byte = ((mask_swapped >> (i * 8)) & 0xFF);
#endif
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->mode_reg.afm = single_filter;
#else
    hw->mode_reg.acceptance_filter = single_filter;
#endif
}

/* ------------------------- TX/RX Buffer Registers ------------------------- */

/**
 * @brief   Copy a formatted TWAI frame into TX buffer for transmission
 *
 * @param hw Start address of the TWAI registers
 * @param tx_frame Pointer to formatted frame
 *
 * @note Call twai_ll_format_frame_buffer() to format a frame
 */
static inline void twai_ll_set_tx_buffer(ESP32_CAN_DEV_TYPE *hw, twai_ll_frame_buffer_t *tx_frame)
{
    //Copy formatted frame into TX buffer
    for (int i = 0; i < 13; i++) {
        hw->tx_rx_buffer[i].val = tx_frame->bytes[i];
    }
}

/**
 * @brief   Copy a received frame from the RX buffer for parsing
 *
 * @param hw Start address of the TWAI registers
 * @param rx_frame Pointer to store formatted frame
 *
 * @note Call twai_ll_prase_frame_buffer() to parse the formatted frame
 */
static inline void twai_ll_get_rx_buffer(ESP32_CAN_DEV_TYPE *hw, twai_ll_frame_buffer_t *rx_frame)
{
    //Copy RX buffer registers into frame
    for (int i = 0; i < 13; i++) {
        rx_frame->bytes[i] =  hw->tx_rx_buffer[i].byte;
    }
}

/**
 * @brief   Format contents of a TWAI frame into layout of TX Buffer
 *
 * This function encodes a message into a frame structure. The frame structure
 * has an identical layout to the TX buffer, allowing the frame structure to be
 * directly copied into TX buffer.
 *
 * @param[in] 11bit or 29bit ID
 * @param[in] dlc Data length code
 * @param[in] data Pointer to an 8 byte array containing data. NULL if no data
 * @param[in] format Type of TWAI frame
 * @param[in] single_shot Frame will not be retransmitted on failure
 * @param[in] self_rx Frame will also be simultaneously received
 * @param[out] tx_frame Pointer to store formatted frame
 */
static inline void twai_ll_format_frame_buffer(uint32_t id, uint8_t dlc, const uint8_t *data,
                                              uint32_t flags, twai_ll_frame_buffer_t *tx_frame)
{
    bool is_extd = flags & TWAI_MSG_FLAG_EXTD;
    bool is_rtr = flags & TWAI_MSG_FLAG_RTR;

    //Set frame information
    tx_frame->dlc = dlc;
    tx_frame->frame_format = is_extd;
    tx_frame->rtr = is_rtr;
    tx_frame->self_reception = (flags & TWAI_MSG_FLAG_SELF) ? 1 : 0;
    tx_frame->single_shot = (flags & TWAI_MSG_FLAG_SS) ? 1 : 0;

    //Set ID. The ID registers are big endian and left aligned, therefore a bswap will be required
    if (is_extd) {
        uint32_t id_temp = __builtin_bswap32((id & TWAI_EXTD_ID_MASK) << 3); //((id << 3) >> 8*(3-i))
        for (int i = 0; i < 4; i++) {
            tx_frame->extended.id[i] = (id_temp >> (8 * i)) & 0xFF;
        }
    } else {
        uint32_t id_temp =  __builtin_bswap16((id & TWAI_STD_ID_MASK) << 5); //((id << 5) >> 8*(1-i))
        for (int i = 0; i < 2; i++) {
            tx_frame->standard.id[i] = (id_temp >> (8 * i)) & 0xFF;
        }
    }

    uint8_t *data_buffer = (is_extd) ? tx_frame->extended.data : tx_frame->standard.data;
    if (!is_rtr) {  //Only copy data if the frame is a data frame (i.e not RTR)
        for (int i = 0; (i < dlc) && (i < TWAI_FRAME_MAX_DLC); i++) {
            data_buffer[i] = data[i];
        }
    }
}

/**
 * @brief   Parse formatted TWAI frame (RX Buffer Layout) into its constituent contents
 *
 * @param[in] rx_frame Pointer to formatted frame
 * @param[out] id 11 or 29bit ID
 * @param[out] dlc Data length code
 * @param[out] data Data. Left over bytes set to 0.
 * @param[out] format Type of TWAI frame
 */
static inline void twai_ll_prase_frame_buffer(twai_ll_frame_buffer_t *rx_frame, uint32_t *id, uint8_t *dlc,
                                             uint8_t *data, uint32_t *flags)
{
    //Copy frame information
    *dlc = rx_frame->dlc;
    uint32_t flags_temp = 0;
    flags_temp |= (rx_frame->frame_format) ? TWAI_MSG_FLAG_EXTD : 0;
    flags_temp |= (rx_frame->rtr) ? TWAI_MSG_FLAG_RTR : 0;
    flags_temp |= (rx_frame->dlc > TWAI_FRAME_MAX_DLC) ? TWAI_MSG_FLAG_DLC_NON_COMP : 0;
    *flags = flags_temp;

    //Copy ID. The ID registers are big endian and left aligned, therefore a bswap will be required
    if (rx_frame->frame_format) {
        uint32_t id_temp = 0;
        for (int i = 0; i < 4; i++) {
            id_temp |= rx_frame->extended.id[i] << (8 * i);
        }
        id_temp = __builtin_bswap32(id_temp) >> 3;  //((byte[i] << 8*(3-i)) >> 3)
        *id = id_temp & TWAI_EXTD_ID_MASK;
    } else {
        uint32_t id_temp = 0;
        for (int i = 0; i < 2; i++) {
            id_temp |= rx_frame->standard.id[i] << (8 * i);
        }
        id_temp = __builtin_bswap16(id_temp) >> 5;  //((byte[i] << 8*(1-i)) >> 5)
        *id = id_temp & TWAI_STD_ID_MASK;
    }

    uint8_t *data_buffer = (rx_frame->frame_format) ? rx_frame->extended.data : rx_frame->standard.data;
    //Only copy data if the frame is a data frame (i.e. not a remote frame)
    int data_length = (rx_frame->rtr) ? 0 : ((rx_frame->dlc > TWAI_FRAME_MAX_DLC) ? TWAI_FRAME_MAX_DLC : rx_frame->dlc);
    for (int i = 0; i < data_length; i++) {
        data[i] = data_buffer[i];
    }
    //Set remaining bytes of data to 0
    for (int i = data_length; i < TWAI_FRAME_MAX_DLC; i++) {
        data[i] = 0;
    }
}

/* ----------------------- RX Message Count Register ------------------------ */

/**
 * @brief   Get RX Message Counter
 *
 * @param hw Start address of the TWAI registers
 * @return RX Message Counter
 */
static inline uint32_t twai_ll_get_rx_msg_count(ESP32_CAN_DEV_TYPE *hw)
{
    return hw->rx_message_counter_reg.val;
}

/* ------------------------- Clock Divider Register ------------------------- */

/**
 * @brief   Set CLKOUT Divider and enable/disable
 *
 * Configure CLKOUT. CLKOUT is a pre-scaled version of APB CLK. Divider can be
 * 1, or any even number from 2 to 14. Set the divider to 0 to disable CLKOUT.
 *
 * @param hw Start address of the TWAI registers
 * @param divider Divider for CLKOUT. Set to 0 to disable CLKOUT
 */
static inline void twai_ll_set_clkout(ESP32_CAN_DEV_TYPE *hw, uint32_t divider)
{
    if (divider >= 2 && divider <= 14) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        hw->clock_divider_reg.co = 0;
        hw->clock_divider_reg.cd = (divider / 2) - 1;
#else
        hw->clock_divider_reg.clock_off = 0;
        hw->clock_divider_reg.clock_divider = (divider / 2) - 1;
#endif
    } else if (divider == 1) {
        //Setting the divider reg to max value (7) means a divider of 1
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        hw->clock_divider_reg.co = 0;
        hw->clock_divider_reg.cd = 7;
#else
        hw->clock_divider_reg.clock_off = 0;
        hw->clock_divider_reg.clock_divider = 7;
#endif
    } else {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
        hw->clock_divider_reg.co = 1;
        hw->clock_divider_reg.cd = 0;
#else
        hw->clock_divider_reg.clock_off = 1;
        hw->clock_divider_reg.clock_divider = 0;
#endif
    }
}

/**
 * @brief   Set register address mapping to extended mode
 *
 * Extended mode register address mapping consists of more registers and extra
 * features.
 *
 * @param hw Start address of the TWAI registers
 *
 * @note Must be called before setting any configuration
 * @note Must be called in reset mode
 */
static inline void twai_ll_enable_extended_reg_layout(ESP32_CAN_DEV_TYPE *hw)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    hw->clock_divider_reg.cm = 1;
#else
    hw->clock_divider_reg.can_mode = 1;
#endif
}

#include <stdint.h>
#include <stdbool.h>

/* ------------------------- Defines and Typedefs --------------------------- */

#define TWAI_HAL_SET_FLAG(var, flag)            ((var) |= (flag))
#define TWAI_HAL_RESET_FLAG(var, flag)          ((var) &= ~(flag))

//HAL state flags
#define TWAI_HAL_STATE_FLAG_RUNNING             (1 << 0)    //Controller is active (not in reset mode)
#define TWAI_HAL_STATE_FLAG_RECOVERING          (1 << 1)    //Bus is undergoing bus recovery
#define TWAI_HAL_STATE_FLAG_ERR_WARN            (1 << 2)    //TEC or REC is >= error warning limit
#define TWAI_HAL_STATE_FLAG_ERR_PASSIVE         (1 << 3)    //TEC or REC is >= 128
#define TWAI_HAL_STATE_FLAG_BUS_OFF             (1 << 4)    //Bus-off due to TEC >= 256
#define TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED    (1 << 5)    //Transmit buffer is occupied

//Error active interrupt related
#define TWAI_HAL_EVENT_BUS_OFF                  (1 << 0)
#define TWAI_HAL_EVENT_BUS_RECOV_CPLT           (1 << 1)
#define TWAI_HAL_EVENT_BUS_RECOV_PROGRESS       (1 << 2)
#define TWAI_HAL_EVENT_ABOVE_EWL                (1 << 3)
#define TWAI_HAL_EVENT_BELOW_EWL                (1 << 4)
#define TWAI_HAL_EVENT_ERROR_PASSIVE            (1 << 5)
#define TWAI_HAL_EVENT_ERROR_ACTIVE             (1 << 6)
#define TWAI_HAL_EVENT_BUS_ERR                  (1 << 7)
#define TWAI_HAL_EVENT_ARB_LOST                 (1 << 8)
#define TWAI_HAL_EVENT_RX_BUFF_FRAME            (1 << 9)
#define TWAI_HAL_EVENT_TX_BUFF_FREE             (1 << 10)

typedef struct {
    ESP32_CAN_DEV_TYPE *dev;
    uint32_t state_flags;
} twai_hal_context_t;

typedef twai_ll_frame_buffer_t twai_hal_frame_t;

/* ---------------------------- Init and Config ----------------------------- */

//Default values written to various registers on initialization
#define TWAI_HAL_INIT_TEC    0
#define TWAI_HAL_INIT_REC    0
#define TWAI_HAL_INIT_EWL    96

/**
 * @brief Initialize TWAI peripheral and HAL context
 *
 * Sets HAL context, puts TWAI peripheral into reset mode, then sets some
 * registers with default values.
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successfully initialized, false otherwise.
 */
static inline bool twai_hal_init(twai_hal_context_t *hal_ctx)
{
    //Initialize HAL context
#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(4,1,0)
    hal_ctx->dev = (ESP32_CAN_DEV_TYPE*)&CAN;
#else
    hal_ctx->dev = (ESP32_CAN_DEV_TYPE*)&TWAI;
#endif
    hal_ctx->state_flags = 0;
    //Initialize TWAI controller, and set default values to registers
    twai_ll_enter_reset_mode(hal_ctx->dev);
    if (!twai_ll_is_in_reset_mode(hal_ctx->dev)) {    //Must enter reset mode to write to config registers
        return false;
    }
    twai_ll_enable_extended_reg_layout(hal_ctx->dev);        //Changes the address layout of the registers
    twai_ll_set_mode(hal_ctx->dev, TWAI_MODE_LISTEN_ONLY);    //Freeze REC by changing to LOM mode
    //Both TEC and REC should start at 0
    twai_ll_set_tec(hal_ctx->dev, TWAI_HAL_INIT_TEC);
    twai_ll_set_rec(hal_ctx->dev, TWAI_HAL_INIT_REC);
    twai_ll_set_err_warn_lim(hal_ctx->dev, TWAI_HAL_INIT_EWL);    //Set default value of for EWL
    return true;
}

/**
 * @brief Deinitialize the TWAI peripheral and HAL context
 *
 * Clears any unhandled interrupts and unsets HAL context
 *
 * @param hal_ctx Context of the HAL layer
 */
static inline void twai_hal_deinit(twai_hal_context_t *hal_ctx)
{
    //Clear any pending registers
    (void) twai_ll_get_and_clear_intrs(hal_ctx->dev);
    twai_ll_set_enabled_intrs(hal_ctx->dev, 0);
    twai_ll_clear_arb_lost_cap(hal_ctx->dev);
    twai_ll_clear_err_code_cap(hal_ctx->dev);
    hal_ctx->dev = NULL;
}

/**
 * @brief Configure the TWAI peripheral
 *
 * @param hal_ctx Context of the HAL layer
 * @param t_config Pointer to timing configuration structure
 * @param f_config Pointer to filter configuration structure
 * @param intr_mask Mask of interrupts to enable
 * @param clkout_divider Clock divider value for CLKOUT. Set to -1 to disable CLKOUT
 */
static inline void twai_hal_configure(twai_hal_context_t *hal_ctx, const twai_timing_config_t *t_config, const twai_filter_config_t *f_config, uint32_t intr_mask, uint32_t clkout_divider)
{
    //Configure bus timing, acceptance filter, CLKOUT, and interrupts
    twai_ll_set_bus_timing(hal_ctx->dev, t_config->brp, t_config->sjw, t_config->tseg_1, t_config->tseg_2, t_config->triple_sampling);
    twai_ll_set_acc_filter(hal_ctx->dev, f_config->acceptance_code, f_config->acceptance_mask, f_config->single_filter);
    twai_ll_set_clkout(hal_ctx->dev, clkout_divider);
    twai_ll_set_enabled_intrs(hal_ctx->dev, intr_mask);
    (void) twai_ll_get_and_clear_intrs(hal_ctx->dev);    //Clear any latched interrupts
}

/* -------------------------------- Actions --------------------------------- */

/**
 * @brief Start the TWAI peripheral
 *
 * Start the TWAI peripheral by configuring its operating mode, then exiting
 * reset mode so that the TWAI peripheral can participate in bus activities.
 *
 * @param hal_ctx Context of the HAL layer
 * @param mode Operating mode
 */
static inline void twai_hal_start(twai_hal_context_t *hal_ctx, twai_mode_t mode)
{
    twai_ll_set_mode(hal_ctx->dev, mode);                //Set operating mode
    (void) twai_ll_get_and_clear_intrs(hal_ctx->dev);    //Clear any latched interrupts
    TWAI_HAL_SET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_RUNNING);
    twai_ll_exit_reset_mode(hal_ctx->dev); 
}

/**
 * @brief Stop the TWAI peripheral
 *
 * Stop the TWAI peripheral by entering reset mode to stop any bus activity, then
 * setting the operating mode to Listen Only so that REC is frozen.
 *
 * @param hal_ctx Context of the HAL layer
 */
static inline void twai_hal_stop(twai_hal_context_t *hal_ctx)
{
    twai_ll_enter_reset_mode(hal_ctx->dev);
    (void) twai_ll_get_and_clear_intrs(hal_ctx->dev);
    twai_ll_set_mode(hal_ctx->dev, TWAI_MODE_LISTEN_ONLY);    //Freeze REC by changing to LOM mode
    //Any TX is immediately halted on entering reset mode
    TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
    TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_RUNNING);
}

/**
 * @brief Start bus recovery
 *
 * @param hal_ctx Context of the HAL layer
 */
static inline void twai_hal_start_bus_recovery(twai_hal_context_t *hal_ctx)
{
    TWAI_HAL_SET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_RECOVERING);
    twai_ll_exit_reset_mode(hal_ctx->dev);
}

/**
 * @brief Get the value of the TX Error Counter
 *
 * @param hal_ctx Context of the HAL layer
 * @return TX Error Counter Value
 */
static inline uint32_t twai_hal_get_tec(twai_hal_context_t *hal_ctx)
{
    return twai_ll_get_tec((hal_ctx)->dev);
}

/**
 * @brief Get the value of the RX Error Counter
 *
 * @param hal_ctx Context of the HAL layer
 * @return RX Error Counter Value
 */
static inline uint32_t twai_hal_get_rec(twai_hal_context_t *hal_ctx)
{
    return twai_ll_get_rec((hal_ctx)->dev);
}

/**
 * @brief Get the RX message count register
 *
 * @param hal_ctx Context of the HAL layer
 * @return RX message count
 */
static inline uint32_t twai_hal_get_rx_msg_count(twai_hal_context_t *hal_ctx)
{
    return twai_ll_get_rx_msg_count((hal_ctx)->dev);
}

/**
 * @brief Check if the last transmitted frame was successful
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successful
 */
static inline bool twai_hal_check_last_tx_successful(twai_hal_context_t *hal_ctx)
{
    return twai_ll_is_last_tx_successful((hal_ctx)->dev);
}

/**
 * @brief Check if certain HAL state flags are set
 *
 * The HAL will maintain a record of the controller's state via a set of flags.
 * These flags are automatically maintained (i.e., set and reset) inside various
 * HAL function calls. This function checks if certain flags are currently set.
 *
 * @param hal_ctx Context of the HAL layer
 * @param check_flags Bit mask of flags to check
 * @return True if one or more of the flags in check_flags are set
 */

static inline bool twai_hal_check_state_flags(twai_hal_context_t *hal_ctx, uint32_t check_flags)
{
    return hal_ctx->state_flags & check_flags;
}

/* ----------------------------- Event Handling ----------------------------- */

/**
 * @brief Decode current events that triggered an interrupt
 *
 * This function should be the called at the beginning of an ISR. This
 * function will do the following:
 * - Read and clear interrupts
 * - Decode current events that triggered an interrupt
 * - Respond to low latency interrupt events
 *      - Bus off: Change to LOM to free TEC/REC
 *      - Recovery complete: Enter reset mode
 *      - Clear ECC and ALC
 * - Update state flags based on events that have occurred.
 *
 * @param hal_ctx Context of the HAL layer
 * @return Bit mask of events that have occurred
 */
static inline uint32_t twai_hal_decode_interrupt_events(twai_hal_context_t *hal_ctx)
{
    uint32_t events = 0;
    //Read interrupt, status
    uint32_t interrupts = twai_ll_get_and_clear_intrs(hal_ctx->dev);
    uint32_t status = twai_ll_get_status(hal_ctx->dev);
    uint32_t tec = twai_ll_get_tec(hal_ctx->dev);
    uint32_t rec = twai_ll_get_rec(hal_ctx->dev);

    //Error Warning Interrupt set whenever Error or Bus Status bit changes
    if (interrupts & TWAI_LL_INTR_EI) {
        if (status & TWAI_LL_STATUS_BS) {
            //Currently in BUS OFF state
            if (status & TWAI_LL_STATUS_ES) {    //EWL is exceeded, thus must have entered BUS OFF
                twai_ll_set_mode(hal_ctx->dev, TWAI_MODE_LISTEN_ONLY);  //Set to listen only to freeze tec and rec
                events |= TWAI_HAL_EVENT_BUS_OFF;
                TWAI_HAL_SET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_BUS_OFF);
                TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_RUNNING);
                //Any TX would have been halted by entering bus off. Reset its flag
                TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
            } else {
                //Below EWL. Therefore TEC is counting down in bus recovery
                events |= TWAI_HAL_EVENT_BUS_RECOV_PROGRESS;
            }
        } else {
            //Not in BUS OFF
            if (status & TWAI_LL_STATUS_ES) {       //Just Exceeded EWL
                events |= TWAI_HAL_EVENT_ABOVE_EWL;  
                TWAI_HAL_SET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_ERR_WARN);
            } else if (hal_ctx->state_flags & TWAI_HAL_STATE_FLAG_RECOVERING) {
                //Previously undergoing bus recovery. Thus means bus recovery complete
                twai_ll_enter_reset_mode(hal_ctx->dev);     //Enter reset mode to stop the peripheral
                events |= TWAI_HAL_EVENT_BUS_RECOV_CPLT;
                TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_RECOVERING);
                TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_BUS_OFF);
            } else {        //Just went below EWL
                events |= TWAI_HAL_EVENT_BELOW_EWL;
                TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_ERR_WARN);
            }
        }
    }
    //Receive Interrupt set whenever RX FIFO  is not empty
    if (interrupts & TWAI_LL_INTR_RI) {
        events |= TWAI_HAL_EVENT_RX_BUFF_FRAME;
    }
    //Transmit interrupt set whenever TX buffer becomes free
    if (interrupts & TWAI_LL_INTR_TI) {
        events |= TWAI_HAL_EVENT_TX_BUFF_FREE;
        TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
    }
    //Error Passive Interrupt on transition from error active to passive or vice versa
    if (interrupts & TWAI_LL_INTR_EPI) {
        if (tec >= TWAI_ERR_PASS_THRESH || rec >= TWAI_ERR_PASS_THRESH) {
            events |= TWAI_HAL_EVENT_ERROR_PASSIVE;
            TWAI_HAL_SET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_ERR_PASSIVE);
        } else {
            events |= TWAI_HAL_EVENT_ERROR_ACTIVE;
            TWAI_HAL_RESET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_ERR_PASSIVE);
        }
    }
    //Bus error interrupt triggered on a bus error (e.g. bit, ACK, stuff etc)
    if (interrupts & TWAI_LL_INTR_BEI) {
        twai_ll_clear_err_code_cap(hal_ctx->dev);
        events |= TWAI_HAL_EVENT_BUS_ERR;
    }
    //Arbitration Lost Interrupt triggered on losing arbitration
    if (interrupts & TWAI_LL_INTR_ALI) {
        twai_ll_clear_arb_lost_cap(hal_ctx->dev);
        events |= TWAI_HAL_EVENT_ARB_LOST;
    }
    return events;
}

/* ------------------------------- TX and RX -------------------------------- */

/**
 * @brief Format a TWAI Frame
 *
 * This function takes a TWAI message structure (containing ID, DLC, data, and
 * flags) and formats it to match the layout of the TX frame buffer.
 *
 * @param message Pointer to TWAI message
 * @param frame Pointer to empty frame structure
 */
static inline void twai_hal_format_frame(const twai_message_t *message, twai_hal_frame_t *frame)
{
    //Direct call to ll function
    twai_ll_format_frame_buffer(message->identifier, message->data_length_code, message->data,
                               message->flags, frame);
}

/**
 * @brief Parse a TWAI Frame
 *
 * This function takes a TWAI frame (in the format of the RX frame buffer) and
 * parses it to a TWAI message (containing ID, DLC, data and flags).
 *
 * @param frame Pointer to frame structure
 * @param message Pointer to empty message structure
 */
static inline void twai_hal_parse_frame(twai_hal_frame_t *frame, twai_message_t *message)
{
    //Direct call to ll function
    twai_ll_prase_frame_buffer(frame, &message->identifier, &message->data_length_code,
                              message->data, &message->flags);
}

/**
 * @brief Copy a frame into the TX buffer and transmit
 *
 * This function copies a formatted TX frame into the TX buffer, and the
 * transmit by setting the correct transmit command (e.g. normal, single shot,
 * self RX) in the command register.
 *
 * @param hal_ctx Context of the HAL layer
 * @param tx_frame Pointer to structure containing formatted TX frame
 */
static inline void twai_hal_set_tx_buffer_and_transmit(twai_hal_context_t *hal_ctx, twai_hal_frame_t *tx_frame)
{
    //Copy frame into tx buffer
    twai_ll_set_tx_buffer(hal_ctx->dev, tx_frame);
    //Hit the send command
    if (tx_frame->self_reception) {
        if (tx_frame->single_shot) {
            twai_ll_set_cmd_self_rx_single_shot(hal_ctx->dev);
        } else {
            twai_ll_set_cmd_self_rx_request(hal_ctx->dev);
        }
    } else if (tx_frame->single_shot){
        twai_ll_set_cmd_tx_single_shot(hal_ctx->dev);
    } else {
        twai_ll_set_cmd_tx(hal_ctx->dev);
    }
    TWAI_HAL_SET_FLAG(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
}

/**
 * @brief Copy a frame from the RX buffer and release
 *
 * This function copies a frame from the RX buffer, then release the buffer (so
 * that it loads the next frame in the RX FIFO).
 *
 * @param hal_ctx Context of the HAL layer
 * @param rx_frame Pointer to structure to store RX frame
 */
static inline void twai_hal_read_rx_buffer_and_clear(twai_hal_context_t *hal_ctx, twai_hal_frame_t *rx_frame)
{
    twai_ll_get_rx_buffer(hal_ctx->dev, rx_frame);
    twai_ll_set_cmd_release_rx_buffer(hal_ctx->dev);
    /*
     * Todo: Support overrun handling by:
     * - Check overrun status bit. Return false if overrun
     */
}

static constexpr int TWAI_VFS_FD = 0;

/// Interval at which to print the ESP32 TWAI bus status.
static constexpr TickType_t STATUS_PRINT_INTERVAL = pdMS_TO_TICKS(10000);

static const uint16_t TWAI_RX_BUFFER_SIZE = ((uint8_t)config_can_rx_buffer_size()) * 4;
static const uint16_t TWAI_TX_BUFFER_SIZE = (uint8_t)config_can_tx_buffer_size();

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

static portMUX_TYPE twai_spinlock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t twai_select_pending = 0;
static esp_vfs_select_sem_t twai_select_sem;
static Notifiable* twai_tx_notifiable = nullptr;
static Notifiable* twai_rx_notifiable = nullptr;

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

    portENTER_CRITICAL(&twai_spinlock);
    twai_hal_start(&twai_context, TWAI_MODE_NORMAL);
    portEXIT_CRITICAL(&twai_spinlock);
}

/// Disables the low-level TWAI driver.
static inline void twai_disable()
{
    portENTER_CRITICAL(&twai_spinlock);
    twai_hal_stop(&twai_context);
    portEXIT_CRITICAL(&twai_spinlock);

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
            portENTER_CRITICAL(&twai_spinlock);
            twai_hal_start_bus_recovery(&twai_context);
            portEXIT_CRITICAL(&twai_spinlock);
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
            portENTER_CRITICAL(&twai_spinlock);
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
            portEXIT_CRITICAL(&twai_spinlock);
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
        portENTER_CRITICAL(&twai_spinlock);
        twai_rx_pending--;
        portEXIT_CRITICAL(&twai_spinlock);

        if (hal_frame.dlc > TWAI_FRAME_MAX_DLC)
        {
            twai_rx_discard_count++;
            continue;
        }
        twai_message_t rx_frame;
        memset(&rx_frame, 0, sizeof(twai_message_t));
        twai_hal_parse_frame(&hal_frame, &rx_frame);
        memset(data, 0, sizeof(struct can_frame));
        memcpy(data->data, rx_frame.data, rx_frame.data_length_code);
        data->can_dlc = rx_frame.data_length_code;
        data->can_id = rx_frame.identifier;
        if (rx_frame.extd)
        {
            SET_CAN_FRAME_EFF(*data);
        }
        if (rx_frame.rtr)
        {
            SET_CAN_FRAME_RTR(*data);
        }
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
    portENTER_CRITICAL(&twai_spinlock);
    twai_select_pending = 1;
    twai_select_sem = sem;
    portEXIT_CRITICAL(&twai_spinlock);

    return ESP_OK;
}

/// VFS interface helper invoked when select() is woken up.
///
/// @param end_select_args is any arguments provided in vfs_start_select().
/// @return ESP_OK for success.
static esp_err_t vfs_end_select(void *end_select_args)
{
    portENTER_CRITICAL(&twai_spinlock);
    twai_select_pending = 0;
    portEXIT_CRITICAL(&twai_spinlock);
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
        portENTER_CRITICAL(&twai_spinlock);
        if (cmd == CAN_IOC_WRITE_ACTIVE && twai_tx_pending == 0)
        {
            std::swap(n, twai_tx_notifiable);
        }
        else if (cmd == CAN_IOC_READ_ACTIVE && twai_rx_pending == 0)
        {
            std::swap(n, twai_rx_notifiable);
        }
        portEXIT_CRITICAL(&twai_spinlock);

        if (n)
        {
            n->notify();
        }
    }
    return 0;
}

static IRAM_ATTR void twai_isr(void *arg)
{
    BaseType_t wakeup = pdFALSE;

    portENTER_CRITICAL_ISR(&twai_spinlock);
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
            if (xQueueSendFromISR(twai_rx_queue, &frame, &wakeup) == pdTRUE)
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
    }
    portEXIT_CRITICAL_ISR(&twai_spinlock);

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
    portENTER_CRITICAL(&twai_spinlock);
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
    portEXIT_CRITICAL(&twai_spinlock);

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
    vfs.ioctl = vfs_ioctl;
    vfs.start_select = vfs_start_select;
    vfs.end_select = vfs_end_select;
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    ESP_ERROR_CHECK(esp_vfs_register(vfsPath_, &vfs, this));
    vfs_is_registered = true;

    LOG(INFO, "[TWAI] Configuring TWAI TX pin: %d", txPin_);

    gpio_set_pull_mode(txPin_, GPIO_FLOATING);
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    gpio_matrix_out(txPin_, TWAI_TX_IDX, false, false);
#else
    gpio_matrix_out(txPin_, CAN_TX_IDX, false, false);
#endif
    gpio_pad_select_gpio(txPin_);

    LOG(INFO, "[TWAI] Configuring TWAI RX pin: %d", rxPin_);
    gpio_set_pull_mode(rxPin_, GPIO_FLOATING);
    gpio_set_direction(rxPin_, GPIO_MODE_INPUT);
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    gpio_matrix_in(rxPin_, TWAI_RX_IDX, false);
#else
    gpio_matrix_in(rxPin_, CAN_RX_IDX, false);
#endif
    gpio_pad_select_gpio(rxPin_);

    LOG(INFO, "[TWAI] Creating TWAI TX queue: %d", TWAI_TX_BUFFER_SIZE);
    twai_tx_queue = xQueueCreate(TWAI_TX_BUFFER_SIZE, sizeof(twai_hal_frame_t));
    HASSERT(twai_tx_queue != nullptr);
    LOG(INFO, "[TWAI] Creating TWAI RX queue: %d", TWAI_RX_BUFFER_SIZE);
    twai_rx_queue = xQueueCreate(TWAI_RX_BUFFER_SIZE, sizeof(twai_hal_frame_t));
    HASSERT(twai_rx_queue != nullptr);

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
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4,2,0)
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_TWAI_INTR_SOURCE, 0, twai_isr, NULL
                  , &twai_isr_handle));
#else
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, twai_isr, NULL
                  , &twai_isr_handle));
#endif
    twai_is_configured = true;
    if (twai_status_timer != nullptr)
    {
        LOG(INFO, "[TWAI] Starting TWAI periodic timer");
        xTimerStart(twai_status_timer, pdMS_TO_TICKS(1));
    }
}

#endif // INCLUDE_TWAI_DRIVER