/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file constants.cxx
 * Specifies the default values for configuration constants.
 *
 * @author Balazs Racz
 * @date 30 Apr 2014
 */

#include "utils/constants.hxx"

/**
 * @addtogroup constants
 * @{
 * OpenMRN constants are used in order a adjust settings at
 * link time.  This allows for very generic implementations
 * which can later be specialized based on a particular use case.  There is
 * always a default value declared using the @ref DEFAULT_CONST macro.  The
 * default values can be overridden by using the @ref OVERRIDE_CONST macro.
 *
 * Override default for constant @ref _sym_nmranet_can_bitrate, for example:
 * @code
 *     OVERRIDE_CONST(nmranet_can_bitrate, 250000);
 * @endcode
 *
 * Notice that the "\_sym\_" prefix is left off as it is pre-pended to the
 * constant name inside the @ref OVERRIDE_CONST macro.  Typically, these
 * overrides are placed at the top of the main.cxx file or in the HwInit.cxx
 * file for a given application.
 */

/** @var _sym_nmranet_can_bitrate
 * @brief default CAN Bus data rate
 */

/** @var _sym_can2_bitrate
 * @brief default CAN Bus data rate
 */

/** @var _sym_main_thread_priority
 * @brief default priority of the main()/appl_main() thread
 */

/** @var _sym_main_thread_stack_size
 * @brief default stack size of the main()/appl_main() thread
 */

/** @var _sym_executor_max_sleep_msec
 *
 * @brief What is the longest time that the exector will sleep. After this time
 * the executor will wake up and check if there is some work to do. Catches
 * incorrectly implemented drivers that have some form of external timeout that
 * is not integrated with waking up the executor.
 */

/** @var _sym_executor_select_prescaler
 *
 * @brief How many state flows we should invoke before checking with a
 * zero-timeout select() call to see if a new input data has showed up on one
 * of the FDs. This is a tradeoff between responsiveness to the incoming data
 * vs the overhead used by the framework.
 */

/** @var _sym_can_tx_buffer_size
 * @brief default software buffer size for CAN transmission
 */

/** @var _sym_can_rx_buffer_size
 * @brief default software buffer size for CAN reception
 */

/** @var _sym_serial_tx_buffer_size
 * @brief default software buffer size for serial transmission
 */

/** @var _sym_serial_rx_buffer_size
 * @brief default software buffer size for serial reception
 */

/** @var _sym_gc_generate_newlines
 * @brief default behavior of the grid connect protocol to generate newlines.
 * This improves readability when debugging through a terminal.
 */

/** @var _sym_gridconnect_buffer_size
 *
 * @brief How many bytes we should buffer from gridconnect packets before
 * writing them to the file descriptor. Tradeoff between transmit latency and
 * efficiency of output data by using fewer packets on the lowlevel transport
 * (e.g. TCP packets or USB packets).
 */

/** @var _sym_gridconnect_buffer_delay_usec
 *
 * @brief How many microseconds we should delay outgoing gridconnect bytes in
 * the hope that we can complete the buffers.
 */

/**
 * @}
 */

DEFAULT_CONST(nmranet_can_bitrate, 125000);
DEFAULT_CONST(can2_bitrate, 125000);

DEFAULT_CONST(main_thread_priority, 0xdefa01);
DEFAULT_CONST(main_thread_stack_size, 2048);
DEFAULT_CONST(executor_max_sleep_msec, 40);
DEFAULT_CONST(executor_select_prescaler, 5);

DEFAULT_CONST(can_tx_buffer_size, 16);
DEFAULT_CONST(can_rx_buffer_size, 16);

DEFAULT_CONST(serial_tx_buffer_size, 16);
DEFAULT_CONST(serial_rx_buffer_size, 16);

DEFAULT_CONST(gc_generate_newlines, 0);

DEFAULT_CONST(gridconnect_buffer_size, 65);
DEFAULT_CONST(gridconnect_buffer_delay_usec, 300);

/// Number of pending packets per inbound gridconnect port. There is memory
/// cost associated with setting this number high.
DEFAULT_CONST(gridconnect_port_max_incoming_packets, 6);
/// 1 = infinite, do not preallocate memory
DEFAULT_CONST(gridconnect_bridge_max_incoming_packets, 1);
/// 1 = infinite
DEFAULT_CONST(gridconnect_bridge_max_outgoing_packets, 1);

DEFAULT_CONST_FALSE(gridconnect_tcp_use_select);
