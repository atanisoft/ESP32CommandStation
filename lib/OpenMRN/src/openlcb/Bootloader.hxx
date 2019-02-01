/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file Bootloader.hxx
 *
 * A standalone NMRAnet stack with the sole purpose of reflashing a
 * microcontroller.
 *
 * @author Balazs Racz
 * @date 8 Dec 2014
 */

#ifdef _OPENLCB_BOOTLOADER_HXX_
#error Bootloader.hxx should be included only once.
#endif

/// include guard macro
#define _OPENLCB_BOOTLOADER_HXX_

#include <string.h>
#include <unistd.h>

#include "freertos/bootloader_hal.h"
#include "openlcb/Defs.hxx"
#include "openlcb/CanDefs.hxx"
#include "openlcb/DatagramDefs.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/ApplicationChecksum.hxx"
#include "can_frame.h"

namespace openlcb
{

/// State machine states for initializing the bootloader node. Represents what
/// the next outgoing packet should be.
enum InitState
{
    PICK_ALIAS = 0,
    SEND_CID_7,
    SEND_CID_6,
    SEND_CID_5,
    SEND_CID_4,
    WAIT_RID,
    SEND_RID,
    SEND_AMD,
    SEND_NMRANET_INIT,
    INITIALIZED,
};

/// Internal state of the bootloader stack.
struct BootloaderState
{
    struct can_frame input_frame;
    struct can_frame output_frame;
    unsigned input_frame_full : 1;
    unsigned output_frame_full : 1;
    unsigned request_reset : 1;
#ifdef BOOTLOADER_STREAM
    unsigned stream_pending : 1;
    unsigned stream_open : 1;
    unsigned stream_proceed_pending : 1;
#endif
#ifdef BOOTLOADER_DATAGRAM
    unsigned incoming_datagram_pending : 1;
    unsigned datagram_write_pending : 1;
#endif
    // 1 if the datagram buffer is busy
    unsigned datagram_output_pending : 1;
    // 1 if we are waiting for an incoming reply to a sent datagram
    unsigned datagram_reply_waiting : 1;

    NodeAlias alias;
    InitState init_state;

    // response datagram
    NodeAlias datagram_dst;
    uint8_t datagram_dlc;
    uint8_t datagram_offset;
    uint8_t datagram_payload[14];

    // Node that is sending us the stream of data.
    NodeAlias write_src_alias;

#ifdef BOOTLOADER_STREAM
    // stream source ID of incoming data.
    uint8_t stream_src_id;
    // What's the total length of the negotiated stream buffer.
    uint16_t stream_buffer_size;
    // How many bytes are left of the strem buffer before a continue needs to
    // be sent.
    int stream_buffer_remaining;
#endif

    // Offset of the beginning of the write buffer.
    uintptr_t write_buffer_offset;
    // Offset inside the write buffer for the next incoming data.
    unsigned write_buffer_index;
};

/// Global state variables.
extern BootloaderState state_;
BootloaderState state_;

//#define WRITE_BUFFER_SIZE 1024

#ifdef BOOTLOADER_STREAM
#ifndef WRITE_BUFFER_SIZE
/// How many bytes the bootloader should buffer before flushing to Flash. Will
/// influence the stream buffer size negotiation and thus the maximum speed
/// that the bootloading will work at.
#define WRITE_BUFFER_SIZE 1024
#endif
#else
/// How many bytes the bootloader should buffer before flushing to Flash. There
/// is no need to make this bigger than a datagram.
#define WRITE_BUFFER_SIZE 64
#endif
/// Write buffer; the OpenLCB protocol engine collects the incoming bytes into
/// this buffer and repeatedly flushes to flash.
uint8_t g_write_buffer[WRITE_BUFFER_SIZE];

/// Which OpenLCB Memory Config Space number should the bootloader export.
#define FLASH_SPACE (MemoryConfigDefs::SPACE_FIRMWARE)
/// local stream ID.
#define STREAM_ID 0x5A
}
using namespace openlcb;

extern "C" {

/// 1 if the bootloader is active, 0 if the bootloader is not active. Used by
/// unit tests instead of sleeping to avoid race conditions.
extern volatile unsigned g_bootloader_busy;
volatile unsigned g_bootloader_busy = 1;
#ifdef __linux__
Atomic g_bootloader_lock;
#endif

#ifdef BOOTLOADER_STREAM
/// Protocol support bitmask that the bootloader should export.
#define PIP_REPLY_VALUE                                                        \
    (Defs::DATAGRAM | Defs::STREAM | Defs::MEMORY_CONFIGURATION |              \
        Defs::FIRMWARE_UPGRADE_ACTIVE)
#else
/// Protocol support bitmask that the bootloader should export.
#define PIP_REPLY_VALUE                                                        \
    (Defs::DATAGRAM | Defs::MEMORY_CONFIGURATION |                             \
        Defs::FIRMWARE_UPGRADE_ACTIVE)
#endif
/// We manually convert to big-endian to store this value in .rodata.
static const uint64_t PIP_REPLY =         //
    ((PIP_REPLY_VALUE >> 40) & 0xff) |    //
    ((PIP_REPLY_VALUE >> 24) & 0xff00) |  //
    ((PIP_REPLY_VALUE >> 8) & 0xff0000) | //
    ((PIP_REPLY_VALUE & 0xff0000) << 8) | //
    ((PIP_REPLY_VALUE & 0xff00) << 24) |  //
    ((PIP_REPLY_VALUE & 0xff) << 40);

/** Clears out the stream state in state_. */
void reset_stream_state();

/// Prepares the outgoing frame buffer for a frame to be sent.
void setup_can_frame()
{
    CLR_CAN_FRAME_RTR(state_.output_frame);
    CLR_CAN_FRAME_ERR(state_.output_frame);
    SET_CAN_FRAME_EFF(state_.output_frame);
    state_.output_frame.can_dlc = 0;
    state_.output_frame_full = 1;
}

/// Sets up the outgoing frame buffer for a global OpenLCB packet.
///
/// @param mti message transmit indicator to set for the outgoing message.
///
void set_can_frame_global(Defs::MTI mti)
{
    setup_can_frame();
    uint32_t id;
    CanDefs::set_fields(&id, state_.alias, mti, CanDefs::GLOBAL_ADDRESSED,
        CanDefs::NMRANET_MSG, CanDefs::NORMAL_PRIORITY);
    SET_CAN_FRAME_ID_EFF(state_.output_frame, id);
}

/** Sets the outgoing CAN frame to addressed, destination taken from the source
 * field of the incoming message or the given alias. 
 * @param mti the MTI of the outgoing value.
 * @param alias the destination node alias; defaults to taking the alias of the
 * incoming frame.
*/
void set_can_frame_addressed(
    Defs::MTI mti, NodeAlias alias = CanDefs::get_src(
                       GET_CAN_FRAME_ID_EFF(state_.input_frame)))
{
    setup_can_frame();
    uint32_t id;
    CanDefs::set_fields(&id, state_.alias, mti, CanDefs::GLOBAL_ADDRESSED,
        CanDefs::NMRANET_MSG, CanDefs::NORMAL_PRIORITY);
    SET_CAN_FRAME_ID_EFF(state_.output_frame, id);
    state_.output_frame.can_dlc = 2;
    state_.output_frame.data[0] = (alias >> 8) & 0xf;
    state_.output_frame.data[1] = alias & 0xff;
}

/// Adds the node id ad the data payload of the outgoing can frame.
void set_can_frame_nodeid()
{
    uint64_t node_id = nmranet_nodeid();
    for (int i = 5; i >= 0; --i)
    {
        state_.output_frame.data[i] = node_id & 0xff;
        node_id >>= 8;
    }
    state_.output_frame.can_dlc = 6;
}

/// Checks whether the incoming frame contains the current (bootloader) node's
/// node_id as the data payload.
///
///
/// @return true if the current node's address is in the payload.
///
bool is_can_frame_nodeid()
{
    if (state_.input_frame.can_dlc != 6)
        return false;
    uint64_t node_id = nmranet_nodeid();
    for (int i = 5; i >= 0; --i)
    {
        if (state_.input_frame.data[i] != (node_id & 0xff))
            return false;
        node_id >>= 8;
    }
    return true;
}

/** Sets output frame dlc to 4; adds the given error code to bytes 2 and 3.
 * @param error_code send this in bytes 2 and 3 of the reply message */
void set_error_code(uint16_t error_code)
{
    state_.output_frame.can_dlc = 4;
    state_.output_frame.data[2] = error_code >> 8;
    state_.output_frame.data[3] = error_code & 0xff;
}

/// Kills the input frame and sends back a datagram rejected error message with
/// permanent error.
void reject_datagram()
{
    set_can_frame_addressed(Defs::MTI_DATAGRAM_REJECTED);
    set_error_code(Defs::ERROR_PERMANENT);
    state_.input_frame_full = 0;
}

/** Loads an unaligned 32-bit value that is network-endian. 
    @param ptr is an unaligned pointer to ram.
    @return the big-endian interpreted value at the pointed value converted to
    host endian.
 */
uint32_t load_uint32_be(const uint8_t *ptr)
{
    return (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
}

/// turns an already prepared memory config response datagram into an error
/// response.
///
/// @param error_code 16-bit OpenLCB error code to report in the response
/// datagram.
///
void add_memory_config_error_response(uint16_t error_code)
{
    state_.datagram_payload[1] |= 0x08; // Turns success into error reply.
    state_.datagram_payload[state_.datagram_dlc++] = error_code >> 8;
    state_.datagram_payload[state_.datagram_dlc++] = error_code & 0xff;
}

/// Clears out the flash write buffer with all 0xFF values.
void init_flash_write_buffer()
{
    memset(g_write_buffer, 0xff, WRITE_BUFFER_SIZE);
}

/// Translates from the logical address space of the OpenLCB memory config
/// protocol memory space to the physical address space in the flash.
///
///
/// @return true if the translation is successful, false if the address is out
/// of bounds.
///
bool normalize_write_buffer_offset()
{
    const void *flash_min;
    const void *flash_max;
    const struct app_header *app_header;
    get_flash_boundaries(&flash_min, &flash_max, &app_header);
    if (state_.write_buffer_offset >=
        ((uintptr_t)flash_max - (uintptr_t)flash_min))
    {
        return false;
    }
    state_.write_buffer_offset += (uintptr_t)flash_min;
    init_flash_write_buffer();
    return true;
}

/// Writes the flash write buffer into flash, and clears it out for continuing
/// the bootloading process. This call usually takes quite a few milliseconds.
void flush_flash_buffer()
{
    const void *address =
        reinterpret_cast<const void *>(state_.write_buffer_offset);
    const void *page_start = nullptr;
    uint32_t page_length_bytes = 0;
    get_flash_page_info(address, &page_start, &page_length_bytes);
    if (page_start == address)
    {
        // Beginning of a page -- let's do an erase.
        erase_flash_page(address);
    }
    write_flash((const void *)state_.write_buffer_offset, g_write_buffer,
        state_.write_buffer_index);
    state_.write_buffer_offset += state_.write_buffer_index;
    state_.write_buffer_index = 0;
    init_flash_write_buffer();
}

/// Decodes the memory config protocol's incoming data.
void handle_memory_config_frame()
{
    uint8_t command = state_.input_frame.data[1];
    switch (command)
    {
        case MemoryConfigDefs::COMMAND_FREEZE:
        {
            if (state_.input_frame.can_dlc < 3 ||
                state_.input_frame.data[2] != MemoryConfigDefs::SPACE_FIRMWARE)
            {
                // Invalid request.
                reject_datagram();
                set_error_code(DatagramDefs::INVALID_ARGUMENTS);
                return;
            }
            // fall through
        }
        case MemoryConfigDefs::COMMAND_ENTER_BOOTLOADER:
        {
            // Poor man's reset. Clears the entire state machine, which will
            // cause us to run the boot sequence again.
            memset(&state_, 0, sizeof(state_));
            return;
        }
        case MemoryConfigDefs::COMMAND_UNFREEZE:
        {
            if (state_.input_frame.can_dlc < 3 ||
                state_.input_frame.data[2] != MemoryConfigDefs::SPACE_FIRMWARE)
            {
                // Invalid request.
                reject_datagram();
                set_error_code(DatagramDefs::INVALID_ARGUMENTS);
                return;
            }
            uint16_t r = flash_complete();
            if (r != 0) {
                // Invalid request.
                reject_datagram();
                set_error_code(r);
                return;
            }
            // fall through
        }
        case MemoryConfigDefs::COMMAND_RESET:
        {
            set_can_frame_addressed(Defs::MTI_DATAGRAM_OK);
            state_.request_reset = 1;
            state_.input_frame_full = 0;
            return;
        }
#ifdef BOOTLOADER_DATAGRAM
        case MemoryConfigDefs::COMMAND_WRITE:
        {
            NodeAlias src =
                CanDefs::get_src(GET_CAN_FRAME_ID_EFF(state_.input_frame));
            if (state_.incoming_datagram_pending)
            {
                reject_datagram();
                if (src == state_.write_src_alias)
                {
                    set_error_code(DatagramDefs::OUT_OF_ORDER);
                }
                else
                {
                    set_error_code(DatagramDefs::BUFFER_UNAVAILABLE);
                }
                return;
            }
            if (state_.input_frame.can_dlc < 8)
            {
                reject_datagram();
                set_error_code(Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
                return;
            }
            if (state_.input_frame.data[6] != FLASH_SPACE)
            {
                reject_datagram();
                set_error_code(MemoryConfigDefs::ERROR_SPACE_NOT_KNOWN);
                return;
            }

            state_.write_buffer_offset =
                load_uint32_be(state_.input_frame.data + 2);
            if (!normalize_write_buffer_offset()) {
                reject_datagram();
                set_error_code(MemoryConfigDefs::ERROR_OUT_OF_BOUNDS);
                return;
            }

            state_.incoming_datagram_pending = 1;
            state_.datagram_write_pending = 1;
            state_.write_src_alias =
                CanDefs::get_src(GET_CAN_FRAME_ID_EFF(state_.input_frame));

            g_write_buffer[0] = state_.input_frame.data[7];
            state_.write_buffer_index = 1;

            if (CanDefs::get_can_frame_type(GET_CAN_FRAME_ID_EFF(
                    state_.input_frame)) == CanDefs::DATAGRAM_ONE_FRAME)
            {
                // We also need to finish writing here.
                flush_flash_buffer();
                set_can_frame_addressed(Defs::MTI_DATAGRAM_OK);
            }

            state_.input_frame_full = 0;
            return;
        }
#endif
#ifdef BOOTLOADER_STREAM
        case MemoryConfigDefs::COMMAND_WRITE_STREAM:
        {
            if (state_.datagram_output_pending || state_.stream_open)
            {
                // No buffer for response datagram or we are busy
                reject_datagram();
                set_error_code(DatagramDefs::BUFFER_UNAVAILABLE);
                return;
            }
            if (state_.input_frame.can_dlc < 7)
            {
                // Invalid request.
                reject_datagram();
                set_error_code(DatagramDefs::INVALID_ARGUMENTS);
                return;
            }
            // Replies OK.
            set_can_frame_addressed(Defs::MTI_DATAGRAM_OK);
            state_.input_frame_full = 0;

            // Composes write stream reply datagram.
            state_.datagram_dlc = state_.input_frame.can_dlc - 1;
            memcpy(state_.datagram_payload, state_.input_frame.data,
                state_.input_frame.can_dlc - 1);
            state_.datagram_payload[1] |= MemoryConfigDefs::COMMAND_WRITE_REPLY;
            state_.datagram_output_pending = 1;
            state_.output_frame.data[state_.output_frame.can_dlc++] =
                DatagramDefs::REPLY_PENDING;
            state_.datagram_dst =
                CanDefs::get_src(GET_CAN_FRAME_ID_EFF(state_.input_frame));
            state_.datagram_offset = 0;

            if (state_.input_frame.data[6] != FLASH_SPACE)
            {
                add_memory_config_error_response(DatagramDefs::UNIMPLEMENTED);
                return;
            }
            state_.stream_pending = 1;
            state_.write_src_alias = state_.datagram_dst;
            state_.stream_src_id = state_.input_frame.data[7];
            state_.write_buffer_index = 0;
            state_.write_buffer_offset =
                load_uint32_be(state_.input_frame.data + 2);
            if (!normalize_write_buffer_offset())
            {
                add_memory_config_error_response(
                    DatagramDefs::INVALID_ARGUMENTS);
                return reset_stream_state();
            }
            return;
        }
#endif
    } // switch
    reject_datagram();
    set_error_code(DatagramDefs::UNIMPLEMENTED);
    return;
}

/// Handles incoming stream data complete message.
void handle_stream_complete();

/// Handles incoming addressed message (non-datagram).
void handle_addressed_message(Defs::MTI mti)
{
    switch (mti)
    {
        case Defs::MTI_PROTOCOL_SUPPORT_INQUIRY:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_addressed(Defs::MTI_PROTOCOL_SUPPORT_REPLY);
            state_.output_frame.can_dlc = 8;
            memcpy(state_.output_frame.data + 2, &PIP_REPLY, 6);
            break;
        }
        case Defs::MTI_VERIFY_NODE_ID_ADDRESSED:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_global(Defs::MTI_VERIFIED_NODE_ID_NUMBER);
            set_can_frame_nodeid();
            state_.input_frame_full = 0;
            return;
        }
        case Defs::MTI_DATAGRAM_OK:
        {
            if (state_.datagram_reply_waiting)
            {
                state_.datagram_reply_waiting = 0;
                state_.datagram_output_pending = 0;
            }
            break;
        }
#ifdef BOOTLOADER_STREAM
        case Defs::MTI_STREAM_INITIATE_REQUEST:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            if (state_.input_frame.can_dlc < 7)
            {
                set_can_frame_addressed(Defs::MTI_TERMINATE_DUE_TO_ERROR);
                set_error_code(DatagramDefs::INVALID_ARGUMENTS);
                break;
            }
            set_can_frame_addressed(Defs::MTI_STREAM_INITIATE_REPLY);
            state_.output_frame.can_dlc = 8;
            state_.output_frame.data[6] = state_.input_frame.data[6];
            state_.output_frame.data[7] = STREAM_ID;
            if (!state_.stream_pending ||
                state_.input_frame.data[6] != state_.stream_src_id)
            {
                // Request out of the blue. Reject.
                state_.output_frame.data[2] = 0;
                state_.output_frame.data[3] = 0;
                state_.output_frame.data[4] =
                    0b01000010; // permanent error, "should not happen"
                // invalid stream request
                state_.output_frame.data[5] = 0b00100000;
            }
            else
            {
                uint16_t proposed_size = (state_.input_frame.data[2] << 8) |
                    state_.input_frame.data[3];
                uint16_t final_size = WRITE_BUFFER_SIZE;
                while (final_size > proposed_size)
                {
                    final_size >>= 1;
                }
                state_.stream_buffer_size = final_size;
                state_.stream_buffer_remaining = state_.stream_buffer_size;
                state_.output_frame.data[2] = state_.stream_buffer_size >> 8;
                state_.output_frame.data[3] = state_.stream_buffer_size & 0xff;
                state_.output_frame.data[4] = 0x80; // accept, no type id.
                state_.output_frame.data[5] = 0x00;

                state_.stream_pending = 0;
                state_.stream_open = 1;
            }
            break;
        }
        case Defs::MTI_STREAM_COMPLETE:
        {
            return handle_stream_complete();
        }
#endif
        default:
        {
            // Send reject.
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_addressed(Defs::MTI_OPTIONAL_INTERACTION_REJECTED);
            set_error_code(DatagramDefs::UNIMPLEMENTED);
            break;
        }
    }
    state_.input_frame_full = 0;
    return;
}

/// Handles incoming global message.
void handle_global_message(Defs::MTI mti)
{
    switch (mti)
    {
        case Defs::MTI_VERIFY_NODE_ID_GLOBAL:
        {
            if (state_.output_frame_full)
            {
                // No buffer. Re-try next round.
                return;
            }
            set_can_frame_global(Defs::MTI_VERIFIED_NODE_ID_NUMBER);
            set_can_frame_nodeid();
            state_.input_frame_full = 0;
            return;
        }
    default:
        break;
    }
    // Drop to the floor.
    state_.input_frame_full = 0;
    /// @todo(balazs.racz) what about global identify messages?
    return;
}

#ifdef BOOTLOADER_STREAM
/** Clears out the stream state in state_. */
void reset_stream_state()
{
    state_.stream_open = 0;
    state_.stream_pending = 0;
    state_.stream_proceed_pending = 0;
    state_.stream_src_id = 0;
    state_.write_src_alias = 0;
    state_.stream_buffer_size = 0;
    state_.stream_buffer_remaining = 0;
    state_.write_buffer_index = 0;
    state_.write_buffer_offset = 0;
}

void handle_stream_data()
{
    if (!state_.stream_open || state_.input_frame.data[0] != STREAM_ID)
    {
        // no or wrong stream -- reject.
        if (state_.output_frame_full)
        {
            return; // re-try.
        }
        set_can_frame_addressed(Defs::MTI_TERMINATE_DUE_TO_ERROR);
        set_error_code(Defs::ERROR_TEMPORARY);
        state_.input_frame_full = 0;
        return;
    }
    int len = state_.input_frame.can_dlc - 1;
    if (WRITE_BUFFER_SIZE < state_.write_buffer_index + len)
    {
        if (state_.output_frame_full)
        {
            return; // re-try.
        }
        set_can_frame_addressed(Defs::MTI_TERMINATE_DUE_TO_ERROR);
        set_error_code(Defs::ERROR_TEMPORARY);
        state_.input_frame_full = 0;
        return;
    }
    state_.input_frame_full = 0;
    memcpy(&g_write_buffer[state_.write_buffer_index],
        &state_.input_frame.data[1], len);
    state_.write_buffer_index += len;
    state_.stream_buffer_remaining -= len;
    if (state_.stream_buffer_remaining <= 0)
    {
        state_.stream_proceed_pending = 1;
        state_.stream_buffer_remaining += state_.stream_buffer_size;
    }
    if (state_.write_buffer_index >= WRITE_BUFFER_SIZE)
    {
        flush_flash_buffer();
    }
}

void handle_stream_complete()
{
    if (!state_.stream_open ||
        state_.input_frame.data[2] != state_.stream_src_id ||
        state_.input_frame.data[3] != STREAM_ID ||
        CanDefs::get_src(GET_CAN_FRAME_ID_EFF(state_.input_frame)) !=
            state_.write_src_alias)
    {
        // no or wrong stream -- reject.
        if (state_.output_frame_full)
        {
            return; // re-try.
        }
        set_can_frame_addressed(Defs::MTI_TERMINATE_DUE_TO_ERROR);
        set_error_code(Defs::ERROR_TEMPORARY);
        state_.input_frame_full = 0;
        // We might be leaking the stream here. If the src id is actually
        // correct, and we send this termination command, we might never get
        // the stream close command ever again. The caller will have to reset
        // the node to be able to start a new stream.  On the other hand if the
        // message comes from an unexpected source, then this is a very valid
        // thing to do and we might be getting more data from the original
        // source.
        return;
    }
    if (state_.write_buffer_index)
    {
        flush_flash_buffer();
    }
    // Input frame is processed.
    state_.input_frame_full = 0;
    reset_stream_state();
}
#endif

/// Handles an incoming CAN frame.
void handle_input_frame()
{
    if (IS_CAN_FRAME_ERR(state_.input_frame) ||
        IS_CAN_FRAME_RTR(state_.input_frame) ||
        !IS_CAN_FRAME_EFF(state_.input_frame))
    {
        state_.input_frame_full = 0;
        return;
    }
    uint32_t can_id = GET_CAN_FRAME_ID_EFF(state_.input_frame);
    int dlc = state_.input_frame.can_dlc;
    if (CanDefs::get_priority(can_id) != CanDefs::NORMAL_PRIORITY)
    {
        // Non-OpenLCB frame. Ignore.
        state_.input_frame_full = 0;
        return;
    }
    else if ((can_id & 0xfff) == state_.alias)
    {
        // Alias conflict.
        if (CanDefs::is_cid_frame(can_id))
        {
            // Someone is sending a CID for our alias. Reply with an RID.
            if (state_.output_frame_full)
            {
                return; // re-try.
            }
            setup_can_frame();
            CanDefs::control_init(
                state_.output_frame, state_.alias, CanDefs::RID_FRAME, 0);
        }
        else
        {
            // The hard way. Reset to init state to pick a new
            // alias.
            state_.init_state = (InitState)0;
        }
        state_.input_frame_full = 0;
        return;
    }
    else if (CanDefs::get_frame_type(can_id) == CanDefs::CONTROL_MSG)
    {
        if (CanDefs::get_control_field(can_id) == CanDefs::AME_FRAME &&
            (dlc == 0 || is_can_frame_nodeid()))
        {
            if (state_.output_frame_full)
            {
                return; // re-try.
            }
            setup_can_frame();
            CanDefs::control_init(
                state_.output_frame, state_.alias, CanDefs::AMD_FRAME, 0);
            set_can_frame_nodeid();
        }
        // All other control messages are ignored.
        state_.input_frame_full = 0;
        return;
    }
    else if ((can_id >> 12) == (0x1A000 | state_.alias) ||
        (can_id >> 12) == (0x1B000 | state_.alias))
    {
        // Datagram start frame.

        // Datagrams always need an answer. If we cannot render the answer,
        // let's not even try to parse the message.
        if (state_.output_frame_full)
        {
            return; // re-try.
        }
        if (dlc > 1 &&
            state_.input_frame.data[0] == DatagramDefs::CONFIGURATION)
        {
            return handle_memory_config_frame();
        }
        else
        {
            return reject_datagram();
        }
    }
#ifdef BOOTLOADER_DATAGRAM
    else if ((can_id >> 12) == (0x1C000 | state_.alias) ||
        (can_id >> 12) == (0x1D000 | state_.alias))
    {
        if (!state_.incoming_datagram_pending ||
            CanDefs::get_src(can_id) != state_.write_src_alias)
        {
            if (state_.output_frame_full)
            {
                return; // re-try.
            }
            reject_datagram();
            set_error_code(DatagramDefs::OUT_OF_ORDER);
            return;
        }
        memcpy(g_write_buffer + state_.write_buffer_index,
            state_.input_frame.data, state_.input_frame.can_dlc);
        state_.write_buffer_index += state_.input_frame.can_dlc;

        if (CanDefs::get_can_frame_type(can_id) ==
            CanDefs::DATAGRAM_FINAL_FRAME)
        {
            flush_flash_buffer();
            set_can_frame_addressed(Defs::MTI_DATAGRAM_OK);
            state_.incoming_datagram_pending = 0;
            state_.datagram_write_pending = 0;
        }

        state_.input_frame_full = 0;
        return;
    }
#endif
#ifdef BOOTLOADER_STREAM
    else if ((can_id >> 12) == (0x1F000 | state_.alias) && dlc > 1)
    {
        return handle_stream_data();
    }
#endif
    else if ((can_id >> 24) == 0x19)
    {
        // global or addressed message
        Defs::MTI mti = (Defs::MTI)CanDefs::get_mti(can_id);
        if (Defs::get_mti_address(mti) && state_.input_frame.can_dlc >= 2 &&
            (state_.input_frame.data[0] & 0xf) == ((state_.alias >> 8) & 0xf) &&
            state_.input_frame.data[1] == (state_.alias & 0xff))
        {
            return handle_addressed_message(mti);
        }
        else if (!Defs::get_mti_address(mti))
        {
            return handle_global_message(mti);
        }
    }
    // NMRAnet message
    state_.input_frame_full = 0;
    return;
}

/// Initialization state machine. Called repeatedly from the main loop so long
/// as initialization is not complete.
void handle_init()
{
    switch (state_.init_state)
    {
        case PICK_ALIAS:
        {
            auto node = nmranet_nodeid();
            if (!state_.alias)
            {
                // No alias yet. Pick default.
                state_.alias = nmranet_alias();
                // If no default, then generate from the node id.
                if (!state_.alias)
                {
                    state_.alias = (node & 0xfff) ^ ((node >> 12) & 0xfff) ^
                        ((node >> 24) & 0xfff) ^ ((node >> 36) & 0xfff);
                }
            }
            else
            {
                do
                {
                    state_.alias =
                        (state_.alias + ((uint16_t(node)) & 0xffe) + 1759) &
                        0xfff;
                } while (!state_.alias);
            }
            break;
        }
        case SEND_CID_7:
        {
            setup_can_frame();
            CanDefs::control_init(
                state_.output_frame, state_.alias, nmranet_nodeid() >> 36, 7);
            break;
        }
        case SEND_CID_6:
        {
            setup_can_frame();
            CanDefs::control_init(state_.output_frame, state_.alias,
                (nmranet_nodeid() >> 24) & 0xfff, 6);
            break;
        }
        case SEND_CID_5:
        {
            setup_can_frame();
            CanDefs::control_init(state_.output_frame, state_.alias,
                (nmranet_nodeid() >> 12) & 0xfff, 5);
            break;
        }
        case SEND_CID_4:
        {
            setup_can_frame();
            CanDefs::control_init(
                state_.output_frame, state_.alias, nmranet_nodeid() & 0xfff, 4);
            break;
        }
        case WAIT_RID:
        {
            break;
        }
        case SEND_RID:
        {
            setup_can_frame();
            CanDefs::control_init(
                state_.output_frame, state_.alias, CanDefs::RID_FRAME, 0);
            break;
        }
        case SEND_AMD:
        {
            setup_can_frame();
            CanDefs::control_init(
                state_.output_frame, state_.alias, CanDefs::AMD_FRAME, 0);
            set_can_frame_nodeid();
            break;
        }
        case SEND_NMRANET_INIT:
        {
            set_can_frame_global(Defs::MTI_INITIALIZATION_COMPLETE);
            set_can_frame_nodeid();
            break;
        }
        case INITIALIZED:
        {
            // shouldn't get here.
            return;
        }
    }
    state_.init_state = static_cast<InitState>(state_.init_state + 1);
}

/// Fills outgoing frame from the pending datagram send buffer.
void handle_send_datagram()
{
    setup_can_frame();
    uint32_t id;
    CanDefs::CanFrameType frame_type;
    if (!state_.datagram_offset)
    {
        if (state_.datagram_dlc <= 8)
        {
            frame_type = CanDefs::DATAGRAM_ONE_FRAME;
        }
        else
        {
            frame_type = CanDefs::DATAGRAM_FIRST_FRAME;
        }
    }
    else if (state_.datagram_dlc - state_.datagram_offset <= 8)
    {
        frame_type = CanDefs::DATAGRAM_FINAL_FRAME;
    }
    else
    {
        frame_type = CanDefs::DATAGRAM_MIDDLE_FRAME;
    }
    CanDefs::set_datagram_fields(
        &id, state_.alias, state_.datagram_dst, frame_type);
    SET_CAN_FRAME_ID_EFF(state_.output_frame, id);
    int len = std::min(state_.datagram_dlc - state_.datagram_offset, 8);
    memcpy(state_.output_frame.data,
        &state_.datagram_payload[state_.datagram_offset], len);
    state_.datagram_offset += len;
    state_.output_frame.can_dlc = len;
    if (state_.datagram_offset >= state_.datagram_dlc)
    {
        state_.datagram_reply_waiting = 1;
    }
}

/// Called once before starting the bootloader loop.
///
/// @return true if the application should be started, false if the bootloader.
bool bootloader_init() {
    {
        bool request = request_bootloader();
        bootloader_led(LED_REQUEST, request);
        bool csum_ok = check_application_checksum();
        bootloader_led(LED_CSUM_ERROR, !csum_ok);
        if (!request && csum_ok)
        {
            g_bootloader_busy = 0;
            application_entry();            
            return true;
        }
    }
    memset(&state_, 0, sizeof(state_));
    return false;
}

/// Called repeatedly in an infinite loop to run the bootloader.
///
/// @return true if the board has to be rebooted, false if the bootloader
/// should keep running (i.e. to call again).
bool bootloader_loop()
{
    {
#ifdef __linux__
        AtomicHolder h(&g_bootloader_lock);
#endif
        if (!state_.input_frame_full && read_can_frame(&state_.input_frame))
        {
            state_.input_frame_full = 1;
        }
        unsigned new_busy =
            (state_.input_frame_full || state_.output_frame_full ||
                state_.init_state != INITIALIZED ||
                (state_.datagram_output_pending
                    /*&& !state_.datagram_reply_waiting*/))
            ? 1
            : 0;
        if (g_bootloader_busy != new_busy)
        {
            bootloader_led(LED_ACTIVE, new_busy);
            g_bootloader_busy = new_busy;
        }
    }
    if (state_.output_frame_full && try_send_can_frame(state_.output_frame))
    {
        state_.output_frame_full = 0;
    }
    if (state_.request_reset && !g_bootloader_busy)
    {
        g_bootloader_busy = 0;
        bootloader_reboot();
        return true;
    }
    if (state_.input_frame_full)
    {
        handle_input_frame();
    }
    if (state_.init_state != INITIALIZED && !state_.output_frame_full)
    {
        handle_init();
    }
#ifdef BOOTLOADER_STREAM
    if (state_.stream_proceed_pending && !state_.output_frame_full)
    {
        set_can_frame_addressed(
            Defs::MTI_STREAM_PROCEED, state_.write_src_alias);
        state_.stream_proceed_pending = 0;
        state_.output_frame.data[state_.output_frame.can_dlc++] =
            state_.stream_src_id;
        state_.output_frame.data[state_.output_frame.can_dlc++] = STREAM_ID;
        state_.output_frame.data[state_.output_frame.can_dlc++] = 0;
        state_.output_frame.data[state_.output_frame.can_dlc++] = 0;
    }
#endif
    if (state_.datagram_output_pending && !state_.datagram_reply_waiting &&
        !state_.output_frame_full)
    {
        handle_send_datagram();
    }
    return false;
}

/// Main entry point for MCU-based bootloaders. Never returns.
void bootloader_entry()
{
    bootloader_hw_set_to_safe();
    bootloader_hw_init();

    if (bootloader_init()) return;

    while (true)
    {
        if (bootloader_loop()) return;
#ifdef __linux__
        usleep(10);
#endif
    } // while true
    try_send_can_frame(state_.output_frame);
}

} // extern "C"
