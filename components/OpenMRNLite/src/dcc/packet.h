/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file packet.h
 *
 * Defines a DCC Packet structure.
 *
 * @author Balazs Racz
 * @date 5 February 2016
 */

#ifndef _DCC_PACKET_H_
#define _DCC_PACKET_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of payload bytes. */
#define DCC_PACKET_MAX_PAYLOAD (6)
/** Send this speed step to emergency-stop the locomotive. */
#define DCC_PACKET_EMERGENCY_STOP (0xFFFF)
/** Send this speed step to switch direction of the locomotive. Only used
* for marklin-14-step speed commands. */
#define DCC_PACKET_CHANGE_DIR (0xFFFF)

/// Stores a DCC packet in memory. Used to send data from the packet generation
/// (usually the command station refresh loop flows) to the DCC track driver.
typedef struct dcc_packet
{
    /// Specifies the meaning of the command byte for packets to send.
    struct pkt_t
    {
        /// Always 0.
        uint8_t is_pkt : 1;
        /// 0: DCC packet, 1: motorola packet.
        uint8_t is_marklin : 1;

        /// typically for DCC packets:
        /// 1: do NOT append an EC byte to the end of the packet.
        uint8_t skip_ec : 1;
        /// 1: send long preamble instead of packet. 0: send normal preamble
        /// and pkt.
        uint8_t send_long_preamble : 1;
        /// 1: wait for service mode ack and report it back to the host.
        uint8_t sense_ack : 1;
        /// The packet will be sent 1 + rept_count times to the wire. default:
        /// 0.
        uint8_t rept_count : 2;
        /// reserved for future use.
        uint8_t reserved : 1;
    };

    /// Specifies the meaning of the command byte for meta-commands to send.
    struct cmd_t
    {
        /// Always 1.
        uint8_t is_pkt : 1;
        /// Command identifier.
        uint8_t cmd : 7;
    };

    union
    {
        uint8_t header_raw_data;
        struct pkt_t packet_header;
        struct cmd_t command_header;
    };
    /** Specifies the number of used payload bytes. */
    uint8_t dlc;
    /** Packet payload bytes. */
    uint8_t payload[DCC_PACKET_MAX_PAYLOAD];

    /** An opaque key used by the hardware driver to attribute feedback
     * information to the source of the packet. This key will be sent back in
     * the dcc::Feedback structure. If the key is non-zero it is guaranteed
     * that some feedback (maybe empty) will be sent back after the packet is
     * transmitted to the track. */
    uintptr_t feedback_key;
} DCCPacket;

#ifdef __cplusplus
}
#endif

#endif /* _DCC_PACKET_H_ */

