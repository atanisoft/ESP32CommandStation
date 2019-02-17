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
 * \file RailCom.hxx
 *
 * Helper functions and tables for RailCom implementations.
 *
 * @author Balazs Racz
 * @date 12 Nov 2014
 */

#ifndef _DCC_RAILCOM_HXX_
#define _DCC_RAILCOM_HXX_

#include <cstdint>
#include <string>
#include <vector>

#include "dcc/railcom.h"

namespace dcc
{

/// Structure used for reading (railcom) feedback data from DCC / Railcom
///  device drivers.
struct Feedback : public DCCFeedback
{
    /// Clears the structure and sets the feedback key to a specific value.
    void reset(uint32_t feedback_key)
    {
        this->feedbackKey = feedback_key;
        ch1Size = 0;
        ch2Size = 0;
        channel = 0;
    }

    /// Appends a byte to the channel 1 payload.
    void add_ch1_data(uint8_t data)
    {
        if (ch1Size < sizeof(ch1Data))
        {
            ch1Data[ch1Size++] = data;
        }
    }

    /// Appends a byte to the channel 2 payload.
    void add_ch2_data(uint8_t data)
    {
        if (ch2Size < sizeof(ch2Data))
        {
            ch2Data[ch2Size++] = data;
        }
    }
};

/// Formats a dcc::Feedback message into a debug string.
std::string railcom_debug(const Feedback& fb);

/// Special constant values returned by the @ref railcom_decode[] array.
namespace RailcomDefs
{
    /// invalid value (not conforming to the 4bit weighting requirement)
    static const uint8_t INV = 0xff;
    /// Railcom ACK; the decoder received the message ok. NOTE: some early
    /// software versions may have ACK and NACK exchanged.
    static const uint8_t ACK = 0xfe;
    /// The decoder rejected the packet.
    static const uint8_t NACK = 0xfd;
    /// The decoder is busy; send the packet again. This is typically returned
    /// when a POM CV write is still pending; the caller must re-try sending the
    /// packet later.
    static const uint8_t BUSY = 0xfc;
    /// Reserved for future expansion.
    static const uint8_t RESVD1 = 0xfb;
    /// Reserved for future expansion.
    static const uint8_t RESVD2 = 0xfa;
    /// Reserved for future expansion.
    static const uint8_t RESVD3 = 0xf8;
}

/** Table for 8-to-6 decoding of railcom data. This table can be indexed by the
 * 8-bit value read from the railcom channel, and the return value will be
 * either a 6-bit number, or one of the constants in @ref RailcomDefs. If the
 * value is invalid, the INV constant is returned. */
extern const uint8_t railcom_decode[256];

/// Packet identifiers from Mobile Decoders.
enum RailcomMobilePacketId
{
    RMOB_POM = 0,
    RMOB_ADRHIGH = 1,
    RMOB_ADRLOW = 2,
    RMOB_EXT = 3,
    RMOB_DYN = 7,
    RMOB_SUBID = 12,
};

/// Represents a single Railcom datagram. There can be multiple railcom
/// datagrams in the cutout of one railcom packet: usually zero or one in
/// channel 1 and up to four datagrams in channel 2.
struct RailcomPacket
{
    enum
    {
        GARBAGE,
        ACK,
        NACK,
        BUSY,
        MOB_POM,
        MOB_ADRHIGH,
        MOB_ADRLOW,
        MOB_EXT,
        MOB_DYN,
        MOB_SUBID
    };
    /// which detector supplied this data
    uint8_t hw_channel;
    /// which part of the railcom cutout (1 or 2)
    uint8_t railcom_channel;
    /// packet type, see enum above
    uint8_t type;
    /// payload of the railcom packet, justified to LSB.
    uint32_t argument;
    /// Constructor.
    ///
    /// @param _hw_channel which detector supplied this data
    /// @param _railcom_channel which cutout (ch1 or ch2) this is coming from
    /// @param _type see enum
    /// @param _argument payload of the railcom packet, justified to LSB.
    RailcomPacket(uint8_t _hw_channel, uint8_t _railcom_channel, uint8_t _type,
        uint32_t _argument)
        : hw_channel(_hw_channel)
        , railcom_channel(_railcom_channel)
        , type(_type)
        , argument(_argument)
    {
    }
};

/** Interprets the data from a railcom feedback. If the railcom data contains
 * error, will add a packet of type "GARBAGE" into the output list. Clears the
 * output list before fillign with the railcom data. */
void parse_railcom_data(
    const dcc::Feedback &fb, std::vector<struct RailcomPacket> *output);

}  // namespace dcc

#endif // _DCC_RAILCOM_HXX_
