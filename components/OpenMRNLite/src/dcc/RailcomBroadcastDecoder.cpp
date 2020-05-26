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
 * \file RailcomBroadcastDecoder.cxx
 *
 * Simple state machine to decode DCC address from railcom broadcast packets.
 *
 * @author Balazs Racz
 * @date 11 Jan 2015
 */

#include "dcc/RailcomBroadcastDecoder.hxx"

#include "dcc/RailCom.hxx"

namespace dcc
{

/** Decodes a packet.
 *
 * @returns true if the packet is garbage or matches a global address
 * broadcast, false if it is a valid packet that is not an address
 * broadcast. */
bool RailcomBroadcastDecoder::process_packet(const dcc::Feedback &packet)
{
    if (packet.ch1Size)
    {
        return process_data(packet.ch1Data, packet.ch1Size);
    }
    else if (packet.ch2Size)
    {
        return process_data(packet.ch2Data, packet.ch2Size);
    }
    else
    {
        return true; // empty packet.
    }
}

bool RailcomBroadcastDecoder::process_data(const uint8_t *data, unsigned size)
{
    for (unsigned i = 0; i < size; ++i)
    {
        if (railcom_decode[data[i]] == RailcomDefs::INV)
            return true; // garbage.
    }
    /// TODO(balazs.racz) if we have only one byte in ch1 but we have a second
    /// byte in ch2, we should still process those because it might be a
    /// misaligned window.
    if (size < 2)
        return true; // Dunno what this is.a
    uint8_t type = (dcc::railcom_decode[data[0]] >> 2);
    if (size == 2)
    {
        uint8_t payload = dcc::railcom_decode[data[0]] & 0x3;
        payload <<= 6;
        payload |= dcc::railcom_decode[data[1]];
        switch (type)
        {
            case dcc::RMOB_ADRLOW:
                if (currentL_ == payload) {
                    if (countL_ < REPEAT_COUNT) ++countL_;
                } else {
                    currentL_ = payload;
                    countL_ = 0;
                }
                break;
            case dcc::RMOB_ADRHIGH:
                if (currentH_ == payload) {
                    if (countH_ < REPEAT_COUNT) ++countH_;
                } else {
                    currentH_ = payload;
                    countH_ = 0;
                }
                break;
            default:
                return false; // This is something we don't know about.
        }
        if (countL_ >= REPEAT_COUNT && countH_ >= REPEAT_COUNT) {
            currentAddress_ = (uint16_t(currentH_) << 8) | currentL_;
        }
        return true;
    }
    else
    {
        return false;
    }
}

void RailcomBroadcastDecoder::set_occupancy(bool value) {
    if (value) return;
    if (countH_) --countH_;
    if (countL_) --countL_;
    if ((!countH_) || (!countL_)) {
        currentAddress_ = 0;
    }
}

} // namespace dcc
