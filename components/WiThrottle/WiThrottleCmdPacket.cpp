/** \copyright
 * Copyright (c) 2022, Mike Dunston
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
 * \file WiThrottleCommands.cpp
 *
 * WiThrottle commands implementation.
 *
 * @author Mike Dunston
 * @date 8 Feb 2022
 */

/// @file WiThrottle commands implementation.

#include "WiThrottle.hxx"

#include <dcc/DccDebug.hxx>
#include <dcc/Packet.hxx>
#include <HttpStringUtils.h>

namespace withrottle
{
    WiThrottleClientFlow::WiThrottleCommandRawPacket::WiThrottleCommandRawPacket(
        WiThrottleClientFlow *throttle, dcc::TrackIf *track)
        : WiThrottleCommandBase(throttle, WiThrottleCommands::HEX_PACKET),
        track_(track)
    {
    }

    WiThrottleClientFlow::WiThrottleCommandRawPacket::~WiThrottleCommandRawPacket()
    {
    }

    StateFlowBase::Action WiThrottleClientFlow::WiThrottleCommandRawPacket::entry()
    {
        dcc::TrackIf::message_type *pkt;
        mainBufferPool->alloc(&pkt);
        auto *packet = pkt->data();

        packet->packet_header.rept_count =
            std::stoi(message()->data()->payload.substr(0, 1));
        // hex pairs should include the checksum byte
        packet->packet_header.skip_ec = 1;
        std::vector<std::string> bytes;
        http::tokenize(message()->data()->payload.substr(1), bytes);
        for (auto b : bytes)
        {
            packet->payload[packet->dlc++] = std::stoi(b, nullptr, 16);
        }
  
        track_->send(pkt);

        LOG(INFO, "[WiThrottle:DCCPacket] %s",
            dcc::packet_to_string(*(packet), true).c_str());
        return release_and_exit();
    }
} // namespace withrottle