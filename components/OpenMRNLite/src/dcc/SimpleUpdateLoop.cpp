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
 * \file SimpleUpdateLoop.cxx
 *
 * Control flow central to the command station: it round-robins between
 * refreshing the individual trains.
 *
 * @author Balazs Racz
 * @date 1 Feb 2015
 */

#include "dcc/SimpleUpdateLoop.hxx"
#include "dcc/Packet.hxx"
#include "dcc/PacketSource.hxx"

namespace dcc
{

SimpleUpdateLoop::SimpleUpdateLoop(Service *service,
                                   PacketFlowInterface *track_send)
    : StateFlow(service)
    , trackSend_(track_send)
    , nextRefreshIndex_(0)
    , lastCycleStart_(os_get_time_monotonic())
{
}

SimpleUpdateLoop::~SimpleUpdateLoop()
{
}

StateFlowBase::Action SimpleUpdateLoop::entry()
{
    long long current_time = os_get_time_monotonic();
    long long prev_cycle_start = lastCycleStart_;
    if (nextRefreshIndex_ >= refreshSources_.size())
    {
        nextRefreshIndex_ = 0;
        lastCycleStart_ = current_time;
    }
    if (nextRefreshIndex_ == 0 &&
        (current_time - prev_cycle_start < MSEC_TO_NSEC(5) ||
         refreshSources_.empty()))
    {
        // We do not want to send another packet to the same locomotive too
        // quick. We send an idle packet instead. OR: We do not have any
        // locomotives at all. We will keep sending idle packets.
        message()->data()->set_dcc_idle();
    }
    else
    {
        // Send an update to the current loco.
        refreshSources_[nextRefreshIndex_]
            ->get_next_packet(0, message()->data());
        nextRefreshIndex_++;
    }
    // We pass on the filled packet to the track processor.
    trackSend_->send(transfer_message());
    return exit();
}

} // namespace dcc
