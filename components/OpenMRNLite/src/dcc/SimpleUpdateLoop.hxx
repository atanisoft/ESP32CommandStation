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

#ifndef _DCC_SIMPLEUPDATELOOP_HXX_
#define _DCC_SIMPLEUPDATELOOP_HXX_

#include <algorithm>

#include "dcc/UpdateLoop.hxx"
#include "executor/StateFlow.hxx"

namespace dcc
{

/// Implementation of a command station update loop. This loop iterates over
/// all locomotive implementations and polls them for the next packet in a
/// strict round-robin behavior (no prioritization).
///
/// Usage:
///
/// - Instantiate a state flow for sending outgoing dcc packets to the command
///  station driver, usually dcc::LocalTrackIf.
///
/// - create a FixedPool of dcc::Packets of a given size (usually 2 is enough).
///
/// - instantiate SimpleUpdateLoop, passing the LocalTrackIf pointer.
///
/// - send all packets from the pool to the updateloop using a PoolToQueueFlow.
class SimpleUpdateLoop : public StateFlow<Buffer<dcc::Packet>, QList<1>>,
                         private UpdateLoopBase
{
public:
    SimpleUpdateLoop(Service *service, PacketFlowInterface *track_send);
    ~SimpleUpdateLoop();

    /** Adds a new refresh source to the background refresh packets. */
    bool add_refresh_source(
        dcc::PacketSource *source, unsigned priority) OVERRIDE
    {
        /// @todo implement priority refresh.
        AtomicHolder h(this);
        refreshSources_.push_back(source);
        return true;
    }

    /** Deletes a packet refresh source. */
    void remove_refresh_source(dcc::PacketSource *source) OVERRIDE
    {
        AtomicHolder h(this);
        refreshSources_.erase(
            remove(refreshSources_.begin(), refreshSources_.end(), source),
            refreshSources_.end());
    }

    /** We ignore notifications. The loop will get to them anyway. */
    void notify_update(PacketSource *source, unsigned code) OVERRIDE
    {
    }

    // Entry to the state flow -- when a new packet needs to be sent.
    Action entry() OVERRIDE;

private:
    // Place where we forward the packets filled in.
    PacketFlowInterface *trackSend_;

    // Packet sources to ask about refreshing data periodically.
    vector<dcc::PacketSource *> refreshSources_;

    /// Offset in the refreshSources_ vector for the next loco to send.
    size_t nextRefreshIndex_;
    /// os time for the last time we sent a packet for loco zero.
    long long lastCycleStart_;
};
}

#endif // _DCC_SIMPLEUPDATELOOP_HXX_
