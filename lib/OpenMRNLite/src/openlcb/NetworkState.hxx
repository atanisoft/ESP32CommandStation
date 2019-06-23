/** @copyright
 * Copyright (c) 2019, Stuart W Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * @file Networkstate.hxx
 * OpenLCB implementation of network state.
 *
 * @author Stuart W Baker
 * @date 1 February 2019
 */

#ifndef _OPENLCB_OLCBNETWORKSTATE_HXX_
#define _OPENLCB_OLCBNETWORKSTATE_HXX_

#include "utils/NetworkState.hxx"

namespace openlcb
{

/// OpenLCB implementation of a binary network state.
class NetworkState : public ::NetworkState
{
public:
    /// Constructor.
    /// @param node the virtual node who exposes this bit.
    /// @param write_helper helper for generating events
    /// @param event_on event ID to set the state to true
    /// @param event_off event ID to set the state to false
    NetworkState(Node *node, WriteHelper *write_helper,
                     uint64_t event_on, uint64_t event_off)
        : writeHelper_(write_helper)
        , bit_(node, event_on, event_off, true)
        , bitPC_(&bit_)
    {
    }

private:
    /// Set the change notification callback.
    /// @param cb will be involked every time the state is changed
    ///        (both from local calls as well as from the network stack)
    void set_callback(std::function<void()> cb) override
    {
        bit_.set_change_callback(cb);
    }

    /// Toggle the state.
    /// @param done is the notification callback. If it is NULL, the writer
    ///        will be invoked inline and potentially block the calling thread.
    void toggle(Notifiable *done) override
    {
        bit_.toggle_state();
        bitPC_.SendEventReport(writeHelper_, done);
    }

    /// Set the state.
    /// @param state new state value
    /// @param done is the notification callback. If it is NULL, the writer
    ///        will be invoked inline and potentially block the calling thread.
    void set_state(bool state, Notifiable *done) override
    {
        bit_.set_state(state);
        bitPC_.SendEventReport(writeHelper_, done);
    }

    /// Get the current state.
    /// @return VALID, INVALID, or UNKNOWN
    State get_state() override
    {
        switch (bit_.get_current_state())
        {
            case EventState::VALID:
                return State::VALID;
            case EventState::INVALID:
                return State::INVALID;
            default:
                return State::UNKNOWN;
        }
    }

    /// Get an indication as to if the state is known.
    /// @return true if known, else false
    bool is_state_known() override
    {
        return bit_.is_network_state_known();
    }

    /// Queries and acquires the current state of the bit.
    /// @param done is the notification callback. If it is NULL, the writer
    ///        will be invoked inline and potentially block the calling thread.
    void initiate_query(BarrierNotifiable *done) override
    {
        bitPC_.SendQueryConsumer(writeHelper_, done);
    }

    /// Reset the network state to unknown.
    void reset() override
    {
        bit_.reset();
    }

    /// helper for generating events
    WriteHelper *writeHelper_;

    /// bit (dual event) container
    CallbackNetworkInitializedBit bit_;

    /// bit event producer/consumer
    BitEventPC bitPC_;
};

} // namespace openlcb

#endif // _OPENLCB_OLCBNETWORKSTATE_HXX_

