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
 * @file NetworkState.hxx
 * Abstract interface of network state.
 *
 * @author Stuart W Baker
 * @date 1 February 2019
 */

#ifndef _UTILS_NETWORKSTATE_HXX_
#define _UTILS_NETWORKSTATE_HXX_

#include <functional>

#include "executor/Notifiable.hxx"

/// "Abstract" interface for a binary network state.
class NetworkState
{
public:
    enum State
    {
        VALID    = 0, ///< state is "valid"
        INVALID  = 1, ///< state is "invalid"
        UNKNOWN  = 3, ///< state is "unknown"
    };

    /// Destructor.
    virtual ~NetworkState()
    {
    }

    /// Set the change notification callback.
    /// @param cb will be involked every time the state is changed
    ///        (both from local calls as well as from the network stack)
    virtual void set_callback(std::function<void()> cb) = 0;

    /// Toggle the state.
    /// @param done is the notification callback. If it is NULL, the writer
    ///        will be invoked inline and potentially block the calling thread.
    virtual void toggle(Notifiable *done) = 0;

    /// Set the state.
    /// @param state new state value
    /// @param done is the notification callback. If it is NULL, the writer
    ///        will be invoked inline and potentially block the calling thread.
    virtual void set_state(bool state, Notifiable *done) = 0;

    /// Get the current state.
    /// @return VALID, INVALID, or UNKNOWN
    virtual State get_state() = 0;

    /// Get an indication as to if the state is known.
    /// @return true if known, else false
    virtual bool is_state_known() = 0;

    /// Queries and acquires the current state of the bit.
    /// @param done is the notification callback. If it is NULL, the writer
    ///        will be invoked inline and potentially block the calling thread.
    virtual void initiate_query(BarrierNotifiable *done) = 0;

    /// Reset the network state to unknown.
    virtual void reset() = 0;
};

#endif // _UTILS_NETWORKSTATE_HXX_
