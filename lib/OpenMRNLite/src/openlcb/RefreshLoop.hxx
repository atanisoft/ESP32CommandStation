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
 * \file RefreshLoop.hxx
 * Flow that performs periodic polling.
 *
 * @author Balazs Racz
 * @date 13 Jul 2014
 */

#ifndef _OPENLCB_REFRESHLOOP_HXX_
#define _OPENLCB_REFRESHLOOP_HXX_

#include <vector>

#include "executor/StateFlow.hxx"
#include "executor/Timer.hxx"
#include "openlcb/WriteHelper.hxx"

namespace openlcb
{

/// Abstract base class for components that need repeated execution (with a
/// specified frequency, i.e. not a busy loop).
class Polling
{
public:
    /** This function will be called approximately 33 times per second by the
     * refresh loop. It must notify done when it is finished using the
     * writehelper. */
    virtual void poll_33hz(WriteHelper *helper, Notifiable *done) = 0;
};

/// State flow that calls a set of @ref Polling objects at a regular interval.
///
/// Usage: Instantiate your objects from descendants of the Polling
/// class. Create a RefreshLoop object and pass in the list of Polling object
/// pointers to the constructor.
class RefreshLoop : public StateFlowBase
{
public:
    RefreshLoop(Node *node, const std::initializer_list<Polling *> &members)
        : StateFlowBase(node->iface())
        , timer_(this)
        , lastTimeout_(os_get_time_monotonic())
        , members_(members)
    {
        start_flow(STATE(wait_for_tick));
    }

    /** Stops the refresh loop. If you call this funciton, then wait for the
     * executor, then it is safe to delete *this. */
    void stop()
    {
        set_terminated();
        timer_.ensure_triggered();
    }

    Action wait_for_tick()
    {
        lastTimeout_ += MSEC_TO_NSEC(30);
        nextMember_ = members_.begin();
        // If we have overflowed our timer, this call will happen immediately.
        return sleep_and_call(&timer_, lastTimeout_ - os_get_time_monotonic(),
                              STATE(call_members));
    }

    Action call_members()
    {
        while (true)
        {
            if (nextMember_ == members_.end())
            {
                return call_immediately(STATE(wait_for_tick));
            }
            bn_.reset(this);
            (*nextMember_)->poll_33hz(&helper_, bn_.new_child());
            ++nextMember_;
            if (bn_.abort_if_almost_done())
            {
                // The polling member called done notifiable inline. Short
                // circuits the yield to the executor.
                continue;
            }
            bn_.maybe_done();
            return wait();
        }
    }

private:
    /// Message write buffer that is passed to each polling object.
    WriteHelper helper_;
    /// Helper object for sleeps.
    StateFlowTimer timer_;
    /// Rolling clock of when the next wakeup should happen.
    long long lastTimeout_;
    /// Controllable notifier to be passed into the polling objects.
    BarrierNotifiable bn_;
    /// Data structure type for storing the polling members.
    typedef vector<Polling*> members_type;
    /// The actual members.
    members_type members_;
    /// Iterator for going through the members list.
    members_type::iterator nextMember_;
};

} // namespace openlcb

#endif // _OPENLCB_REFRESHLOOP_HXX_
