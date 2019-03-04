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
 * \file FakeTrackIf.hxx
 *
 * Allows running a full commandstation update loop in a virtual (e.g. linux)
 * setup for testing.
 *
 * @author Balazs Racz
 * @date 25 Mar 2016
 */

#ifndef _DCC_FAKETRACKIF_HXX_
#define _DCC_FAKETRACKIF_HXX_

#include "executor/Executor.hxx"
#include "executor/StateFlow.hxx"
#include "dcc/Packet.hxx"

namespace dcc
{

/// StateFlow that accepts dcc::Packet structures and drops them to the floor.
class FakeTrackIf : public StateFlow<Buffer<dcc::Packet>, QList<1>>
{
public:
    /// Constructor.
    ///
    /// @param service defines which executor *this should be running on.
    /// @param pool_size how many packets we should generate ahead of time.
    FakeTrackIf(Service *service, int pool_size)
        : StateFlow<Buffer<dcc::Packet>, QList<1>>(service)
        , pool_(sizeof(Buffer<dcc::Packet>), pool_size)
    {
    }

    FixedPool *pool() OVERRIDE
    {
        return &pool_;
    }

protected:
    Action entry() OVERRIDE
    {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(10), STATE(finish));
    }

    /// Do nothing. @return next action.
    Action finish()
    {
        return release_and_exit();
    }

    /// Pool of unallocated packets.
    FixedPool pool_;
    /// Helper object for timing.
    StateFlowTimer timer_{this};
};

} // namespace dcc

#endif // _DCC_FAKETRACKIF_HXX_
