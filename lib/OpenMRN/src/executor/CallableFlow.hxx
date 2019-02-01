/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file CallableFlow.hxx
 *
 * Base class for State Flows that can be called as a subflow from a state
 * flow.
 *
 * @author Balazs Racz
 * @date 4 Feb 2017
 */

#ifndef _EXECUTOR_CALLABLEFLOW_HXX_
#define _EXECUTOR_CALLABLEFLOW_HXX_

#include "executor/StateFlow.hxx"

/// All callable flow request objects have to derive from this struct.
struct CallableFlowRequestBase {
    /// Call this from all instances of reset(...).
    void reset_base() {
        resultCode = 0;
    }

    /// If high bits are zero, this is a 16-bit OpenLCB result code. Higher
    /// values are OpenMRN errors.
    int resultCode;
    /// Used internally by the invoke_subflow mechanism of StateFlow to notify
    /// the calling flow upon completion.
    BarrierNotifiable done;
};


template<class RequestType> class CallableFlow : public StateFlow<Buffer<RequestType>, QList<1> > {
public:
    /// Creates a callable flow. @param s defines the service we are operating
    /// upon.
    CallableFlow(Service* s) :  StateFlow<Buffer<RequestType>, QList<1> >(s) {}

protected:
    using Action = StateFlowBase::Action;
    
    /// @return the current request we are working on.
    RequestType* request() {
        return this->message()->data();
    }

    /// Terminates the flow and returns the request buffer to the caller with
    /// an error code of OK (zero).
    Action return_ok()
    {
        return return_with_error(0);
    }

    /// Waits to be notified before moving onto the next state for termination.
    Action wait_and_return_ok()
    {
        return this->wait_and_call(STATE(wait_done));
    }

    /// Terminates the flow and returns the request buffer to the caller with
    /// an error code of OK (zero).
    Action wait_done()
    {
        return return_ok();
    }

    /// Terminates the flow and returns the request buffer to the caller with
    /// an specific error code.
    Action return_with_error(int error)
    {
        request()->resultCode = error;
        this->return_buffer();
        return this->exit();
    }
};

/** Helper function for testing flow invocations. */
template<class T, typename... Args>
BufferPtr<T> invoke_flow(FlowInterface<Buffer<T>>* flow, Args &&... args) {
    SyncNotifiable n;
    BufferPtr<T> b(flow->alloc());
    b->data()->reset(std::forward<Args>(args)...);
    b->data()->done.reset(&n);
    flow->send(b->ref());
    n.wait_for_notification();
    return b;
}

#endif // _EXECUTOR_CALLABLEFLOW_HXX_
