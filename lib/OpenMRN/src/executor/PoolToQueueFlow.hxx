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
 * \file PoolToQueueFlow.hxx
 *
 * Helper control flow for fixed-size buffering.
 *
 * @author Balazs Racz
 * @date 13 May 2014
 */

#ifndef _EXECUTOR_POOLTOQUEUEFLOW_HXX_
#define _EXECUTOR_POOLTOQUEUEFLOW_HXX_

/** This flow can be used to take all entries that show up in a FixedPool and
 * send them (empty) to a stateflow for processing.
 *
 * This basically allows a regular stateflow for reacting to when the message
 * it has sent out has finished processing by the destination.
 *
 * Usage: create a FixedPool of a given size (that you want to buffer, usually
 * 2 or 3) of the particular type T. Implement your request generator as a
 * StateFlow<Buffer<T> >. Make the request generator's entry() state do the
 * following: clear the incoming buffer, fill in the new request, then send off
 * to the request processor.
 *
 * Then create a PoolToQueueFlow<Buffer<T> > on the pool and give it the
 * request generator's pointer.
 *
 * Whenever the request processor is finished processing a request, the buffer
 * will be returned to the pool, from there it will be automatically picked up
 * by the PoolToQueueFlow, sent to the request generator, which will generate
 * the next request and send it to the request processor. This will run in an
 * infinite loop with the fixed buffer size.
 */
template <class T> class PoolToQueueFlow : public StateFlowBase
{
public:
    /// Constructor.
    ///
    /// @param service defines which executor this flow will run in
    /// @param source the pool to allocate all entries from
    /// @param dest the queue to send all allocated entries to.
    PoolToQueueFlow(Service *service, FixedPool *source, FlowInterface<T> *dest)
        : StateFlowBase(service)
        , source_(source)
        , dest_(dest)
    {
        get_next_entry();
    }

private:
    /// Allocates the next entry from the pool. @return next state.
    Action get_next_entry()
    {
        return allocate_and_call(dest_, STATE(got_entry), source_);
    }

    /// Send the entry we got to the queue. @return next state.
    Action got_entry()
    {
        auto *b = get_allocation_result(dest_);
        dest_->send(b);
        return get_next_entry();
    }

    /// The pool to read all entries from.
    FixedPool *source_;
    /// The queue to send all entries to.
    FlowInterface<T> *dest_;
};

#endif // _EXECUTOR_POOLTOQUEUEFLOW_HXX_
