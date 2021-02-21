/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file BusMaster.hxx
 *
 * An abstract controller to manage a shared bus which needs to be polled.
 *
 * @author Balazs Racz
 * @date 30 Dec 2020
 */

#ifndef _UTILS_BUSMASTER_HXX_
#define _UTILS_BUSMASTER_HXX_

#include "executor/StateFlow.hxx"
#include "utils/Buffer.hxx"
#include "utils/LimitedPool.hxx"
#include "utils/ScheduledQueue.hxx"

/// This is a namespace class that contains a variety of related classes for
/// handling a bus that needs a polling loop.
template <class MessageType> class Bus
{
public:
    /// The buffer type that is handed around to the different participants.
    using Packet = Buffer<MessageType>;

    /// Self-owned buffer type.
    using PacketPtr = BufferPtr<MessageType>;

    /// This is the consumer of the generated packets. Usually a device driver
    /// or something like that.
    using PacketSink = FlowInterface<Packet>;

    /// Abstract class that gets scheduled for polling and when the master
    /// decides to take it, gets a buffer to fill in with a packet.
    class Activity : public QMember
    {
    public:
        using Packet = Bus::Packet;
        using PacketPtr = Bus::PacketPtr;

        /// Complete a packet to send to the bus.
        /// @param packet must be filled in the by callee. Ownership is
        /// retained by the caller.
        virtual void fill_packet(Packet *packet) = 0;
    };

    /// The Bus Master class. This keeps the scheduler of activities and owns a
    /// variety of auxiliary objects and memory management.
    class Master : public StateFlowBase
    {
    public:
        /// Constructor.
        /// @param s the executor to run the service on.
        /// @param sink this is where the generated packets will be sent out.
        /// @param idle when no activity is scheduled, this activity will be
        /// invoked with the packet. This activity must be ready to render a
        /// packet at all times.
        /// @param num_enqueued this is how many packets we fill in and enqueue
        /// for the sink. It is a tradeoff between the sink being out of work
        /// vs the what is the lowest latency between enqueueing a high
        /// priority activity and that getting the next packet. Recommended
        /// values are 2 or 3.
        Master(
            Service *s, PacketSink *sink, Activity *idle, unsigned num_enqueued)
            : StateFlowBase(s)
            , idle_(idle)
            , sink_(sink)
            , pool_(sizeof(Packet), num_enqueued)
            , numPacketsInPool_(num_enqueued)
            , needShutdown_(false)
        {
        }

        /// Used in unittests to cleanly shutdown the bus master.
        void shutdown()
        {
            needShutdown_ = true;
            while (!is_terminated() && (pool_.free_items() < numPacketsInPool_))
            {
                usleep(200);
            }
        }

        /// Sets the scheduling policy. This must be called exactly once after
        /// construction before scheduling any bus activity.
        void set_policy(unsigned num_prio, const Fixed16 *strides)
        {
            queue_.emplace(num_prio, strides);
            start_flow(STATE(get_buffer));
        }

        /// Adds an activity to the bus master's scheduler. The activity must
        /// be ready to fill in a packet right now.
        /// @param a the activity to schedule.  The caller retains ownership.
        /// @param prio the priority band to schedule in.
        void schedule_activity(Activity *a, unsigned prio)
        {
            queue_->insert(a, prio);
        }

    private:
        /// Start of scheduling flow.
        Action get_buffer()
        {
            return allocate_and_call(sink_, STATE(fill_pkt), &pool_);
        }

        /// Executes the scheduling decision, fills in the packet by the
        /// selected activity and sends it to the bus sink.
        Action fill_pkt()
        {
            auto buf = get_buffer_deleter(get_allocation_result(sink_));
            if (needShutdown_)
            {
                return exit();
            }
            // Picks the next activity to do on the bus.
            Activity *a = (Activity *)queue_->next().item;
            if (!a)
            {
                a = idle_;
            }
            a->fill_packet(buf.get());
            sink_->send(buf.release());
            return call_immediately(STATE(get_buffer));
        }

        /// Handles the policy of polling.
        uninitialized<ScheduledQueue> queue_;
        /// If we have no activity to do, this activity gets invoked. It must
        /// always be able to fill in a packet.
        Activity *idle_;
        /// Where to send the generated packets.
        PacketSink *sink_;
        /// Source of the buffers that we fill and hand out.
        LimitedPool pool_;
        /// Total number of packets that the pool can generate.
        uint16_t numPacketsInPool_;
        /// True if shutdown was requested.
        uint16_t needShutdown_ : 1;
    }; // class Master

private:
    /// This class cannot be instantiated.
    Bus();
};

#endif // _UTILS_BUSMASTER_HXX_