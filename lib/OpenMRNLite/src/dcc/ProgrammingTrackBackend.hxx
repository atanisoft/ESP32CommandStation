/** \copyright
 * Copyright (c) 2018, Balazs Racz
 * All rights reserved
 *
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
 * \file ProgrammingTrackBackend.hxx
 *
 * Helper flow for executing programming track commands
 *
 * @author Balazs Racz
 * @date 23 April 2018
 */

#ifndef _DCC_PROGRAMMINGTRACKBACKEND_HXX_
#define _DCC_PROGRAMMINGTRACKBACKEND_HXX_

#include <functional>

#include "dcc/Packet.hxx"
#include "dcc/PacketSource.hxx"
#include "dcc/UpdateLoop.hxx"
#include "executor/CallableFlow.hxx"
#include "openlcb/Datagram.hxx"
#include "utils/Singleton.hxx"

extern "C" {
void enable_dcc();
}

struct ProgrammingTrackRequest : public CallableFlowRequestBase
{
    enum EnterServiceMode
    {
        ENTER_SERVICE_MODE
    };

    enum ExitServiceMode
    {
        EXIT_SERVICE_MODE
    };

    enum SendReset
    {
        SEND_RESET
    };

    enum SendProgrammingPacket
    {
        SEND_PROGRAMMING_PACKET
    };

    /// Set up command to enter service mode.
    void reset(EnterServiceMode)
    {
        reset_base();
        cmd_ = Type::ENTER_SERVICE_MODE;
    }

    /// Set up command to exit service mode.
    void reset(ExitServiceMode)
    {
        reset_base();
        cmd_ = Type::EXIT_SERVICE_MODE;
    }

    /// Set up command to send some number of reset packets.
    /// @param count number of reset packets to send.
    void reset(SendReset, unsigned count)
    {
        reset_base();
        cmd_ = Type::SEND_RESET;
        repeatCount_ = count;
    }

    /// Set up command to send some number of service mode packets, waiting for
    /// an acknowledgement.
    /// @param pkt the DCC packet to send
    /// @param count number of times to send the DCC packet. This determines
    /// how long we are waiting for an acknowledgement.
    /// @param terminate_on_ack if true, will stop sending the packet when a
    /// service mode acknowledgement is detected instead of going all the way
    /// to count repetitions.
    void reset(SendProgrammingPacket, dcc::Packet pkt, unsigned count,
        bool terminate_on_ack = true)
    {
        reset_base();
        cmd_ = Type::SEND_SERVICE_PACKET;
        packetToSend_ = pkt;
        repeatCount_ = count;
        terminateOnAck_ = terminate_on_ack ? 1 : 0;
    }

    enum class Type
    {
        /// Switch from normal mode to service mode.
        ENTER_SERVICE_MODE,
        /// Switch from service mode back to normal operations mode
        EXIT_SERVICE_MODE,
        /// Send some number of reset commands
        SEND_RESET,
        /// Send a specific service mode DCC packet
        SEND_SERVICE_PACKET
    };

    /// What is the instruction to do.
    Type cmd_;

    /// This packet will be repeated on the programming track for
    /// SEND_SERVICE_PACKET
    dcc::Packet packetToSend_;

    /// How many times maximum to repeat the packet before timing out on no
    /// acknowledgement, or how many reset packets to send.
    unsigned repeatCount_ : 16;

    /// Should we short-cut sending the packet repetitions when we've seen an
    /// ack. If 1, returns immediately upon seeing the ack.
    unsigned terminateOnAck_ : 1;

    /// Output arguments. These are filled by the flow upon return.
    unsigned hasAck_ : 1;

    /// Set to 1 if the programming track has reached the current limit.
    /// @todo(balazs.racz) we need to figure out how to recover from a program
    /// track short.
    unsigned hasShortCircuit_ : 1;

private:
    /// Resets all internal variables to default state. Call at the beginning
    /// of all reset(...) functions.
    void reset_base()
    {
        CallableFlowRequestBase::reset_base();
        hasAck_ = 0;
        hasShortCircuit_ = 0;
        terminateOnAck_ = 0;
        repeatCount_ = 0;
    }
};

class ProgrammingTrackBackend : public CallableFlow<ProgrammingTrackRequest>,
                                private dcc::NonTrainPacketSource,
                                public Singleton<ProgrammingTrackBackend>
{
public:
    ProgrammingTrackBackend(Service *service,
        std::function<void()> enable_program_track_mode,
        std::function<void()> disable_program_track_mode)
        : CallableFlow<ProgrammingTrackRequest>(service)
        , enableProgramTrackMode_(std::move(enable_program_track_mode))
        , disableProgramTrackMode_(std::move(disable_program_track_mode))
        , isWaitingForPackets_(0)
    {
    }

    enum ResultCodes
    {
        /// cleared when done is called.
        OPERATION_PENDING = openlcb::DatagramClient::OPERATION_PENDING,
    };

    Action entry() override
    {
        request()->resultCode = OPERATION_PENDING;
        switch (request()->cmd_)
        {
            case ProgrammingTrackRequest::Type::ENTER_SERVICE_MODE:
                return call_immediately(STATE(enter_service_mode));

            case ProgrammingTrackRequest::Type::EXIT_SERVICE_MODE:
                return call_immediately(STATE(exit_service_mode));

            case ProgrammingTrackRequest::Type::SEND_RESET:
                return call_immediately(STATE(send_reset));

            case ProgrammingTrackRequest::Type::SEND_SERVICE_PACKET:
                return call_immediately(STATE(send_service_packet));
        }
        DIE("Unknown programming track request command");
    }

    /// Call this function when the service mode acknowledgement is detected by
    /// the short detector.
    void notify_service_mode_ack()
    {
        if (!request())
        {
            return;
        }
        request()->hasAck_ = 1;
        if (request()->terminateOnAck_ && isWaitingForPackets_)
        {
            // Early exit criteria reached. Wake up the flow and return.
            isWaitingForPackets_ = 0;
            /// @todo: wait for flushing the packets to the track.
            // resume flow
            notify();
        }
    }

    /// Call this function when the service mode current limit is exceeded.
    void notify_service_mode_short()
    {
        if (!request())
        {
            return;
        }
        request()->hasShortCircuit_ = 1;
        /// @todo do we need to do something else here?
    }

private:
    Action enter_service_mode()
    {
        // 1. switch short circuit detector to service mode
        //
        // 2. register exclusive packet source with the update loop.
        //
        // 3. switch the power outputs to the programming track. That's
        // dependant on the hardware revision.
        enableProgramTrackMode_();
        /// @todo: do we need to flush the packet queue here?
        if (!packet_processor_add_refresh_source(
                this, dcc::UpdateLoopBase::PROGRAMMING_PRIORITY))
        {
            // There was another high priority source, probably we are in ESTOP.
            packet_processor_remove_refresh_source(this);
            return return_with_error(openlcb::Defs::ERROR_OUT_OF_ORDER);
        }
        // enable_dcc();
        return return_ok();
    }

    Action exit_service_mode()
    {
        packet_processor_remove_refresh_source(this);
        /// @todo: maybe we need to flush the packet queue here as well,
        /// otherwise we might be sending out service mode reset packets to the
        /// mainline.
        disableProgramTrackMode_();
        return return_ok();
    }

    Action send_reset()
    {
        // record that we want to send reset packets.
        request()->packetToSend_.set_dcc_reset_all_decoders();
        request()->packetToSend_.packet_header.send_long_preamble = 1;
        return call_immediately(STATE(send_service_packet));
    }

    Action send_service_packet()
    {
        isWaitingForPackets_ = 1;
        // This pauses the flow until somebody calls notify(), then continues
        // in the given state.
        request()->packetToSend_.packet_header.send_long_preamble = 1;
        return wait_and_call(STATE(packets_sent));
    }

    Action packets_sent()
    {
        // this will terminate the current flow with a 0 error value.
        /// @todo actually send back an error value if we've seen a short maybe?
        return return_ok();
    }

    /// Function that is called by the track driver when we need to generate a
    /// DCC packet to the track.
    /// @param code ignored for programming track
    /// @param packet buffer to fill in with next packet to send.
    void get_next_packet(unsigned code, dcc::Packet *packet) override
    {
        if (request() == nullptr)
        {
            packet->set_dcc_reset_all_decoders();
            return;
        }

        *packet = request()->packetToSend_;
        if (request()->repeatCount_ > 0)
        {
            --request()->repeatCount_;
        }
        else
        {
            if (isWaitingForPackets_)
            {
                isWaitingForPackets_ = 0;
                /// @todo: wait for flushing the packets to the track.
                // resume flow
                notify();
            }
        }
    }

    /// Callback to connect to the program track hardware control.
    std::function<void()> enableProgramTrackMode_;
    /// Callback to connect to the program track hardware control.
    std::function<void()> disableProgramTrackMode_;

    /// 1 if the flow is blocked waiting for sending out the respective number
    /// of packets.
    unsigned isWaitingForPackets_ : 1;
};

#endif // _DCC_PROGRAMMINGTRACKBACKEND_HXX_
