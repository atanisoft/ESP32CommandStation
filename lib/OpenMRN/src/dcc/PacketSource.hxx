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
 * \file PacketSource.hxx
 *
 * Defines an abstract TrainImpl class that is able to provide packets for
 * refresh.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#ifndef _DCC_PACKETSOURCE_HXX_
#define _DCC_PACKETSOURCE_HXX_

#include "openlcb/TrainInterface.hxx"

namespace dcc {

struct Packet;
/// C++ type encompassing a speed value for OpenLCB.
typedef openlcb::SpeedType SpeedType;

/// Abstract class for streams of DCC packets. there will be typically one
/// object of a child type for every decoder on the track voltage that needs an
/// infinite set of repeating packets. This interface will be polled in a
/// round-robin manner by the @ref UpdateLoopBase.
class PacketSource : public openlcb::TrainImpl {
public:
    /** Generates the next packet to send out to the track. 
     * @param code if 0, then the next background refresh packet shold be
     * generated. If non-zero, then it specifies a train-specific value that
     * tells which recently changed value should be generated. 
     * @param packet is the storage to set the outgoing packet in. */
    virtual void get_next_packet(unsigned code, Packet* packet) = 0;
};

/// Abstract class that is a packet source but not a TrainImpl. Provides dummy
/// implementations for the TrainImpl interface virtual functions.
class NonTrainPacketSource : public PacketSource
{
private:
    void set_speed(SpeedType speed) override
    {
    }
    SpeedType get_speed() override
    {
        return SpeedType();
    }
    void set_emergencystop() override
    {
    }
    bool get_emergencystop() override
    {
        return false;
    }
    void set_fn(uint32_t address, uint16_t value) override
    {
    }
    uint16_t get_fn(uint32_t address) override
    {
        return 0;
    }
    uint32_t legacy_address() override
    {
        return 0;
    }
    dcc::TrainAddressType legacy_address_type() override
    {
        return dcc::TrainAddressType::DCC_SHORT_ADDRESS;
    }
};

}  // namespace dcc


#endif // _DCC_PACKETSOURCE_HXX_
