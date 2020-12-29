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
 * \file Loco.cxx
 *
 * Defines a simple DCC locomotive.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#include "dcc/Loco.hxx"

#include "utils/logging.h"
#include "dcc/UpdateLoop.hxx"

namespace dcc
{

/// Forces compilation of all existing train implementations even though many
/// are actually templates. This avoid needing to put all this code into a .hxx
/// file.
extern void createtrains();

template <> DccTrain<Dcc28Payload>::~DccTrain()
{
    packet_processor_remove_refresh_source(this);
}

template <> DccTrain<Dcc128Payload>::~DccTrain()
{
    packet_processor_remove_refresh_source(this);
}

unsigned Dcc28Payload::get_fn_update_code(unsigned address)
{
    if (address < 5)
    {
        return FUNCTION0;
    }
    else if (address < 9)
    {
        return FUNCTION5;
    }
    else if (address < 13)
    {
        return FUNCTION9;
    }
    else if (address < 21)
    {
        return FUNCTION13;
    }
    else if (address <= 28)
    {
        return FUNCTION21;
    }
    return SPEED;
}

// Generates next outgoing packet.
template <class Payload>
void DccTrain<Payload>::get_next_packet(unsigned code, Packet *packet)
{
    packet->start_dcc_packet();
    if (this->p.isShortAddress_)
    {
        packet->add_dcc_address(DccShortAddress(this->p.address_));
    }
    else
    {
        packet->add_dcc_address(DccLongAddress(this->p.address_));
    }
    if (code == REFRESH)
    {
        code = MIN_REFRESH + this->p.nextRefresh_++;
        if (this->p.nextRefresh_ > MAX_REFRESH - MIN_REFRESH)
        {
            this->p.nextRefresh_ = 0;
        }
    }
    else
    {
        // User action. Up repeat count.
        packet->packet_header.rept_count = 2;
    }
    switch (code)
    {
        case FUNCTION0:
        {
            packet->add_dcc_function0_4(
                (this->p.fn_ & 0x1E) | this->get_effective_f0());
            return;
        }
        case FUNCTION5:
        {
            packet->add_dcc_function5_8(this->p.fn_ >> 5);
            return;
        }
        case FUNCTION9:
        {
            packet->add_dcc_function9_12(this->p.fn_ >> 9);
            return;
        }
        case FUNCTION13:
        {
            packet->add_dcc_function13_20(this->p.fn_ >> 13);
            return;
        }
        case FUNCTION21:
        {
            packet->add_dcc_function21_28(this->p.fn_ >> 21);
            return;
        }
        case ESTOP:
        {
            this->p.add_dcc_estop_to_packet(packet);
            packet->packet_header.rept_count = 3;
            return;
        }
        default:
            LOG(WARNING, "Unknown packet generation code: %x", code);
        // fall through
        case SPEED:
        {
            if (this->p.directionChanged_)
            {
                // packet->packet_header.rept_count = 2;
                this->p.directionChanged_ = 0;
            }
            // packet->packet_header.rept_count = 1;
            this->p.add_dcc_speed_to_packet(packet);
            return;
        }
    }
}

MMOldTrain::MMOldTrain(MMAddress a)
{
    p.address_ = a.value;
    packet_processor_add_refresh_source(this);
}

MMOldTrain::~MMOldTrain()
{
    packet_processor_remove_refresh_source(this);
}

// Generates next outgoing packet.
void MMOldTrain::get_next_packet(unsigned code, Packet *packet)
{
    packet->start_mm_packet();
    packet->add_mm_address(MMAddress(p.address_), get_effective_f0());

    if (code == ESTOP)
    {
        packet->add_mm_speed(
            Packet::EMERGENCY_STOP); // will change the direction.
        p.direction_ = !p.direction_;
        p.directionChanged_ = 0;
    }
    else if (p.directionChanged_)
    {
        packet->add_mm_speed(Packet::CHANGE_DIR);
        p.directionChanged_ = 0;
    }
    else
    {
        packet->add_mm_speed(p.speed_);
        if (code != REFRESH)
        {
            packet->packet_header.rept_count = 2;
        }
    }
}

MMNewTrain::MMNewTrain(MMAddress a)
{
    p.address_ = a.value;
    packet_processor_add_refresh_source(this);
}

MMNewTrain::~MMNewTrain()
{
    packet_processor_remove_refresh_source(this);
}

// Generates next outgoing packet.
void MMNewTrain::get_next_packet(unsigned code, Packet *packet)
{
    packet->start_mm_packet();
    packet->add_mm_address(MMAddress(p.address_), get_effective_f0());

    if (code == REFRESH)
    {
        unsigned r = p.nextRefresh_;
        if ((r & 1) == 0) {
            code = SPEED;
        } else {
            // TODO(bracz): check if this refresh cycle confuses the marklin
            // engines' directional state.
            r >>= 1;
            r += MM_F1;
            code = r;
        }
        if (p.nextRefresh_ == MM_MAX_REFRESH) {
            p.nextRefresh_ = 0;
        } else {
            ++p.nextRefresh_;
        }
    }
    else
    {
        packet->packet_header.rept_count = 2;
    }
    if (code == ESTOP)
    {
        packet->add_mm_new_speed(
            !p.direction_,
            Packet::EMERGENCY_STOP); // will change the direction.
        p.direction_ = !p.direction_;
        p.directionChanged_ = 0;
    }
    else if (code == SPEED)
    {
        if (p.directionChanged_)
        {
            packet->add_mm_new_speed(!p.direction_, Packet::CHANGE_DIR);
            p.directionChanged_ = 0;
            p.nextRefresh_ = 0; // sends another speed packet
            packet->mm_shift();
        }
        else
        {
            packet->add_mm_new_speed(!p.direction_, p.speed_);
            packet->mm_shift();
        }
    }
    else if (MM_F1 <= code && code <= MM_F4)
    {
        unsigned fnum = code + 1 - MM_F1;
        packet->add_mm_new_fn(fnum, p.fn_ & (1 << fnum), p.speed_);
        //packet->mm_shift();
        //packet->add_mm_new_speed(!p.direction_, p.speed_);
    }
}

void createtrains() {
    Dcc28Train train1(DccShortAddress(1));
    Dcc128Train train2(DccShortAddress(1));
    MMNewTrain train3(MMAddress(1));
    MMOldTrain train4(MMAddress(1));
}

} // namespace dcc
