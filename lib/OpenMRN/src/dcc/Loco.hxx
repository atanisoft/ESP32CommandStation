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
 * \file Loco.hxx
 *
 * Defines a simple DCC locomotive.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#ifndef _DCC_LOCO_HXX_
#define _DCC_LOCO_HXX_

#include "dcc/Packet.hxx"
#include "dcc/PacketSource.hxx"
#include "dcc/UpdateLoop.hxx"
#include "dcc/Defs.hxx"
#include "utils/logging.h"

namespace dcc
{

/// Describes what sort of packet the dcc:PacketSource (usually a single train)
/// should generate. This is used for two purposes:
///
/// - When a user action results in a high priority band packet to be enerated,
///   we can enqueue a message containing this code to the refreshloop
///   object. Then when the refresh loop gets to the given packet, the train
///   will be called with the appropriate code to generate the desired packet
///   from its internal state (the freshest version of that).
///
/// - When a train is on the background refresh loop, it can keep an internal
///   variable describing what packet to generate next as a refresh packet.
enum DccTrainUpdateCode
{
    REFRESH = 0,
    SPEED = 1,
    FUNCTION0 = 2,
    FUNCTION5 = 3,
    FUNCTION9 = 4,
    FUNCTION13 = 5,
    FUNCTION21 = 6,
    MM_F1 = 2,
    MM_F2,
    MM_F3,
    MM_F4,
    MIN_REFRESH = SPEED,
    /** @TODO(balazs.racz) choose adaptive max-refresh based on how many
     * functions are actually in use for the loco. */
    MAX_REFRESH = FUNCTION9,
    MM_MAX_REFRESH = 7,
    ESTOP = 16,
};

/// AbstractTrain is a templated class for train implementations in a command
/// station. It gives implementations for most functions that the OpenLCB
/// command station neeeds, while using a compact structure for representing
/// the state the command station needs to know about the train itself.
///
/// The only shared assumption about the train is that it has to participate in
/// the standard command station packet loop. The exact protocol, number of
/// speed steps, number of functions etc. are all defined by the template
/// argument. The goal is to minimize the number of bytes needed to store
/// information about one train in the RAM so that command stations with
/// limited memory can still serve a large number of trains in their update
/// loop.
///
/// Example implementations of the template payload are @ref Dcc28Payload, @ref
/// Dcc128Payload, @ref MMOldPayload, @ref MMNewPayload.
template <class P> class AbstractTrain : public PacketSource
{
public:
    AbstractTrain()
    {
    }

    /// Sets the train speed (asking for a high-priority outgoing update packet
    /// to be generated). @param speed is the desired speed that came from the
    /// throttle.
    void set_speed(SpeedType speed) OVERRIDE
    {
        float16_t new_speed = speed.get_wire();
        if (p.lastSetSpeed_ == new_speed)
        {
            LOG(VERBOSE, "not updating speed: old speed %04x, new speed %04x",
                p.lastSetSpeed_, new_speed);
            return;
        }
        p.lastSetSpeed_ = new_speed;
        if (speed.direction() != p.direction_)
        {
            p.directionChanged_ = 1;
            p.direction_ = speed.direction();
        }
        float f_speed = speed.mph();
        if (f_speed > 0)
        {
            f_speed *= ((p.get_speed_steps() * 1.0) / 126);
            unsigned sp = f_speed;
            sp++; // makes sure it is at least speed step 1.
            if (sp > p.get_speed_steps())
                sp = p.get_speed_steps();
            LOG(VERBOSE, "set speed to step %u", sp);
            p.speed_ = sp;
        }
        else
        {
            p.speed_ = 0;
        }
        packet_processor_notify_update(this, SPEED);
    }

    /// @return the last set speed.
    SpeedType get_speed() OVERRIDE
    {
        SpeedType v;
        v.set_wire(p.lastSetSpeed_);
        return v;
    }
    /// @return the commanded speed (which as of now is interpreted as the last
    /// set speed).
    SpeedType get_commanded_speed() OVERRIDE
    {
        return get_speed();
    }
    /// Sets the train to ESTOP state, generating an emergency stop packet.
    void set_emergencystop() OVERRIDE
    {
        p.speed_ = 0;
        SpeedType dir0;
        dir0.set_direction(p.direction_);
        p.lastSetSpeed_ = dir0.get_wire();
        p.directionChanged_ = 1;
        /// @todo (Stuart.Baker) We should not just send a single E-Stop burst.
        /// It is possible that the loco was on dirt and missed this.  Should
        /// send continuous E-Stop packets until the estop condition is cleared.
        packet_processor_notify_update(this, ESTOP);
    }
    /// Gets the train's ESTOP state.
    bool get_emergencystop() OVERRIDE
    {
        return false;
    }
    /// Sets a function to a given value. @param address is the function number
    /// (0..28), @param value is 0 for funciton OFF, 1 for function ON.
    void set_fn(uint32_t address, uint16_t value) OVERRIDE
    {
        if (address > p.get_max_fn())
        {
            // Ignore.
            return;
        }
        unsigned bit = 1 << address;
        if (value)
        {
            p.fn_ |= bit;
        }
        else
        {
            p.fn_ &= ~bit;
        }
        packet_processor_notify_update(this, p.get_fn_update_code(address));
    }
    /// @return the last set value of a given function, or 0 if the function is
    /// not known. @param address is the function address.
    uint16_t get_fn(uint32_t address) OVERRIDE
    {
        if (address > p.get_max_fn())
        {
            // Unknown.
            return 0;
        }
        return (p.fn_ & (1 << address)) ? 1 : 0;
    }
    /// @return the legacy address of this loco.
    uint32_t legacy_address() OVERRIDE
    {
        return p.address_;
    }
    /// @return the legacy address type.
    TrainAddressType legacy_address_type() OVERRIDE
    {
        return p.get_address_type();
    }

protected:
    /// Payload -- actual data we know about the train.
    P p;
};

/// Structure defining the volatile state for a 28-speed-step DCC locomotive.
struct Dcc28Payload
{
    Dcc28Payload()
    {
        memset(this, 0, sizeof(*this));
    }
    /// Track address. largest address allowed is 10239.
    unsigned address_ : 14;
    /// 1 if this is a short address train.
    unsigned isShortAddress_ : 1;
    /// 0: forward, 1: reverse
    unsigned direction_ : 1;
    /// fp16 value of the last set speed.
    unsigned lastSetSpeed_ : 16;
    /// functions f0-f28.
    unsigned fn_ : 29;
    /// Which refresh packet should go out next.
    unsigned nextRefresh_ : 3;
    /// Speed step we last set.
    unsigned speed_ : 5;
    /// Whether the direction change packet still needs to go out.
    unsigned directionChanged_ : 1;

    /** @return the number of speed steps (in float). */
    static unsigned get_speed_steps()
    {
        return 28;
    }

    /** @returns the largest function number that is still valid. */
    static unsigned get_max_fn()
    {
        return 28;
    }

    /** @return the update code to send ot the packet handler for a given
     * function value change. @param address is the function number(0..28). */
    static unsigned get_fn_update_code(unsigned address);

    /** Adds the speed payload to a DCC packet. @param p is the packet to add
     * the speed payload to. */
    void add_dcc_speed_to_packet(dcc::Packet *p)
    {
        p->add_dcc_speed28(!direction_, speed_);
    }

    /** Adds the speed payload to a DCC packet with value == EMERGENCY_STOP
     * @param p is the packet to add the speed payload to. */
    void add_dcc_estop_to_packet(dcc::Packet *p)
    {
        p->add_dcc_speed28(!direction_, Packet::EMERGENCY_STOP);
    }

    /// @return what type of address this train has.
    TrainAddressType get_address_type()
    {
        return isShortAddress_ ? TrainAddressType::DCC_SHORT_ADDRESS : TrainAddressType::DCC_LONG_ADDRESS;
    }
};

/// TrainImpl class for a DCC locomotive.
template <class Payload> class DccTrain : public AbstractTrain<Payload>
{
public:
    /// Constructor. @param a is the address.
    DccTrain(DccShortAddress a)
    {
        this->p.isShortAddress_ = 1;
        this->p.address_ = a.value;
        packet_processor_add_refresh_source(this);
    }

    /// Constructor. @param a is the address.
    DccTrain(DccLongAddress a)
    {
        this->p.isShortAddress_ = 0;
        this->p.address_ = a.value;
        packet_processor_add_refresh_source(this);
    }

    ~DccTrain();

    /// Generates next outgoing packet. @param code is the packet code (as
    /// requested by the previous cycle or the on-update notification). @param
    /// packet needs to be filled in for the output.
    void get_next_packet(unsigned code, Packet *packet) OVERRIDE;
};

/// TrainImpl class for a 28-speed-step DCC locomotive.
typedef DccTrain<Dcc28Payload> Dcc28Train;

/// Structure defining the volatile state for a 128-speed-step DCC locomotive.
struct Dcc128Payload
{
    Dcc128Payload()
    {
        memset(this, 0, sizeof(*this));
    }
    /// Track address. largest address allowed is 10239.
    unsigned address_ : 14;
    /// 1 if this is a short address train.
    unsigned isShortAddress_ : 1;
    /// 0: forward, 1: reverse
    unsigned direction_ : 1;
    /// fp16 value of the last set speed.
    unsigned lastSetSpeed_ : 16;
    /// functions f0-f28.
    unsigned fn_ : 29;
    /// Which refresh packet should go out next.
    unsigned nextRefresh_ : 3;
    /// Speed step we last set.
    unsigned speed_ : 7;
    /// Whether the direction change packet still needs to go out.
    unsigned directionChanged_ : 1;

    /** @return the number of speed steps (the largest valid speed step). */
    static unsigned get_speed_steps()
    {
        return 126;
    }

    /** @return the largest function number that is still valid. */
    static unsigned get_max_fn()
    {
        return 28;
    }

    /** @return the update code to send ot the packet handler for a given
     * function value change. */
    static unsigned get_fn_update_code(unsigned address)
    {
        return Dcc28Payload::get_fn_update_code(address);
    }

    /** Adds the speed payload to a DCC packet. @param p is the packet to add
     * the speed payload to. */
    void add_dcc_speed_to_packet(dcc::Packet *p)
    {
        p->add_dcc_speed128(!direction_, speed_);
    }

    /** Adds the speed payload to a DCC packet with value == EMERGENCY_STOP
     * @param p is the packet to add the speed payload to. */
    void add_dcc_estop_to_packet(dcc::Packet *p)
    {
        p->add_dcc_speed128(!direction_, Packet::EMERGENCY_STOP);
    }

    /// @return what type of address this train has.
    TrainAddressType get_address_type()
    {
        return isShortAddress_ ? TrainAddressType::DCC_SHORT_ADDRESS : TrainAddressType::DCC_LONG_ADDRESS;
    }
};

/// TrainImpl class for a 128-speed-step DCC locomotive.
typedef DccTrain<Dcc128Payload> Dcc128Train;

/// Structure defining the volatile state for a Marklin-Motorola v1 protocol
/// locomotive (with 14 speed steps, one function and relative direction only).
struct MMOldPayload
{
    MMOldPayload()
    {
        memset(this, 0, sizeof(*this));
    }
    /// largest address allowed is 80, but we keep a few more bits around to
    /// allow for an extension to arbitrary MM address packets.
    unsigned address_ : 8;
    /// fp16 value of the last set speed.
    unsigned lastSetSpeed_ : 16;
    /// function f0.
    unsigned fn_ : 1;
    /// 0: forward, 1: reverse
    unsigned direction_ : 1;
    /// Whether the direction change packet still needs to go out.
    unsigned directionChanged_ : 1;
    /// Speed step we last set.
    unsigned speed_ : 4;

    /** @return the number of speed steps (in float). */
    unsigned get_speed_steps()
    {
        return 14;
    }

    /** @return the largest function number that is still valid. */
    unsigned get_max_fn()
    {
        return 0;
    }

    /** @return the update code to send to the packet handler for a given
     * function value change. @param address is ignored */
    unsigned get_fn_update_code(unsigned address)
    {
        return SPEED;
    }

    /// @return what type of address this train has.
    static TrainAddressType get_address_type()
    {
        return TrainAddressType::MM;
    }
};

/// TrainImpl structure for Marklin-Motorola v1 protocol locomotives.
class MMOldTrain : public AbstractTrain<MMOldPayload>
{
public:
    /// Constructor. @param a is the MM address.
    MMOldTrain(MMAddress a);
    ~MMOldTrain();

    /// Generates next outgoing packet. @param code is the packet code (as
    /// requested by the previous cycle or the on-update notification). @param
    /// packet needs to be filled in for the output.
    void get_next_packet(unsigned code, Packet *packet) OVERRIDE;
};

/// Structure defining the volatile state for a Marklin-Motorola v2 protocol
/// locomotive (with 28 speed steps, five functions and absolute direction).
struct MMNewPayload
{
    MMNewPayload()
    {
        memset(this, 0, sizeof(*this));
    }
    /// largest address allowed is 80, but we keep a few more bits around to
    /// allow for an extension to arbitrary MM address packets.
    unsigned address_ : 8;
    /// fp16 value of the last set speed.
    unsigned lastSetSpeed_ : 16;
    /// function f0-f4.
    unsigned fn_ : 5;
    /// 0: forward, 1: reverse
    unsigned direction_ : 1;
    /// Whether the direction change packet still needs to go out.
    unsigned directionChanged_ : 1;
    /// reserved
    unsigned resvd1_ : 1;
    /// Speed step we last set.
    unsigned speed_ : 4;
    /// internal refresh cycle state machine
    unsigned nextRefresh_ : 3;

    /** @return the number of speed steps (in float). */
    unsigned get_speed_steps()
    {
        return 14;
    }

    /** @return the largest function number that is still valid. */
    unsigned get_max_fn()
    {
        return 4;
    }

    /** @return the update code to send to the packet handler for a given
     * function value change. @param address is the function number (0..4) */
    unsigned get_fn_update_code(unsigned address)
    {
        if (1 <= address && address <= 4)
        {
            return MM_F1 + address - 1;
        }
        else
        {
            return SPEED;
        }
    }

    /// @return what type of address this train has.
    static TrainAddressType get_address_type()
    {
        return TrainAddressType::MM;
    }
};

/// TrainImpl structure for Marklin-Motorola v2 protocol locomotives.
class MMNewTrain : public AbstractTrain<MMNewPayload>
{
public:
    /// Constructor. @param a is the MM address.
    MMNewTrain(MMAddress a);
    ~MMNewTrain();

    /// Generates next outgoing packet. @param code is the packet code (as
    /// requested by the previous cycle or the on-update notification). @param
    /// packet needs to be filled in for the output.
    void get_next_packet(unsigned code, Packet *packet) OVERRIDE;
};

} // namespace dcc

#endif // _DCC_LOCO_HXX_
