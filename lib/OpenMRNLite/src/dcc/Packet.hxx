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
 * \file Packet.hxx
 *
 * Defines a DCC Packet structure.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#ifndef _DCC_PACKET_HXX_
#define _DCC_PACKET_HXX_

#include <stdint.h>
#include <string.h>

#include "dcc/Address.hxx"
#include "dcc/packet.h"

namespace dcc
{

/** Represents a command to be sent to the track driver. Most of the commands
 * are "send packet X", but there are some that are controlling the track
 * booster itself, such as "power off". */
struct Packet : public DCCPacket
{
    /** Maximum number of payload bytes. */
    static const unsigned MAX_PAYLOAD = DCC_PACKET_MAX_PAYLOAD;
    /** Send this speed step to emergency-stop the locomotive. */
    static const unsigned EMERGENCY_STOP = DCC_PACKET_EMERGENCY_STOP;
    /** Send this speed step to switch direction of the locomotive. Only used
     * for marklin-14-step speed commands. */
    static const unsigned CHANGE_DIR = DCC_PACKET_EMERGENCY_STOP;

    Packet()
    {
        clear();
    }

    struct DCC_IDLE {};
    /// Constructor generating a DCC IDLE packet. @param i indicator type.
    Packet(DCC_IDLE i) {
        clear();
        set_dcc_idle();
    }

    /// Resets the packet ot empty.
    void clear()
    {
        memset((DCCPacket*)this, 0, sizeof(*this));
    }

    /** @return true if this is a packet, false if it is a command to the
     * track processor. */
    bool IsPacket()
    {
        return packet_header.is_pkt;
    }

    /// Sets the packet to a standalone command. @param cmd is the standalone
    /// command to send.
    void set_cmd(uint8_t cmd)
    {
        dlc = 0;
        HASSERT(cmd & 1);
        header_raw_data = cmd;
    }

    /** Initializes the packet structure for a regular DCC packet. */
    void start_dcc_packet()
    {
        header_raw_data = 0;
        dlc = 0;
    }

    /** Initializes the packet structure for a regular DCC packet. */
    void start_dcc_svc_packet()
    {
        header_raw_data = 0;
        dlc = 0;
        packet_header.send_long_preamble = true;
    }

    /// Adds the header to the packet needed for addressing a DCC
    /// locomotive. @param address is the DCC (short) address.
    void add_dcc_address(DccShortAddress address);
    /// Adds the header to the packet needed for addressing a DCC
    /// locomotive. @param address is the DCC (long) address.
    void add_dcc_address(DccLongAddress address);

    /** Adds a speed-and-direction command (dcc baseline command) ot the
     * packet. Speed is maximum 14. This should be called after
     * add_dcc_address. */
    /// @param is_fwd true for forward speed
    /// @param light true for light on, false for off
    /// @param speed the speed step to set (0..14)
    void add_dcc_speed14(bool is_fwd, bool light, unsigned speed);
    /** Adds a speed-and-direction command (dcc baseline command) ot the
     * packet. Speed is maximum 14. This should be called after
     * add_dcc_address. */
    /// @param a the DCC address
    /// @param is_fwd true for forward speed
    /// @param light true for light on, false for off
    /// @param speed the speed step to set (0..14)
    template <class A>
    void set_dcc_speed14(A a, bool is_fwd, bool light, unsigned speed)
    {
        add_dcc_address(a);
        add_dcc_speed14(is_fwd, light, speed);
    }

    /** Adds a speed-and-direction command (dcc baseline command) to the
     * packet. Speed is maximum 28. This should be called after
     * add_dcc_address. */
    /// @param is_fwd true for forward speed
    /// @param speed the speed step to set (0..14)
    void add_dcc_speed28(bool is_fwd, unsigned speed);
    /// @param a the DCC address
    /// @param is_fwd true for forward speed
    /// @param speed the speed step to set (0..14)
    template <class A> void set_dcc_speed28(A a, bool is_fwd, unsigned speed)
    {
        add_dcc_address(a);
        add_dcc_speed28(is_fwd, speed);
    }

    /** Adds a speed-and-direction command (dcc extended command) for 128 speed
     * steps to the packet. Speed is maximum 126. This shoudl be called after
     * add_dcc_address. */
    /// @param is_fwd true for forward speed
    /// @param speed the speed step to set (0..14)
    void add_dcc_speed128(bool is_fwd, unsigned speed);
    /// @param a the DCC address
    /// @param is_fwd true for forward speed
    /// @param speed the speed step to set (0..14)
    template <class A> void set_dcc_speed128(A a, bool is_fwd, unsigned speed)
    {
        add_dcc_address(a);
        add_dcc_speed128(is_fwd, speed);
    }

    /** Adds a DCC function group command to the packet. The lowest numbered
     * function is always at bit zero. @param values are bitmask of functions
     * to send to the loco. */
    void add_dcc_function0_4(unsigned values);
    /** Adds a DCC function group command to the packet. The lowest numbered
     * function is always at bit zero. @param values are bitmask of functions
     * to send to the loco. */
    void add_dcc_function5_8(unsigned values);
    /** Adds a DCC function group command to the packet. The lowest numbered
     * function is always at bit zero. @param values are bitmask of functions
     * to send to the loco. */
    void add_dcc_function9_12(unsigned values);
    /** Adds a DCC function group command to the packet. The lowest numbered
     * function is always at bit zero. @param values are bitmask of functions
     * to send to the loco. */
    void add_dcc_function13_20(unsigned values);
    /** Adds a DCC function group command to the packet. The lowest numbered
     * function is always at bit zero. @param values are bitmask of functions
     * to send to the loco. */
    void add_dcc_function21_28(unsigned values);

    /** Helper function for adding programming mode packets. */
    void add_dcc_prog_command(
        uint8_t cmd_hi, unsigned cv_number, uint8_t value);

    /** Adds a DCC POM read single CV command and the xor byte. This should be
     * called after add_dcc_address. @param cv_number which CV to read. */
    void add_dcc_pom_read1(unsigned cv_number);

    /** Adds a DCC POM write single CV command and the xor byte. This should be
     * called after add_dcc_address.
     * @param cv_number which CV to write - 1,
     * @param value is the value to set it to. */
    void add_dcc_pom_write1(unsigned cv_number, uint8_t value);

    /** Sets the packet to a DCC service mode packet verifying the contents of
     * an entire CV. This function does not need a DCC address. (Includes the
     * checksum.)
     * @param cv_number which CV to test - 1,
     * @param value is the value to test. */
    void set_dcc_svc_verify_byte(unsigned cv_number, uint8_t value);

    /** Sets the packet to a DCC service mode packet writing the contents of
     * an entire CV. This function does not need a DCC address. (Includes the
     * checksum.)
     * @param cv_number which CV to write - 1,
     * @param value is the value to write. */
    void set_dcc_svc_write_byte(unsigned cv_number, uint8_t value);

    /** Sets the packet to a DCC service mode packet verifying the contents of
     * a single bit in a CV. This function does not need a DCC address.
     * (Includes the checksum.)
     * @param cv_number which CV to test - 1,
     * @param bit is 0..7 to set which bit to test
     * @param desired is true if bit==1 should be tested */
    void set_dcc_svc_verify_bit(
        unsigned cv_number, unsigned bit, bool expected);

    /** Sets the packet to a DCC service mode packet verifying the contents of
     * a single bit in a CV. This function does not need a DCC address.
     * (Includes the checksum.)
     * @param cv_number which CV to edit - 1,
     * @param bit is 0..7 to define which bit to edit
     * @param desired is true if bit:=1 should be written */
    void set_dcc_svc_write_bit(unsigned cv_number, unsigned bit, bool desired);

    /** Adds a DCC basic accessory decoder command packet and the checksum
     * byte.
     * @param address is the unencoded 12-bit address, containing both the A
     * bits and the D bits in the DCC standard, in the range of 0..4095. The
     * values of 4088-4095 are broadcast addresses.
     * @param is_activate is true for setting bit C to one, false for setting
     * bit C to zero. Usually commandstations set is_activate to one and use
     * the lowest bit of the address to decide whether the turnouts should be
     * closed or thrown.
     */
    void add_dcc_basic_accessory(unsigned address, bool is_activate);
    
    /** Appends one byte to the packet payload that represents the XOR checksum
     * for DCC. */
    void add_dcc_checksum();

    /** Creates a DCC idle packet. (Includes the checksum.) */
    void set_dcc_idle();

    /** Creates a DCC reset-all-decoders packet. (Includes the checksum.) */
    void set_dcc_reset_all_decoders();

    /** Sets the packet type to marklin-motorola. Initilizes the packet with 18
     * zero bits. */
    void start_mm_packet();

    /** Sets the address and F0 bits of an MM packet to a specific loco
     * address. */
    void add_mm_address(MMAddress address, bool light);

    /** Sets the packet to a 14-step MM speed-and-light packet. Max value of
     * speed is 14. A special value of speed == CHANGE_DIR will signal
     * direction change. */
    void add_mm_speed(unsigned speed);

    /** Sets the packet to a direction-aware 14-step MM speed-and-light
     * packet. Max value of speed is 14. */
    /// @param is_fwd true for forward speed
    /// @param speed the speed step to set (0..14)
    void add_mm_new_speed(bool is_fwd, unsigned speed);

    /** Creates a speed-and-fn packet for the new MM format.
     * @param fn_num is the function, valid values = 1..4
     * @param value is whether the funciton is on or off
     * @param speed is the speed step (0..14). If it is set to emergency stop,
     * then no function packet will be generated.
     */
    void add_mm_new_fn(unsigned fn_num, bool value, unsigned speed);

    /** Shifts a MM packet to the second half of the packet buffer. After this
     * call another add_mm_speed call is valid, which will fill in the first
     * half of the double packet. */
    void mm_shift();

private:
    /** Sets the speed bits of an MM packet. Clips speed to 15, avoids speed==1
     * and returns the final speed step number. @param speed is the raw speed
     * step value (0..14 but more are accepted). @return the final speed step
     * number */
    unsigned set_mm_speed_bits(unsigned speed);
};

} // namespace dcc

#endif // _DCC_PACKET_HXX_
