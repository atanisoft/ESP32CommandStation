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
 * \file TractionDefs.hxx
 *
 * Static definitions for implementations of the NMRAnet Traction and Traction
 * Proxy protocols.
 *
 * @author Balazs Racz
 * @date 5 May 2014
 */

#ifndef _OPENLCB_TRACTIONDEFS_HXX_
#define _OPENLCB_TRACTIONDEFS_HXX_

#include <cmath>
#include <stdint.h>

#include "dcc/Defs.hxx"
#include "openlcb/If.hxx"
#include "openlcb/Payload.hxx"
#include "openlcb/Velocity.hxx"
#include "utils/format_utils.hxx"

namespace openlcb {

/// Represents an OpenLCB speed value with accessors to convert to and from
/// various formats.
typedef Velocity SpeedType;

/** Parses a SpeedType value from an unaligned memory address, typically from
 * the input buffer. */
SpeedType fp16_to_speed(const void *fp16);

/** Renders a SpeedType value to an unaligned memory address, typically to the
 * output buffer.
 *
 * @param speed is the speed to write
 * @param fp16 is an unaligned two-byte location to write the float16 value
 * to.*/
void speed_to_fp16(SpeedType speed, void *fp16);

/** @returns NAN as speed. */
inline SpeedType nan_to_speed() {
    SpeedType s;
    s.set_wire(0xFFFFU);
    return s;
}

/// Static constants and helper functions for the Traciton protocol family.
struct TractionDefs {
    /// This event should be produced by train nodes.
    static const uint64_t IS_TRAIN_EVENT = 0x0101000000000303ULL;
    /// This event should be produced by traction proxy nodes.
    static const uint64_t IS_PROXY_EVENT = 0x0101000000000304ULL;
    /// Base address of DCC accessory decoder well-known event range (active)
    static constexpr uint64_t ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE = 0x0101020000FF0000ULL;
    /// Base address of DCC accessory decoder well-known event range (inactive)
    static constexpr uint64_t INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE = 0x0101020000FE0000ULL;
    /// Node ID space allocated for DC blocks.
    static const uint64_t NODE_ID_DC_BLOCK = 0x060000000000ULL;
    /// Node ID space allocated for DCC locomotives.
    static const uint64_t NODE_ID_DCC = 0x060100000000ULL;
    /// Node ID space allocated for TMCC protocol.
    static const uint64_t NODE_ID_TMCC = 0x060200000000ULL;
    /// Node ID space allocated for the Marklin-Motorola protocol.
    static const uint64_t NODE_ID_MARKLIN_MOTOROLA = 0x060300000000ULL;
    /// Node ID space allocated for the MTH DCS protocol.
    static const uint64_t NODE_ID_MTH_DCS = 0x060400000000ULL;
    /// Node ID space mask.
    static const uint64_t NODE_ID_MASK = 0xFFFF00000000ULL;

    enum
    {
        // Byte 0 of request commands.
        REQ_SET_SPEED = 0x00,
        REQ_SET_FN = 0x01,
        REQ_EMERGENCY_STOP = 0x02,

        REQ_QUERY_SPEED = 0x10,
        REQ_QUERY_FN = 0x11,

        REQ_CONTROLLER_CONFIG = 0x20,
        REQ_CONSIST_CONFIG = 0x30,
        REQ_TRACTION_MGMT = 0x40,

        // Byte 1 of REQ_CONTROLLER_CONFIG command
        CTRLREQ_ASSIGN_CONTROLLER = 0x01,
        CTRLREQ_RELEASE_CONTROLLER = 0x02,
        CTRLREQ_QUERY_CONTROLLER = 0x03,
        CTRLREQ_NOTIFY_CONTROLLER_CHANGED = 0x04,

        // Byte 1 of REQ_CONSIST_CONFIG command
        CNSTREQ_ATTACH_NODE = 0x01,
        CNSTREQ_DETACH_NODE = 0x02,
        CNSTREQ_QUERY_NODES = 0x03,

        // Byte 1 of REQ_TRACTION_MGMT command
        MGMTREQ_RESERVE = 0x01,
        MGMTREQ_RELEASE = 0x02,
        // Byte 0 of response commands
        RESP_QUERY_SPEED = REQ_QUERY_SPEED,
        RESP_QUERY_FN = REQ_QUERY_FN,
        RESP_CONTROLLER_CONFIG = REQ_CONTROLLER_CONFIG,
        RESP_CONSIST_CONFIG = REQ_CONSIST_CONFIG,
        RESP_TRACTION_MGMT = REQ_TRACTION_MGMT,

        // Byte 1 of Controller Configuration response
        CTRLRESP_ASSIGN_CONTROLLER = CTRLREQ_ASSIGN_CONTROLLER,
        CTRLRESP_QUERY_CONTROLLER = CTRLREQ_QUERY_CONTROLLER,
        CTRLRESP_NOTIFY_CONTROLLER_CHANGED = CTRLREQ_NOTIFY_CONTROLLER_CHANGED,

        // Byte 2 of assign controller response
        CTRLRESP_ASSIGN_ERROR_TRAIN = 0x02,
        CTRLRESP_ASSIGN_ERROR_CONTROLLER = 0x01,

        // Byte 1 of Consist Configuration response
        CNSTRESP_ATTACH_NODE = CNSTREQ_ATTACH_NODE,
        CNSTRESP_DETACH_NODE = CNSTREQ_DETACH_NODE,
        CNSTRESP_QUERY_NODES = CNSTREQ_QUERY_NODES,

        CNSTFLAGS_ALIASVALID = 0x01,
        CNSTFLAGS_REVERSE = 0x02,
        CNSTFLAGS_LINKF0 = 0x04,
        CNSTFLAGS_LINKFN = 0x08,
        CNSTFLAGS_HIDE = 0x80,

        // Byte 1 of Traction Management replies
        MGMTRESP_RESERVE = MGMTREQ_RESERVE,

        PROXYREQ_ALLOCATE = 0x01,
        PROXYREQ_ATTACH = 0x02,
        PROXYREQ_DETACH = 0x03,
        PROXYREQ_MANAGE = 0x80,

        PROXYRESP_ALLOCATE = PROXYREQ_ALLOCATE,
        PROXYRESP_ATTACH = PROXYREQ_ATTACH,
        PROXYRESP_MANAGE = PROXYREQ_MANAGE,

        // byte 1 of PROXYREQ_MANAGE commands
        PROXYREQ_MANAGE_RESERVE = 0x01,
        PROXYREQ_MANAGE_RELEASE = 0x02,

        PROXYRESP_MANAGE_RESERVE_REPLY = 0x01,

        // Legacy Technology IDs from the GenTractionProxyWN.
        PROXYTYPE_DCC = 1,
        PROXYTYPE_DC = 2,
        PROXYTYPE_MARKLIN_DIGITAL = 3,
        PROXYTYPE_MARKLIN_DELTA = 4,
        PROXYTYPE_MARKLIN_MFX = 5,
        PROXYTYPE_SELECTRIX = 6,
        PROXYTYPE_MTH_DCS = 7,
        PROXYTYPE_LIONEL_TMCC = 8,

        /** This is the memory space number for accessing an NMRA DCC
         * locomotive's functions via the memory config protocol. */
        FUNCTION_CONFIG_MEMORY_SPACE = 0xF9,
        // These declare the configuration space layout. They are always the
        // third byte of the address in the configuration memory space, aka
        // address = type << 16 + offset.
        /** F0-F28 are at offset 0 to 28 here. */
        FNCONFIG_FN = 0x0,
        /** Binary State Control Instruction long form. Offset 0 to 32767 */
        FNCONFIG_BINARYSTATE_LONG = 0x1,
        /** Binary State Control Instruction short form. Offset 0 to 127 */
        FNCONFIG_BINARYSTATE_SHORT = 0x2,
        /** Analog outputs, defined by NMRA DCC WG Topic 9910241. Offset 0-255,
         * values 0-255 each. */
        FNCONFIG_ANALOG_OUTPUT = 0x3
    };

    /** Converts a legacy address to an NMRAnet node ID.

        The conversion algorithm chosen here is rather arbitrary, as it is not
        specified in the standard how the individual reserved blocks are laid
        out internally to addresses. The only important thing is that for
        different track bit patterns a different node ID be generated.

      @param type defines what address type it is (dcc-short, dcc-long or MM)
      @param addr is the legacy address, the valid values are defined by the
      specific protocols.
    */
    static NodeID train_node_id_from_legacy(
        dcc::TrainAddressType type, uint32_t addr)
    {
        switch (type)
        {
            case dcc::TrainAddressType::DCC_SHORT_ADDRESS:
                return NODE_ID_DCC | addr;
            case dcc::TrainAddressType::DCC_LONG_ADDRESS:
                if (addr < 128)
                {
                    return NODE_ID_DCC | 0xC000 | addr;
                }
                else
                {
                    return NODE_ID_DCC | addr;
                }
            case dcc::TrainAddressType::MM:
                return NODE_ID_MARKLIN_MOTOROLA | addr;
            default:
                DIE("Unknown train address type");
        }
    }

    /** Converts a node ID to a legacy address if possible.
        @param id is an openLCB NodeID
        @param type (must be not null) will be filled with the legacy train
       address type
        @param addr (must be not null) will be filled with the legacy train
       address.
        @return true if the address was recognized and type, addr was filled
       with values. Returns false if the address was not recognized as a legacy
       train node's address.
    */
    static bool legacy_address_from_train_node_id(
        NodeID id, dcc::TrainAddressType *type, uint32_t *addr)
    {
        HASSERT(type);
        HASSERT(addr);
        if ((id & NODE_ID_MASK) == NODE_ID_DCC)
        {
            *addr = (id & 0x3FFF);
            if (((id & 0xC000) == 0xC000) || (*addr >= 128u))
            {
                // overlapping long address
                *type = dcc::TrainAddressType::DCC_LONG_ADDRESS;
            }
            else
            {
                *type = dcc::TrainAddressType::DCC_SHORT_ADDRESS;
            }
            return true;
        }
        else if ((id & NODE_ID_MASK) == NODE_ID_MARKLIN_MOTOROLA)
        {
            *type = dcc::TrainAddressType::MM;
            *addr = (id & 0x3FFF);
            return true;
        }
        return false;
    }

    /** Converts a legacy address to a user-visible name.

      @param type defines what address type it is (dcc-short, dcc-long or MM)
      @param addr is the legacy address, the valid values are defined by the
      specific protocols.
      @return user-readable address.
    */
    static string train_node_name_from_legacy(
        dcc::TrainAddressType type, uint32_t addr)
    {
        string ret(14, 0);
        char *s = &ret[0];
        if ((type == dcc::TrainAddressType::DCC_LONG_ADDRESS) && (addr < 128))
        {
            s[0] = '0';
            s++;
        }
        char *e = integer_to_buffer(addr, s);
        ret.resize(e - &ret[0]);
        if (type == dcc::TrainAddressType::MM)
        {
            ret.push_back('M');
        }
        else if (addr < 128)
        {
            if (type == dcc::TrainAddressType::DCC_SHORT_ADDRESS)
            {
                ret.push_back('S');
            }
            else
            {
                ret.push_back('L');
            }
        }
        return ret;
    }

    /** Renders a guess at the traction node name. If it is a recognized train
     * node range, renders te default name for that protocol, otherwise a mac
     * address format.
     *
     * @param node_id is the train node OpenLCB node ID
     * @return a user-visible node name */
    static string guess_train_node_name(NodeID node_id)
    {
        dcc::TrainAddressType decoded_type;
        uint32_t decoded_address = 0xFFFF;
        if (legacy_address_from_train_node_id(
                node_id, &decoded_type, &decoded_address))
        {
            // We recognized this as a legacy address. Use the legacy node name
            // to display.
            return train_node_name_from_legacy(decoded_type, decoded_address);
        }
        else
        {
            // Format the node MAC address.
            uint8_t idbuf[6];
            node_id_to_data(node_id, idbuf); // convert to big-endian
            return mac_to_string(idbuf);
        }
    }

    static Payload estop_set_payload() {
        Payload p(1, 0);
        p[0] = REQ_EMERGENCY_STOP;
        return p;
    }

    static Payload speed_set_payload(Velocity v) {
        Payload p(3, 0);
        p[0] = REQ_SET_SPEED;
        speed_to_fp16(v, &p[1]);
        return p;
    }

    static Payload speed_get_payload() {
        Payload p(1, 0);
        p[0] = REQ_QUERY_SPEED;
        return p;
    }

    /** Parses the response payload of a GET_SPEED packet.
     * @returns true if the last_set_speed value was present and non-NaN.
     * @param p is the response payload.
     * @param v is the velocity that will be set to the speed value. */
    static bool speed_get_parse_last(const Payload &p, Velocity *v)
    {
        if (p.size() < 3)
        {
            return false;
        }
        *v = fp16_to_speed(p.data() + 1);
        if (std::isnan(v->speed()))
        {
            return false;
        }
        return true;
    }

    static Payload fn_set_payload(unsigned address, uint16_t value) {
        Payload p(6, 0);
        p[0] = REQ_SET_FN;
        p[1] = (address >> 16) & 0xff;
        p[2] = (address >> 8) & 0xff;
        p[3] = address & 0xff;
        p[4] = (value >> 8) & 0xff;
        p[5] = value & 0xff;
        return p;
    }

    static Payload fn_get_payload(unsigned address) {
        Payload p(4, 0);
        p[0] = REQ_QUERY_FN;
        p[1] = (address >> 16) & 0xff;
        p[2] = (address >> 8) & 0xff;
        p[3] = address & 0xff;
        return p;
    }

    /** Parses the response payload of a GET_FN packet.
     * @returns true if there is a valid function value.
     * @param p is the response payload.
     * @param value will be set to the output value.
     * @param address will be set to the function address. */
    static bool fn_get_parse(
        const Payload &p, uint16_t *value, unsigned *address)
    {
        if (p.size() < 6)
        {
            return false;
        }
        unsigned num = uint8_t(p[1]);
        num <<= 8;
        num |= uint8_t(p[2]);
        num <<= 8;
        num |= uint8_t(p[3]);
        *address = num;
        *value = (((uint16_t)p[4]) << 8) | uint8_t(p[5]);
        return true;
    }

    static Payload assign_controller_payload(Node *ctrl)
    {
        Payload p(9, 0);
        p[0] = REQ_CONTROLLER_CONFIG;
        p[1] = CTRLREQ_ASSIGN_CONTROLLER;
        p[2] = 0;
        node_id_to_data(ctrl->node_id(), &p[3]);
        return p;
    }

    static Payload release_controller_payload(Node *ctrl)
    {
        Payload p(9, 0);
        p[0] = REQ_CONTROLLER_CONFIG;
        p[1] = CTRLREQ_RELEASE_CONTROLLER;
        p[2] = 0;
        node_id_to_data(ctrl->node_id(), &p[3]);
        return p;
    }

    static Payload consist_add_payload(NodeID slave, uint8_t flags)
    {
        Payload p(9, 0);
        p[0] = REQ_CONSIST_CONFIG;
        p[1] = CNSTREQ_ATTACH_NODE;
        p[2] = flags;
        node_id_to_data(slave, &p[3]);
        return p;
    }

    static Payload consist_add_response(NodeID slave, uint16_t error_code)
    {
        Payload p(10, 0);
        p[0] = RESP_CONSIST_CONFIG;
        p[1] = CNSTRESP_ATTACH_NODE;
        node_id_to_data(slave, &p[2]);
        error_to_data(error_code, &p[8]);
        return p;
    }

    static Payload consist_del_payload(NodeID slave)
    {
        Payload p(9, 0);
        p[0] = REQ_CONSIST_CONFIG;
        p[1] = CNSTREQ_DETACH_NODE;
        node_id_to_data(slave, &p[3]);
        return p;
    }

    static Payload consist_del_response(NodeID slave, uint16_t error_code)
    {
        Payload p = consist_add_response(slave, error_code);
        p[1] = CNSTRESP_DETACH_NODE;
        return p;
    }

    static Payload consist_qry_payload()
    {
        Payload p(2, 0);
        p[0] = REQ_CONSIST_CONFIG;
        p[1] = CNSTREQ_QUERY_NODES;
        return p;
    }

    static Payload consist_qry_payload(uint8_t arg)
    {
        Payload p(3, 0);
        p[0] = REQ_CONSIST_CONFIG;
        p[1] = CNSTREQ_QUERY_NODES;
        p[2] = arg;
        return p;
    }

    static Payload consist_qry_response_short(uint8_t num_entries)
    {
        Payload p(3, 0);
        p[0] = RESP_CONSIST_CONFIG;
        p[1] = CNSTRESP_QUERY_NODES;
        p[2] = num_entries;
        return p;
    }

    static Payload consist_qry_response_long(
        uint8_t num_entries, uint8_t index, uint8_t flags, NodeID slave)
    {
        Payload p(11, 0);
        p[0] = RESP_CONSIST_CONFIG;
        p[1] = CNSTRESP_QUERY_NODES;
        p[2] = num_entries;
        p[3] = index;
        p[4] = flags;
        node_id_to_data(slave, &p[5]);
        return p;
    }
};

}  // namespace openlcb

#endif // _OPENLCB_TRACTIONDEFS_HXX_
