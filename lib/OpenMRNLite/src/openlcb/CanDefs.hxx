/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file CanDefs.hxx
 * This file provides an NMRAnet interface specific to CAN.
 *
 * @author Stuart W. Baker
 * @date 18 September 2013
 */

#ifndef _OPENLCB_CANDEFS_HXX_
#define _OPENLCB_CANDEFS_HXX_

#include <cstdint>

#include "openlcb/Defs.hxx"
#include "can_frame.h"
#include "nmranet_config.h"

namespace openlcb
{

/// Static values and helper functions for CAN-bus interfaces.
struct CanDefs {
    /** Status value for an alias pool item.
     */
    enum AliasStatus
    {
        UNDER_TEST, /**< this is an alias we are trying to claim */
        RESERVED,   /**< the alias has been reserved for use */
        CONFLICT,   /**< we discovered someone else already is using this alias */
        FREE        /**< the alias is free for another request */
    };


    /** CAN ID bit fields for most CAN frames. */
    enum ID
    {
        SRC_MASK            = 0x00000fff, /**< mask for source field of CAN ID */
        MTI_MASK            = 0x00fff000, /**< mask for MTI field of CAN ID */
        DST_MASK            = 0x00fff000, /**< mask for MTI field of CAN ID */
        CAN_FRAME_TYPE_MASK = 0x07000000, /**< mask for can frame type field of CAN ID */
        FRAME_TYPE_MASK     = 0x08000000, /**< mask for frame type field of CAN ID */
        PRIORITY_MASK       = 0x10000000, /**< mask for priority field of CAN ID */
        PADDING_MASK        = 0xe0000000, /**< mask for padding field of CAN ID */

        SRC_SHIFT            =  0, /**< shift for source field of CAN ID */
        MTI_SHIFT            = 12, /**< shift for MTI field of CAN ID */
        DST_SHIFT            = 12, /**< shift for MTI field of CAN ID */
        CAN_FRAME_TYPE_SHIFT = 24, /**< shift for can frame type field of CAN ID */
        FRAME_TYPE_SHIFT     = 27, /**< shift for frame type field of CAN ID */
        PRIORITY_SHIFT       = 28, /**< shift for priority field of CAN ID */
        PADDING_SHIFT        = 29, /**< shift for padding field of CAN ID */
        
        CONTROL_SRC_MASK      = 0x00000fff, /**< source alias mask */
        CONTROL_FIELD_MASK    = 0x07fff000, /**< control field data mask */
        CONTROL_SEQUENCE_MASK = 0x07000000, /**< frame sequence number mask */
        CONTROL_TYPE_MASK     = 0x08000000, /**< value of '0' means control frame mask */
        CONTROL_PRIORITY_MASK = 0x10000000, /**< priority mask */
        CONTROL_PADDING_MASK  = 0xe0000000, /**< pad out to a full 32-bit word */

        CONTROL_SRC_SHIFT      =  0, /**< source alias shift */
        CONTROL_FIELD_SHIFT    = 12, /**< control field data shift */
        CONTROL_SEQUENCE_SHIFT = 24, /**< frame sequence number shift */
        CONTROL_TYPE_SHIFT     = 27, /**< value of '0' means control frame shift */
        CONTROL_PRIORITY_SHIFT = 28, /**< priority shift */
        CONTROL_PADDING_SHIFT  = 29  /**< pad out to a full 32-bit word */
    };



    // @TODO(balazs.racz) do we need this?
    typedef uint16_t CanMTI;

    /** CAN Frame Types. */
    enum CanFrameType
    {
        GLOBAL_ADDRESSED      = 1, /**< most CAN frame types fall in this category */
        DATAGRAM_ONE_FRAME    = 2, /**< a single frame datagram */
        DATAGRAM_FIRST_FRAME  = 3, /**< first frame of multi-frame datagram */
        DATAGRAM_MIDDLE_FRAME = 4, /**< middle frame of multi-frame datagram */
        DATAGRAM_FINAL_FRAME  = 5, /**< last frame of multi-frame datagram */
        STREAM_DATA           = 7, /**< stream data frame */
    };
    
    /** Frame Types, Control or normal NMRAnet message. */
    enum FrameType
    {
        CONTROL_MSG = 0, /**< CAN control frame message */
        NMRANET_MSG = 1  /**< normal NMRAnet message */
    };
    
    /** Highest order priority of a CAN message.  Most messages fall into
     * the NORMAL_PRIORITY category.
     */
    enum Priority
    {
        HIGH_PRIORITY   = 0, /**< high priority CAN message */
        NORMAL_PRIORITY = 1  /**< normal priority CAN message */
    };
    
    enum ControlField
    {
        RID_FRAME = 0x0700, /**< Reserve ID Frame */
        AMD_FRAME = 0x0701, /**< Alias Map Definition frame */
        AME_FRAME = 0x0702, /**< Alias Mapping Inquery */
        AMR_FRAME = 0x0703  /**< Alias Map Reset */
    };

    enum AddressedPayloadFlags
    {
        NOT_FIRST_FRAME = 0x20,
        NOT_LAST_FRAME = 0x10,
    };

    /** Get the source field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return source field
     */
    static NodeAlias get_src(uint32_t can_id)
    {
        return (can_id & SRC_MASK) >> SRC_SHIFT;
    }

    /** Get the MTI field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return MTI field value
     */
    static CanMTI get_mti(uint32_t can_id)
    {
        return ((can_id & MTI_MASK) >> MTI_SHIFT);
    }

    /** Get the destination field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return destination field value
     */
    static NodeAlias get_dst(uint32_t can_id)
    {
        return (can_id & DST_MASK) >> DST_SHIFT;
    }

    /** Get the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return CAN frame type field value
     */
    static CanFrameType get_can_frame_type(uint32_t can_id)
    {
        return (CanFrameType)((can_id & CAN_FRAME_TYPE_MASK) >> CAN_FRAME_TYPE_SHIFT);
    }

    /** Get the frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return frame type field value
     */
    static FrameType get_frame_type(uint32_t can_id)
    {
        return (FrameType)((can_id & FRAME_TYPE_MASK) >> FRAME_TYPE_SHIFT);
    }

    /** Get the priority field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return priority field value
     */
    static Priority get_priority(uint32_t can_id)
    {
        return (Priority)((can_id & PRIORITY_MASK) >> PRIORITY_SHIFT);
    }

    /** Tests if the incoming frame is a CID frame.
     * @param can_id identifier to act upon
     * @return true for CID frame, false for any other frame.
     */
    static bool is_cid_frame(uint32_t can_id)
    {
        return ((can_id >> CAN_FRAME_TYPE_SHIFT) & 0x14) == 0x14;
    }


    /** Set the MTI field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param mti MTI field value
     */
    static void set_mti(uint32_t *can_id, CanMTI mti)
    {
        *can_id &= ~MTI_MASK;
        *can_id |= mti << MTI_SHIFT;
    }

    /** Set the source field value of the CAN ID.
     * @param can_id identifier to act upon, passed by pointer
     * @param src source field value
     */
    static void set_src(uint32_t *can_id, NodeAlias src)
    {
        *can_id &= ~SRC_MASK;
        *can_id |= src << SRC_SHIFT;
    }

    /** Set the destination field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param dst destination field value
     */
    static void set_dst(uint32_t *can_id, NodeAlias dst)
    {
        *can_id &= ~DST_MASK;
        *can_id |= dst << DST_SHIFT;
    }

    /** Set the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type CAN frame type field value
     */
    static void set_can_frame_type(uint32_t *can_id, CanFrameType type)
    {
        *can_id &= ~CAN_FRAME_TYPE_MASK;
        *can_id |= type << CAN_FRAME_TYPE_SHIFT;
    }

    /** Set the frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type frame type field value
     */
    static void set_frame_type(uint32_t *can_id, FrameType type)
    {
        *can_id &= ~FRAME_TYPE_MASK;
        *can_id |= type << FRAME_TYPE_SHIFT;
    }

    /** Set the priority field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param priority pryority field value
     */
    static void set_priority(uint32_t *can_id, Priority priority)
    {
        *can_id &= ~PRIORITY_MASK;
        *can_id |= priority << PRIORITY_SHIFT;
    }

    /** Set all the CAN ID fields.
     * @param can_id identifier to act upon
     * @param source source field value
     * @param mti MTI field value
     * @param can_type CAN frame type field value
     * @param type frame type field value
     * @param priority priority field value
     */
    static void set_fields(uint32_t *can_id, NodeAlias src, Defs::MTI mti, CanFrameType can_type, FrameType type, Priority priority)
    {
        *can_id = (src      << SRC_SHIFT           ) +
                  (mti      << MTI_SHIFT           ) +
                  (can_type << CAN_FRAME_TYPE_SHIFT) +
                  (type     << FRAME_TYPE_SHIFT    ) +
                  (priority << PRIORITY_SHIFT      );
    }

    /** Set all the CAN ID fields for datagram or stream message.
     * @param can_id identifier to act upon
     * @param source source alias
     * @param dst desitnation alias
     * @param can_type CAN frame type field value
     */
    static void set_datagram_fields(uint32_t *can_id, NodeAlias src,
                                    NodeAlias dst, CanFrameType can_type)
    {
        *can_id = (src      << SRC_SHIFT           ) +
                  (dst      << DST_SHIFT           ) +
                  (can_type << CAN_FRAME_TYPE_SHIFT) +
                  (NMRANET_MSG << FRAME_TYPE_SHIFT ) +
                  (NORMAL_PRIORITY << PRIORITY_SHIFT);
    }

    /** Get the NMRAnet MTI from a can identifier.
     * @param can_id CAN identifider
     * @return NMRAnet MTI
     */
    static Defs::MTI nmranet_mti(uint32_t can_id);

    /** Get the CAN identifier from an NMRAnet mti and source alias.
     * @param mti NMRAnet MTI
     * @param src Source node alias
     * @return CAN identifier
     */
    static uint32_t can_identifier(Defs::MTI mti, NodeAlias src);

    /** Get the control field of a can control frame. This includes the
     * sequence number and the variable field.

     * @param can_id CAN ID of the control frame
     * @return value of the control field
     */
    static ControlField get_control_field(uint32_t can_id)
    {
        return (ControlField)((can_id & CONTROL_FIELD_MASK) >> CONTROL_FIELD_SHIFT);
    }

#if 0
    /** Get the source field of the a can control frame.
     * @param can_id CAN ID of the control frame
     * @return value of the source field
     */
    static NodeAlias get_control_src(uint32_t can_id)
    {
        return (can_id & CONTROL_SRC_MASK) >> CONTROL_SRC_SHIFT;
    }

    /** Get the sequence field of the a can control frame.
     * @param can_id CAN ID of the control frame
     * @return value of the sequence field
     */
    static unsigned int get_control_sequence(uint32_t can_id)
    {
        return (can_id & CONTROL_SEQUENCE_MASK) >> CONTROL_SEQUENCE_SHIFT;
    }

#endif

    /** Initialize a control frame CAN ID and set DLC to 0.
     * @param src source node alias
     * @param field control field data (e.g. AME_FRAME)
     * @param sequence sequence number or zero if not CID frame
     */
    static uint32_t set_control_fields(
        NodeAlias src, uint16_t field, int sequence)
    {
        return (src << CONTROL_SRC_SHIFT) | (field << CONTROL_FIELD_SHIFT) |
            (sequence << CONTROL_SEQUENCE_SHIFT) | ((0) << CONTROL_TYPE_SHIFT) |
            ((1) << CONTROL_PRIORITY_SHIFT);
    }

    /** Initialize a control frame CAN ID and set DLC to 0.
     * @param _frame control frame to initialize
     * @param _source source data
     * @param _field field data
     * @param _sequence sequence data
     */
    static void control_init(struct can_frame &frame, NodeAlias src, uint16_t field, int sequence)
    {
        SET_CAN_FRAME_ID_EFF(frame, set_control_fields(src, field, sequence));
        frame.can_dlc = 0;
    }

private:
    /** This class should not be instantiated. */
    CanDefs();
};

}  // namespace openlcb

#endif // _OPENLCB_CANDEFS_HXX_
