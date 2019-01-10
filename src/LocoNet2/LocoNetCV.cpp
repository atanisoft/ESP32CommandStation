/****************************************************************************
 * 	Copyright (C) 2009..2013 Alex Shepherd
 *	Copyright (C) 2013 Damian Philipp
 *
 * 	Portions Copyright (C) Digitrax Inc.
 *	Portions Copyright (C) Uhlenbrock Elektronik GmbH
 *
 * 	This library is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU Lesser General Public
 * 	License as published by the Free Software Foundation; either
 * 	version 2.1 of the License, or (at your option) any later version.
 *
 * 	This library is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * 	Lesser General Public License for more details.
 *
 * 	You should have received a copy of the GNU Lesser General Public
 * 	License along with this library; if not, write to the Free Software
 * 	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************
 *
 * 	IMPORTANT:
 *
 * 	Some of the message formats used in this code are Copyright Digitrax, Inc.
 * 	and are used with permission as part of the MRRwA (previously EmbeddedLocoNet) project.
 *  That permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	MRRwA, please contact Digitrax Inc, for specific permission.
 *
 * 	Note: The sale any LocoNet device hardware (including bare PCB's) that
 * 	uses this or any other LocoNet software, requires testing and certification
 * 	by Digitrax Inc. and will be subject to a licensing agreement.
 *
 * 	Please contact Digitrax Inc. for details.
 *
 *****************************************************************************
 *
 * 	IMPORTANT:
 *
 * 	Some of the message formats used in this code are Copyright Uhlenbrock Elektronik GmbH
 * 	and are used with permission as part of the MRRwA (previously EmbeddedLocoNet) project.
 *  That permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	MRRwA, please contact Copyright Uhlenbrock Elektronik GmbH, for specific permission.
 *
 *****************************************************************************
 * 	DESCRIPTION
 * 	This module provides functions that manage the LNCV-specifiv programming protocol
 *
 *****************************************************************************/

#include "LocoNet2/LocoNet.h"
#include <Arduino.h>

// Adresses for the 'SRC' part of an UhlenbrockMsg
#define LNCV_SRC_MASTER 0x00
#define LNCV_SRC_KPU 0x01
// KPU is, e.g., an IntelliBox
// 0x02 has no associated meaning
#define LNCV_SRC_TWINBOX_FRED 0x03
#define LNCV_SRC_IBSWITCH 0x04
#define LNCV_SRC_MODULE 0x05

// Adresses for the 'DSTL'/'DSTH' part of an UhlenbrockMsg
#define LNCV_BROADCAST_DSTL 0x00
#define LNCV_BROADCAST_DSTH 0x00
#define LNCV_INTELLIBOX_SPU_DSTL 'I'
#define LNCV_INTELLIBOX_SPU_DSTH 'B'
#define LNCV_INTELLIBOX_KPU_DSTL 'I'
#define LNCV_INTELLIBOX_KPU_DSTH 'K'
#define LNCV_TWINBOX_DSTH 'T'
// For TwinBox, DSTL can be anything from 0 to 15
#define LNCV_IBSWITCH_KPU_DSTL 'I'
#define LNCV_IBSWITCH_KPU_DSTH 'S'
#define LNCV_MODULE_DSTL 0x05
#define LNCV_MODULE_DSTH 0x00

// Request IDs
#define LNCV_REQID_CFGREAD 31
#define LNCV_REQID_CFGWRITE 32
#define LNCV_REQID_CFGREQUEST 33

// Flags for the 7th data Byte
#define LNCV_FLAG_PRON 0x80
#define LNCV_FLAG_PROFF 0x40
#define LNCV_FLAG_RO 0x01
// other flags are currently unused

LocoNetCV::LocoNetCV(LocoNet &locoNet) : _locoNet(locoNet) {
    _locoNet.onPacket(OPC_IMM_PACKET, std::bind(&LocoNetCV::processLNCVMessage, this, std::placeholders::_1));
    _locoNet.onPacket(OPC_PEER_XFER, std::bind(&LocoNetCV::processLNCVMessage, this, std::placeholders::_1));
}

void LocoNetCV::processLNCVMessage(lnMsg * LnPacket) {
    DEBUG("Possibly a LNCV message.");
    // Either of these message types may be a LNCV message
    // Sanity check: Message length, Verify addresses
    if (LnPacket->ub.mesg_size == 15 && LnPacket->ub.DSTL == LNCV_MODULE_DSTL && LnPacket->ub.DSTH == LNCV_MODULE_DSTH) {
        // It is a LNCV programming message
        computeBytesFromPXCT(LnPacket->ub);
        DEBUG("Message bytes: %d %x %x %x\n",
            LnPacket->ub.ReqId,
            LnPacket->ub.payload.data.deviceClass,
            LnPacket->ub.payload.data.lncvNumber,
            LnPacket->ub.payload.data.lncvValue);
        lnMsg response;

        switch (LnPacket->ub.ReqId) {
        case LNCV_REQID_CFGREQUEST:
            if (LnPacket->ub.payload.data.deviceClass == 0xFFFF && LnPacket->ub.payload.data.lncvNumber == 0x0000 && LnPacket->ub.payload.data.lncvValue == 0xFFFF) {
                // This is a discover message
                DEBUG("LNCV discover: ");
                if (discoveryCallback) {
                    DEBUG(" executing...");
                    if (discoveryCallback(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvValue) == LNCV_LACK_OK) {
                        makeLNCVresponse(response.ub, LnPacket->ub.SRC, LnPacket->ub.payload.data.deviceClass, 0x00, LnPacket->ub.payload.data.lncvValue, 0x00);
                        _locoNet.send(&response);
                    }
                } else {
                    DEBUG(" NOT EXECUTING!");
                }
            } else if (LnPacket->ub.payload.data.flags == 0x00) {
                // This can only be a read message
                DEBUG("LNCV read: ");
                if (cvReadCallback) {
                    DEBUG(" executing...");
                    int8_t returnCode = cvReadCallback(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvNumber, LnPacket->ub.payload.data.lncvValue);
                    if (returnCode == LNCV_LACK_OK) {
                        // return the read value
                        makeLNCVresponse(response.ub, LnPacket->ub.SRC, LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvNumber, LnPacket->ub.payload.data.lncvValue, 0x00); // TODO: D7 was 0x80 here, but spec says that it is unused.
                        _locoNet.send(&response);
                    } else if (returnCode >= 0) {
                        uint8_t old_opcode = (0x7F & LnPacket->ub.command);
                        _locoNet.send(OPC_LONG_ACK, old_opcode, returnCode);
                        // return a nack
                    }
                } else {
                    DEBUG(" NOT EXECUTING!");
                }
            } else {
                // Its a "control" message
                DEBUG("LNCV control: ");
                if ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PRON) != 0x00 && ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PROFF) != 0x00)) {
                    DEBUG("Illegal, ignoring.");
                    // Illegal message, no action.
                } else if ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PRON) != 0x00) {
                    DEBUG("Programming Start, ");
                    // LNCV PROGAMMING START
                    // We'll skip the check whether D[2]/D[3] are 0x0000.
                    if (progStartCallback) {
                        DEBUG(" executing...");
                        if (progStartCallback(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvValue) == LNCV_LACK_OK) {
                            DEBUG("LNCV_LACK_OK %x %x\n", LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvValue);
                            makeLNCVresponse(response.ub, LnPacket->ub.SRC, LnPacket->ub.payload.data.deviceClass, 0x00, LnPacket->ub.payload.data.lncvValue, 0x80);
                            delay(10); // for whatever reason, we need to delay, otherwise the message will not be sent.
                            DEBUG("LoconetPacket: %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
                                response.ub.command,
                                response.ub.mesg_size,
                                response.ub.SRC,
                                response.ub.DSTL,
                                response.ub.DSTH,
                                response.ub.ReqId,
                                response.ub.PXCT1,
                                response.ub.payload.D[0],
                                response.ub.payload.D[1],
                                response.ub.payload.D[2],
                                response.ub.payload.D[3],
                                response.ub.payload.D[4],
                                response.ub.payload.D[5],
                                response.ub.payload.D[6]);
                            DEBUG("Return Code from Send: %x\n", _locoNet.send(&response));
                        } else { // not for us? then no reaction!
                            DEBUG("Ignoring.\n");
                        }
                    } else {
                        DEBUG(" NOT EXECUTING!");
                    }
                }
                if ((LnPacket->ub.payload.data.flags & LNCV_FLAG_PROFF) != 0x00 && progStopCallback) {
                    // LNCV PROGRAMMING END
                    progStopCallback(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvValue);
                }
                // Read-Only mode not implmeneted.
            }

        break;
        case LNCV_REQID_CFGWRITE:
            if (cvWriteCallback) {
                // Negative return code indicates that we are not interested in this message.
                int8_t returnCode = cvWriteCallback(LnPacket->ub.payload.data.deviceClass, LnPacket->ub.payload.data.lncvNumber, LnPacket->ub.payload.data.lncvValue);
                if (returnCode >= 0) {
                    uint8_t old_opcode = (0x7F & LnPacket->ub.command);
                    _locoNet.send(OPC_LONG_ACK, old_opcode, returnCode);
                }
            }
        break;
        }
    }
}

void LocoNetCV::makeLNCVresponse( UhlenbrockMsg & ub, uint8_t originalSource, uint16_t first, uint16_t second, uint16_t third, uint8_t last) {
	ub.command = OPC_PEER_XFER;
	ub.mesg_size = 15;
	ub.SRC = LNCV_SRC_MODULE;
	switch (originalSource) {
		case LNCV_SRC_KPU:
			// Only in case of SRC == 0x01 should something specific be done.
			ub.DSTL = LNCV_INTELLIBOX_KPU_DSTL;
			ub.DSTH = LNCV_INTELLIBOX_KPU_DSTH;
		break;
		default:
			ub.DSTL = originalSource;
			ub.DSTH = 0x00;
	}
	ub.ReqId = LNCV_REQID_CFGREAD;
	ub.PXCT1 = 0x00;
	ub.payload.data.deviceClass = first;
	ub.payload.data.lncvNumber = second;
	ub.payload.data.lncvValue = third;
	ub.payload.data.flags = last;
	computePXCTFromBytes(ub);
}


void LocoNetCV::computeBytesFromPXCT( UhlenbrockMsg & ub) {
	uint8_t mask(0x01);
	// Data has only 7 bytes, so we consider only 7 bits from PXCT1
	for (int i(0); i < 7; ++i) {
		if ((ub.PXCT1 & mask) != 0x00) {
			// Bit was set
			ub.payload.D[i] |= 0x80;
		}
		mask <<= 1;
	}
	ub.PXCT1 = 0x00;
}

void LocoNetCV::computePXCTFromBytes( UhlenbrockMsg & ub ) {
	uint8_t mask(0x01);
	ub.PXCT1 = 0x00;
	// Data has only 7 bytes, so we consider only 7 bits from PXCT1
	for (int i(0); i < 7; ++i) {
		if ((ub.payload.D[i] & 0x80) != 0x00) {
			ub.PXCT1 |= mask; // add bit to PXCT1
			ub.payload.D[i] &= 0x7F;	// remove bit from data byte
		}
		mask <<= 1;
	}
}

uint16_t LocoNetCV::getAddress(uint8_t lower, uint8_t higher) {
	uint16_t result(higher);
	result <<= 8;
	result |= lower;
	return result;
}
