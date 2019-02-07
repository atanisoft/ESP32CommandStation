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
 * 	This module provides functions that manage the sending and receiving of LocoNet packets.
 *
 * 	As bytes are received from the LocoNet, they are stored in a circular
 * 	buffer and after a valid packet has been received it can be read out.
 *
 * 	When packets are sent successfully, they are also appended to the Receive
 * 	circular buffer so they can be handled like they had been received from
 * 	another device.
 *
 * 	Statistics are maintained for both the send and receiving of packets.
 *
 * 	Any invalid packets that are received are discarded and the stats are
 * 	updated approproately.
 *
 *****************************************************************************/

#include <string.h>

#include "LocoNet.h"

const char * LoconetStatusStrings[] = {
	"CD Backoff",
	"Prio Backoff",
	"Network Busy",
	"Done",
	"Collision",
	"Unknown Error",
	"Retry Error"
};

LocoNet::LocoNet() {
}

bool LocoNet::begin() {
  return true;
}

void LocoNet::end() {
}

const char* LocoNet::getStatusStr(LN_STATUS Status) {
  if ((Status >= LN_CD_BACKOFF) && (Status <= LN_RETRY_ERROR)) {
    return LoconetStatusStrings[Status];
  }

  return "Invalid Status";
}

LN_STATUS LocoNet::send(lnMsg *pPacket, uint8_t ucPrioDelay) {
  LN_STATUS enReturn;
  bool ucWaitForEnterBackoff;

  /* clip maximum prio delay */
  if(ucPrioDelay > LN_BACKOFF_MAX) {
      ucPrioDelay = LN_BACKOFF_MAX;
  }

  /* First calculate the checksum as it may not have been done */
  uint8_t *packet = reinterpret_cast<uint8_t *>(pPacket);
  uint8_t packetLen = LOCONET_PACKET_SIZE(pPacket->sz.command, pPacket->sz.mesg_size);
  uint8_t packetChecksum = 0xFF;
  for(uint8_t index = 0; index < packetLen - 1; index++) {
    packetChecksum ^= packet[index];
  }
  packet[packetLen - 1] = packetChecksum;

  DEBUG("LoconetPacket %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
    pPacket->data[0], pPacket->data[1], pPacket->data[2], pPacket->data[3],
    pPacket->data[4], pPacket->data[5], pPacket->data[6], pPacket->data[7],
    pPacket->data[8], pPacket->data[9], pPacket->data[10], pPacket->data[11],
    pPacket->data[12], pPacket->data[13], pPacket->data[14], pPacket->data[15]);

  for (uint8_t ucTry = 0; ucTry < LN_TX_RETRIES_MAX; ucTry++) {
    // wait previous traffic and than prio delay and than try tx
    // don't want to abort do/while loop before we did not see the backoff state once
    ucWaitForEnterBackoff = true;
    do {
      DEBUG("calling sendLocoNetPacketTry(%p, %d, %d)", packet, packetLen, ucPrioDelay);
      enReturn = sendLocoNetPacketTry(packet, packetLen, ucPrioDelay);
      DEBUG("sendLocoNetPacketTry returned %d", enReturn);
      if (enReturn == LN_DONE) { // success?
        return LN_DONE;
      }

      if (enReturn == LN_PRIO_BACKOFF) {
        // now entered backoff -> next state != LN_BACKOFF is worth incrementing the try counter
        ucWaitForEnterBackoff = false;
      }
    } while ((enReturn == LN_CD_BACKOFF) ||                             // waiting CD backoff
             (enReturn == LN_PRIO_BACKOFF) ||                           // waiting master+prio backoff
            ((enReturn == LN_NETWORK_BUSY) && ucWaitForEnterBackoff));  // or within any traffic unfinished
    // failed -> next try going to higher prio = smaller prio delay
    if (ucPrioDelay > LN_BACKOFF_MIN) {
      ucPrioDelay--;
    }
  }
  txStats.txErrors++;
  return LN_RETRY_ERROR;
}

LN_STATUS LocoNet::send(lnMsg *pPacket) {
  return send(pPacket, LN_BACKOFF_INITIAL);
}

LN_STATUS LocoNet::send(uint8_t OpCode, uint8_t Data1, uint8_t Data2) {
  lnMsg SendPacket;

  SendPacket.data[0] = OpCode;
  SendPacket.data[1] = Data1;
  SendPacket.data[2] = Data2;

  return send(&SendPacket);
}

LN_STATUS LocoNet::send(uint8_t OpCode, uint8_t Data1, uint8_t Data2, uint8_t PrioDelay) {
  lnMsg SendPacket;

  SendPacket.data[0] = OpCode;
  SendPacket.data[1] = Data1;
  SendPacket.data[2] = Data2;

  return send(&SendPacket, PrioDelay);
}

LnRxStats* LocoNet::getRxStats() {
  return &rxBuffer.stats;
}

LnTxStats* LocoNet::getTxStats() {
  return &txStats;
}

LN_STATUS LocoNet::reportPower(bool state) {
  lnMsg SendPacket;

  if (state) {
    SendPacket.data[ 0 ] = OPC_GPON;
  } else {
    SendPacket.data[ 0 ] = OPC_GPOFF;
  }

  return send( &SendPacket ) ;
}

void LocoNet::consume(uint8_t newByte) {
	lnMsg * rxPacket = rxBuffer.addByte(newByte);

	if (rxPacket) {
    if(callbacks.find(CALLBACK_FOR_ALL_OPCODES) != callbacks.end()) {
      for(auto cb : callbacks[CALLBACK_FOR_ALL_OPCODES]) {
        cb(rxPacket);
      }
    }
    if(callbacks.find(rxPacket->sz.command) != callbacks.end()) {
      for(auto cb : callbacks[rxPacket->sz.command]) {
        cb(rxPacket);
      }
#if defined(DEBUG_OUTPUT)
    } else {
      DEBUG("No callbacks for OpCode %d", rxPacket->sz.command);
#endif
    }
	}
}

void LocoNet::onSensorChange(std::function<void(uint16_t, bool)> callback) {
  onPacket(OPC_INPUT_REP, [callback](lnMsg *packet) {
    uint16_t address = (packet->srq.sw1 | ((packet->srq.sw2 & 0x0F ) << 7));
    address <<= 1;
    address += (packet->ir.in2 & OPC_INPUT_REP_SW) ? 2 : 1;
    DEBUG("Sensor addr:%d, state:%d", address, packet->ir.in2 & OPC_INPUT_REP_HI);
    callback(address, packet->ir.in2 & OPC_INPUT_REP_HI);
  });
}

void LocoNet::onSwitchRequest(std::function<void(uint16_t, bool, bool)> callback) {
  onPacket(OPC_SW_REQ, [callback](lnMsg *packet) {
    uint16_t address = (packet->srq.sw1 | ((packet->srq.sw2 & 0x0F ) << 7)) + 1;
    DEBUG("Switch Request addr:%d, out:%d, dir:%d", address, packet->srq.sw2 & OPC_SW_REQ_OUT, packet->srq.sw2 & OPC_SW_REQ_DIR);
    callback(address, packet->srq.sw2 & OPC_SW_REQ_OUT, packet->srq.sw2 & OPC_SW_REQ_DIR);
  });
}

void LocoNet::onSwitchReport(std::function<void(uint16_t, bool, bool)> callback) {
  onPacket(OPC_SW_REP, [callback](lnMsg *packet) {
    uint16_t address = (packet->srq.sw1 | ((packet->srq.sw2 & 0x0F ) << 7)) + 1;
    DEBUG("Switch Report addr:%d, out:%d, dir:%d", address, packet->srq.sw2 & OPC_SW_REP_HI, packet->srq.sw2 & OPC_SW_REP_SW);
    callback(address, packet->srq.sw2 & OPC_SW_REP_HI, packet->srq.sw2 & OPC_SW_REP_SW);
  });
}

void LocoNet::onSwitchState(std::function<void(uint16_t, bool, bool)> callback) {
  onPacket(OPC_SW_STATE, [callback](lnMsg *packet) {
    uint16_t address = (packet->srq.sw1 | ((packet->srq.sw2 & 0x0F ) << 7)) + 1;
    DEBUG("Switch State addr:%d, out:%d, dir:%d", address, packet->srq.sw2 & OPC_SW_REQ_OUT, packet->srq.sw2 & OPC_SW_REQ_DIR);
    callback(address, packet->srq.sw2 & OPC_SW_REQ_OUT, packet->srq.sw2 & OPC_SW_REQ_DIR);
  });
}

void LocoNet::onPowerChange(std::function<void(bool)> callback) {
  onPacket(OPC_GPON, [callback](lnMsg *packet) {
    DEBUG("Power ON");
    callback(true);
  });
  onPacket(OPC_GPON, [callback](lnMsg *packet) {
    DEBUG("Power OFF");
    callback(false);
  });
}

void LocoNet::onMultiSenseDeviceInfo(std::function<void(uint8_t, uint8_t, bool, bool)> callback) {
  onPacket(OPC_MULTI_SENSE, [callback](lnMsg *packet) {
    if((packet->data[1] & OPC_MULTI_SENSE_MSG) == OPC_MULTI_SENSE_DEVICE_INFO &&
      ((packet->msdi.arg3 & 0xF0) == 0x30 || (packet->msdi.arg3 & 0xF0) == 0x10)) {
      uint8_t mask = 1;
      for(uint8_t index = 0; index < 4; index++) {
        callback(OPC_MULTI_SENSE_BOARD_ID(packet->msdi.arg1, packet->msdi.arg2),
          index + 1, packet->msdi.arg3 & mask, packet->msdi.arg4 & mask);
        mask <<= 1;
      }
    }
  });
}

void LocoNet::onMultiSenseTransponder(std::function<void(uint16_t, uint8_t, uint16_t, bool)> callback) {
  onPacket(OPC_MULTI_SENSE, [callback](lnMsg *packet) {
    if((packet->data[1] & OPC_MULTI_SENSE_MSG) == OPC_MULTI_SENSE_ABSENT ||
       (packet->data[1] & OPC_MULTI_SENSE_MSG) == OPC_MULTI_SENSE_PRESENT) {
      callback(OPC_MULTI_SENSE_BOARD_ADDRESS(packet->mstr.zone, packet->mstr.type),
        OPC_MULTI_SENSE_ZONE_ID(packet->mstr.zone),
        OPC_MULTI_SENSE_LOCO_ADDRESS(packet->mstr.adr1, packet->mstr.adr2),
        OPC_MULTI_SENSE_PRESENCE(packet->mstr.zone));
    }
  });
}

LN_STATUS LocoNet::requestSwitch(uint16_t Address, uint8_t Output, uint8_t Direction) {
  uint8_t AddrH = (--Address >> 7) & 0x0F;
  uint8_t AddrL = Address & 0x7F;

  if (Output) {
    AddrH |= OPC_SW_REQ_OUT;
  }

  if (Direction) {
    AddrH |= OPC_SW_REQ_DIR;
  }

  return send(OPC_SW_REQ, AddrL, AddrH);
}

LN_STATUS LocoNet::reportSwitch(uint16_t Address) {
  Address -= 1;
  return send(OPC_SW_STATE, (Address & 0x7F), ((Address >> 7) & 0x0F));
}

LN_STATUS LocoNet::reportSensor( uint16_t Address, uint8_t State ) {
	uint8_t AddrH = ((--Address >> 8) & 0x0F) | OPC_INPUT_REP_CB;
	uint8_t AddrL = (Address >> 1) & 0x7F;
	if( Address % 2) {
		AddrH |= OPC_INPUT_REP_SW;
  }

	if(State) {
		AddrH |= OPC_INPUT_REP_HI;
  }

  return send(OPC_INPUT_REP, AddrL, AddrH);
}

void LocoNet::onPacket(uint8_t OpCode, std::function<void(lnMsg *)> callback) {
  callbacks[OpCode].push_back(callback);
  DEBUG("registering callback function for OpCode: %d, %d callbacks for this OpCode.", OpCode, callbacks[OpCode].size());
}
