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

#include "LocoNet.h"

LocoNetThrottle::LocoNetThrottle(LocoNet &locoNet, uint8_t userData, uint8_t options, uint16_t throttleId) :
  _locoNet(locoNet), _state(TH_ST_FREE), _ticksSinceLastAction(0), _throttleId(throttleId),
  _slot(0xFF), _address(0), _speed(0), _deferredSpeed(0), _status1(0), _dirFunc0to4(0), _func5to8(0),
  _userData(userData), _options(options) {
	_locoNet.onPacket(OPC_SL_RD_DATA, std::bind(&LocoNetThrottle::processMessage, this, std::placeholders::_1));
  _locoNet.onPacket(OPC_LOCO_SPD, std::bind(&LocoNetThrottle::processMessage, this, std::placeholders::_1));
  _locoNet.onPacket(OPC_LOCO_DIRF, std::bind(&LocoNetThrottle::processMessage, this, std::placeholders::_1));
  _locoNet.onPacket(OPC_LOCO_SND, std::bind(&LocoNetThrottle::processMessage, this, std::placeholders::_1));
  _locoNet.onPacket(OPC_SLOT_STAT1, std::bind(&LocoNetThrottle::processMessage, this, std::placeholders::_1));
  _locoNet.onPacket(OPC_LONG_ACK, std::bind(&LocoNetThrottle::processMessage, this, std::placeholders::_1));
}

// LocoNet Throttle Support
// To make it easier to handle the Speed steps 0 = Stop, 1 = Em Stop and 2 -127
// normal speed steps we will swap speed steps 0 and 1 so that the normal
// range for speed steps is Stop = 1 2-127 is normal as before and now 0 = EmStop
static uint8_t SwapSpeedZeroAndEmStop(uint8_t Speed) {
  if(Speed == 0) {
    return 1;
  } else if(Speed == 1) {
    return 0;
  }
  return Speed;
}

void LocoNetThrottle::updateAddress(uint16_t Address, uint8_t ForceNotify) {
  if(ForceNotify || _address != Address) {
    uint16_t oldAddess = _address;
    _address = Address;
    if(addressChangeCallback) {
		  addressChangeCallback(this, _address, oldAddess);
    }
  }
}

void LocoNetThrottle::updateSpeed(uint8_t Speed, uint8_t ForceNotify ) {
  if( ForceNotify || _speed != Speed ) {
    _speed = Speed;
    if(speedChangeCallback) {
      speedChangeCallback(this, SwapSpeedZeroAndEmStop(Speed));
    }
  }
}

void LocoNetThrottle::updateState(TH_STATE State, uint8_t ForceNotify ) {
  if(ForceNotify || _state != State) {
    TH_STATE PrevState = _state;
    _state = State;
    if(throttleStateCallback) {
      throttleStateCallback(this, State, PrevState);
    }
  }
}

void LocoNetThrottle::updateStatus1(uint8_t Status, uint8_t ForceNotify) {
  if( ForceNotify || _status1 != Status ) {
    _status1 = Status;
    if(throttleSlotStateCallback) {
      throttleSlotStateCallback(this, Status);
    }
    updateState(Status & LOCO_IN_USE ? TH_ST_IN_USE : TH_ST_FREE, ForceNotify);
  }
}

void LocoNetThrottle::updateDirectionAndFunctions(uint8_t DirFunc0to4, uint8_t ForceNotify ) {
  if(ForceNotify || _dirFunc0to4 != DirFunc0to4) {
    uint8_t Diffs = _dirFunc0to4 ^ DirFunc0to4;
    _dirFunc0to4 = DirFunc0to4;
    if(functionChangeCallback) {
      // Check Functions 1-4
      for(uint8_t function = 1, Mask = 1; function <= 4; function++, Mask <<= 1) {
        if(ForceNotify || Diffs & Mask) {
          functionChangeCallback(this, function, DirFunc0to4 & Mask);
        }
      }

      // Check Functions 0
      if(ForceNotify || Diffs & DIRF_F0) {
        functionChangeCallback(this, 0, DirFunc0to4 & DIRF_F0);
      }
    }
    // Check Direction
    if(directionChangeCallback && (ForceNotify || Diffs & DIRF_DIR)) {
      directionChangeCallback(this, DirFunc0to4 & DIRF_DIR);
    }
  }
}

void LocoNetThrottle::updateFunctions5to8(uint8_t Func5to8, uint8_t ForceNotify ) {
  if(functionChangeCallback) {
    if( ForceNotify || _func5to8 != Func5to8) {
      uint8_t Diffs = _func5to8 ^ Func5to8;
      _func5to8 = Func5to8;

      // Check Functions 5-8
      for(uint8_t Function = 5, Mask = 1; Function <= 8; Function++, Mask <<= 1) {
        if(ForceNotify || Diffs & Mask) {
          functionChangeCallback(this, Function, Func5to8 & Mask);
        }
      }
    }
  }
}

constexpr uint16_t SLOT_REFRESH_TICKS = 600;   // 600 * 100ms = 60 seconds between speed refresh

void LocoNetThrottle::process100msActions(void) {
  if( _state == TH_ST_IN_USE ) {
	  _ticksSinceLastAction++;

    if( _deferredSpeed || _ticksSinceLastAction > SLOT_REFRESH_TICKS) {
      _locoNet.send(OPC_LOCO_SPD, _slot, _deferredSpeed ? _deferredSpeed : _speed);
      if(_deferredSpeed) {
        _deferredSpeed = 0;
      }
      _ticksSinceLastAction = 0;
    }
  }
}

bool LocoNetThrottle::processMessage(lnMsg *LnPacket) {
  uint8_t  Data2;
  uint16_t  SlotAddress;

  // Update our copy of slot information if applicable
  if( LnPacket->sd.command == OPC_SL_RD_DATA ) {
    SlotAddress = (uint16_t) ((LnPacket->sd.adr2 << 7) + LnPacket->sd.adr);

    if(_slot == LnPacket->sd.slot) {
      // Make sure that the slot address matches even though we have the right slot number
      // as it is possible that another throttle got in before us and took our slot.
      if(_address == SlotAddress) {
        if( _state == TH_ST_SLOT_RESUME && _throttleId != (uint16_t)((LnPacket->sd.id2 << 7) + LnPacket->sd.id1)) {
          updateState(TH_ST_FREE, 1);
          if(throttleErrorCallback) {
		        throttleErrorCallback(this, TH_ER_NO_LOCO);
          }
        }

        updateState(TH_ST_IN_USE, 1);
        updateAddress(SlotAddress, 1);
        updateSpeed(LnPacket->sd.spd, 1);
        updateDirectionAndFunctions(LnPacket->sd.dirf, 1);
        updateFunctions5to8(LnPacket->sd.snd, 1);
        updateStatus1(LnPacket->sd.stat, 1);

        // We need to force a State update to cause a display refresh once all data is known
        updateState(TH_ST_IN_USE, 1);

        // Now Write our own Throttle Id to the slot and write it back to the command station
        LnPacket->sd.command = OPC_WR_SL_DATA;
        LnPacket->sd.id1 = (uint8_t)(_throttleId & 0x7F);
        LnPacket->sd.id2 = (uint8_t)(_throttleId >> 7);
        return true;
      // Ok another throttle did a NULL MOVE with the same slot before we did
      // so we have to try again
      } else if( _state == TH_ST_SLOT_MOVE ) {
        updateState( TH_ST_SELECT, 1 );
        _locoNet.send( OPC_LOCO_ADR, (uint8_t) (_address >> 7), (uint8_t) (_address & 0x7F));
        return true;
      }
    // Slot data is not for one of our slots so check if we have requested a new addres
    } else {
      if(_address == SlotAddress) {
        if( _state == TH_ST_SELECT || _state == TH_ST_DISPATCH) {
          if( ( LnPacket->sd.stat & STAT1_SL_CONUP ) == 0 && ( LnPacket->sd.stat & LOCO_IN_USE ) != LOCO_IN_USE) {
            if(_state == TH_ST_SELECT) {
              updateState(TH_ST_SLOT_MOVE, 1);
              _slot = LnPacket->sd.slot;
              Data2 = LnPacket->sd.slot;
            } else {
              updateState(TH_ST_FREE, 1);
              Data2 = 0;
            }
            _locoNet.send(OPC_MOVE_SLOTS, LnPacket->sd.slot, Data2);
            return true;
          } else {
            if(throttleErrorCallback) {
							throttleErrorCallback(this, TH_ER_SLOT_IN_USE);
            }
            updateState(TH_ST_FREE, 1);
            return true;
          }
        } else if(_state == TH_ST_SLOT_FREE) {
            _locoNet.send(OPC_SLOT_STAT1, LnPacket->sd.slot, _status1 & ~STAT1_SL_BUSY);
            _slot = 0xFF;
            updateState(TH_ST_FREE, 1);
            return true;
        }
      }

      if(_state == TH_ST_ACQUIRE ) {
        _slot = LnPacket->sd.slot;
        updateState(TH_ST_IN_USE, 1);

        updateAddress(SlotAddress, 1);
        updateSpeed(LnPacket->sd.spd, 1);
        updateDirectionAndFunctions(LnPacket->sd.dirf, 1);
        updateStatus1(LnPacket->sd.stat, 1);
        return true;
      }
    }
  }else if( ( ( LnPacket->sd.command >= OPC_LOCO_SPD ) && ( LnPacket->sd.command <= OPC_LOCO_SND ) ) || ( LnPacket->sd.command == OPC_SLOT_STAT1 ) ) {
    if(_slot == LnPacket->ld.slot) {
      if(LnPacket->ld.command == OPC_LOCO_SPD) {
        updateSpeed( LnPacket->ld.data, 0);
      } else if(LnPacket->ld.command == OPC_LOCO_DIRF) {
        updateDirectionAndFunctions( LnPacket->ld.data, 0);
      } else if(LnPacket->ld.command == OPC_LOCO_SND) {
        updateFunctions5to8( LnPacket->ld.data, 0);
      } else if(LnPacket->ld.command == OPC_SLOT_STAT1) {
        updateStatus1( LnPacket->ld.data, 0 );
      }
      return true;
    }
  } else if( LnPacket->lack.command == OPC_LONG_ACK) {
    if(_state >= TH_ST_ACQUIRE && _state <= TH_ST_SLOT_MOVE) {
      if(LnPacket->lack.opcode == (OPC_MOVE_SLOTS & 0x7F) && throttleErrorCallback) {
				throttleErrorCallback(this, TH_ER_NO_LOCO);
      }
      if(LnPacket->lack.opcode == (OPC_LOCO_ADR & 0x7F) && throttleErrorCallback) {
				throttleErrorCallback(this, TH_ER_NO_SLOTS);
      }

      updateState(TH_ST_FREE, 1);
      return true;
    }
  }
  return false;
}

uint16_t LocoNetThrottle::getAddress(void) {
  return _address;
}

TH_ERROR LocoNetThrottle::setAddress(uint16_t Address) {
  if(_state == TH_ST_FREE ) {
    updateAddress( Address, 1 );
    updateState( TH_ST_SELECT, 1 );

    _locoNet.send( OPC_LOCO_ADR, (uint8_t) ( Address >> 7 ), (uint8_t) ( Address & 0x7F ) );
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_BUSY);
  }
  return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottle::resumeAddress(uint16_t Address, uint8_t LastSlot ) {
  if(_state == TH_ST_FREE ) {
    _slot = LastSlot;
    updateAddress(Address, 1);
    updateState(TH_ST_SLOT_RESUME, 1);

    _locoNet.send(OPC_RQ_SL_DATA, LastSlot, 0);
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_BUSY);
  }
  return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottle::freeAddress(uint16_t Address) {
  if(_state == TH_ST_FREE) {
    updateAddress(Address, 1);
    updateState(TH_ST_SLOT_FREE, 1);

    _locoNet.send(OPC_LOCO_ADR, (uint8_t)(Address >> 7), (uint8_t)(Address & 0x7F ));
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_BUSY);
  }
  return TH_ER_BUSY;
}


TH_ERROR LocoNetThrottle::dispatchAddress(uint16_t Address ) {
  if(_state == TH_ST_FREE) {
    updateAddress(Address, 1);
    updateState(TH_ST_DISPATCH, 1);

    _locoNet.send(OPC_LOCO_ADR, (uint8_t)(Address >> 7), (uint8_t)(Address & 0x7F));
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_BUSY);
  }
  return TH_ER_BUSY;
}

TH_ERROR LocoNetThrottle::acquireAddress(void) {
  if(_state == TH_ST_FREE) {
    updateState(TH_ST_ACQUIRE, 1);
    _locoNet.send(OPC_MOVE_SLOTS, 0, 0);
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_BUSY);
  }
  return TH_ER_BUSY;
}

void LocoNetThrottle::releaseAddress(void) {
  if(_state == TH_ST_IN_USE) {
    _locoNet.send(OPC_SLOT_STAT1, _slot, (uint8_t) (_status1 & ~STAT1_SL_BUSY));
  }
  _slot = 0xFF;
  updateState(TH_ST_FREE, 1);
}

uint8_t LocoNetThrottle::getSpeed(void) {
  return SwapSpeedZeroAndEmStop(_speed);
}

TH_ERROR LocoNetThrottle::setSpeed(uint8_t Speed ) {
  if(_state == TH_ST_IN_USE) {
    Speed = SwapSpeedZeroAndEmStop(Speed);
    if(_speed != Speed) {
      // Always defer any speed other than stop or em stop
      if( (_options & TH_OP_DEFERRED_SPEED) &&
        ( ( Speed > 1 ) || !_ticksSinceLastAction ) ) {
        _deferredSpeed = Speed;
      } else {
        _locoNet.send( OPC_LOCO_SPD, _slot, Speed );
        _ticksSinceLastAction = 0;
        _deferredSpeed = 0;
      }
    }
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_NOT_SELECTED);
  }
  return TH_ER_NOT_SELECTED;
}

uint8_t LocoNetThrottle::getDirection(void) {
  return _dirFunc0to4 & (uint8_t)DIRF_DIR;
}

TH_ERROR LocoNetThrottle::setDirection(uint8_t Direction ) {
  if(_state == TH_ST_IN_USE) {
    _locoNet.send(OPC_LOCO_DIRF, _slot, Direction ? (uint8_t)(_dirFunc0to4 | DIRF_DIR) : (uint8_t)(_dirFunc0to4 & ~DIRF_DIR));

    _ticksSinceLastAction = 0;
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_NOT_SELECTED);
  }
  return TH_ER_NOT_SELECTED;
}

uint8_t LocoNetThrottle::getFunction(uint8_t Function ) {
  uint8_t Mask;

  if( Function <= 4 ) {
    Mask = (uint8_t) (1 << ((Function) ? Function - 1 : 4 ));
    return _dirFunc0to4 & Mask;
  }

  Mask = (uint8_t) (1 << (Function - 5));
  return _func5to8 & Mask;
}

TH_ERROR LocoNetThrottle::setFunction(uint8_t Function, uint8_t Value ) {
  uint8_t Mask;
  uint8_t OpCode;
  uint8_t Data;

  if(_state == TH_ST_IN_USE ) {
    if(Function <= 4) {
      OpCode = OPC_LOCO_DIRF;
      Data = _dirFunc0to4;
      Mask = (uint8_t)(1 << ((Function) ? Function - 1 : 4 ));
    } else {
      OpCode = OPC_LOCO_SND;
      Data = _func5to8;
      Mask = (uint8_t)(1 << (Function - 5));
    }

    if( Value ) {
      Data |= Mask;
    } else {
      Data &= (uint8_t)~Mask;
    }

    _locoNet.send( OpCode, _slot, Data );

    _ticksSinceLastAction = 0;
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_NOT_SELECTED);
  }
  return TH_ER_NOT_SELECTED;
}

TH_ERROR LocoNetThrottle::setDirFunc0to4Direct(uint8_t Value ) {
  if(_state == TH_ST_IN_USE) {
    _locoNet.send( OPC_LOCO_DIRF, _slot, Value & 0x7F );
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_NOT_SELECTED);
  }
  return TH_ER_NOT_SELECTED;
}

TH_ERROR LocoNetThrottle::setFunc5to8Direct(uint8_t Value ) {
  if(_state == TH_ST_IN_USE) {
    _locoNet.send( OPC_LOCO_SND, _slot, Value & 0x7F );
    return TH_ER_OK;
  }

  if(throttleErrorCallback) {
		throttleErrorCallback(this, TH_ER_NOT_SELECTED);
  }
  return TH_ER_NOT_SELECTED;
}

TH_STATE LocoNetThrottle::getState(void) {
  return _state;
}


const char *LocoNetThrottle::getStateStr( TH_STATE State ) {
  switch( State ) {
  case TH_ST_FREE:
    return "Free";

  case TH_ST_ACQUIRE:
    return "Acquire";

  case TH_ST_SELECT:
    return "Select";

  case TH_ST_DISPATCH:
    return "Dispatch";

  case TH_ST_SLOT_MOVE:
    return "Slot Move";

  case TH_ST_SLOT_FREE:
    return "Slot Free";

  case TH_ST_IN_USE:
    return "In Use";

  default:
    return "Unknown";
  }
}

const char *LocoNetThrottle::getErrorStr( TH_ERROR Error ) {
  switch( Error ) {
  case TH_ER_OK:
    return "Ok";
  case TH_ER_SLOT_IN_USE:
    return "In Use";
  case TH_ER_BUSY:
    return "Busy";
  case TH_ER_NOT_SELECTED:
    return "Not Sel";
  case TH_ER_NO_LOCO:
    return "No Loco";
  case TH_ER_NO_SLOTS:
    return "No Free Slots";
  default:
    return "Unknown";
  }
}