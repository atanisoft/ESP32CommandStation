/****************************************************************************
 * 	Copyright (C) 2009 to 2013 Alex Shepherd
 * 	Copyright (C) 2013 Damian Philipp
 *
 * 	Portions Copyright (C) Digitrax Inc.
 * 	Portions Copyright (C) Uhlenbrock Elektronik GmbH
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

constexpr uint16_t FC_FRAC_MIN_BASE   				    = 0x3FFF;
constexpr uint8_t FC_FRAC_RESET_HIGH              = 0x78;
constexpr uint8_t FC_FRAC_RESET_LOW               = 0x6D;
constexpr uint8_t FC_TIMER_TICKS                  = 65;    // 65ms ticks
constexpr uint8_t FC_TIMER_TICKS_REQ              = 250;   // 250ms waiting for Response to FC Req

LocoNetFastClock::LocoNetFastClock(LocoNet &locoNet, bool DCS100CompatibleSpeed, bool CorrectDCS100Clock) :
  _locoNet(locoNet), _DCS100CompatibleSpeed(DCS100CompatibleSpeed), _CorrectDCS100Clock(CorrectDCS100Clock), _state(FC_ST_IDLE) {
	  _locoNet.onPacket(OPC_WR_SL_DATA, std::bind(&LocoNetFastClock::processMessage, this, std::placeholders::_1));
    _locoNet.onPacket(OPC_SL_RD_DATA, std::bind(&LocoNetFastClock::processMessage, this, std::placeholders::_1));
    _locoNet.onPacket(FC_SLOT, std::bind(&LocoNetFastClock::processMessage, this, std::placeholders::_1));
}

void LocoNetFastClock::poll() {
  _locoNet.send(OPC_RQ_SL_DATA, FC_SLOT, 0);
}

void LocoNetFastClock::processMessage(lnMsg *packet) {
  if(packet->fc.clk_cntrl & 0x40) {
    if(_state >= FC_ST_REQ_TIME) {
      _data.fc = packet->fc;
      if(_updateCallback) {
        _updateCallback(_data.fc.clk_rate, _data.fc.days,
          (_data.fc.hours_24 >= (128-24)) ? _data.fc.hours_24 - (128-24) : _data.fc.hours_24 % 24,
          _data.fc.mins_60 - (127-60), true);
      }
      if(_fractionalMinCallback) {
        _fractionalMinCallback(FC_FRAC_MIN_BASE - ((_data.fc.frac_minsh << 7) + _data.fc.frac_minsl));
      }
      _state = FC_ST_READY;
    }
  } else {
    _state = FC_ST_DISABLED;
  }
}

void LocoNetFastClock::process66msActions() {
	// If we are all initialised and ready then increment accumulators
  if(_state == FC_ST_READY) {
    _data.fc.frac_minsl += _data.fc.clk_rate;
    if(_data.fc.frac_minsl & 0x80) {
      _data.fc.frac_minsl &= ~0x80;
      _data.fc.frac_minsh++ ;
      if(_data.fc.frac_minsh & 0x80) {
				// For the next cycle prime the fraction of a minute accumulators
        _data.fc.frac_minsl = FC_FRAC_RESET_LOW;

				// If we are in FC_FLAG_DCS100_COMPATIBLE_SPEED mode we need to run faster
				// by reducong the FRAC_MINS duration count by 128
        _data.fc.frac_minsh = FC_FRAC_RESET_HIGH + _DCS100CompatibleSpeed;

        _data.fc.mins_60++;
        if(_data.fc.mins_60 >= 0x7F) {
          _data.fc.mins_60 = 127 - 60 ;
          _data.fc.hours_24++ ;
          if(_data.fc.hours_24 & 0x80) {
            _data.fc.hours_24 = 128 - 24 ;
            _data.fc.days++;
          }
        }
        DEBUG("FastClockUpdate rate: %d, days: %d, time: %d:%d", _data.fc.clk_rate, _data.fc.days,
            (_data.fc.hours_24 >= (128-24)) ? _data.fc.hours_24 - (128-24) : _data.fc.hours_24 % 24,
            _data.fc.mins_60 - (127-60));
        // We either send a message out onto the LocoNet to change the time,
        // which we will also see and act on or just notify our user
        // function that our internal time has changed.
        if(_CorrectDCS100Clock) {
          _data.fc.command = OPC_WR_SL_DATA;
          _locoNet.send(&_data);
        } else if(_updateCallback) {
          _updateCallback(_data.fc.clk_rate, _data.fc.days,
            (_data.fc.hours_24 >= (128-24)) ? _data.fc.hours_24 - (128-24) : _data.fc.hours_24 % 24,
            _data.fc.mins_60 - (127-60), false);
        }
      }
    }
    if(_fractionalMinCallback) {
      _fractionalMinCallback(FC_FRAC_MIN_BASE - ((_data.fc.frac_minsh << 7) + _data.fc.frac_minsl));
    }
  }

  if(_state == FC_ST_IDLE) {
    _locoNet.send(OPC_RQ_SL_DATA, FC_SLOT, 0);
    _state = FC_ST_REQ_TIME;
  }
}