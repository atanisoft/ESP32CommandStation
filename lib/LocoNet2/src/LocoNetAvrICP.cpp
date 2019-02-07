/****************************************************************************
 * 	Copyright (C) 2015 Alex Shepherd
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

#include "LocoNetAvrICP.h"
#ifndef ESP32

bool LocoNetAvrIcp::begin(uint8_t txPin)
{
}

LN_STATUS LocoNetAvrIcp::sendLocoNetPacketTry(lnMsg *txData, unsigned char ucPrioDelay)
{
	txData = txData;						// Keep the Compilar happy
	ucPrioDelay = ucPrioDelay;

	return LN_DONE;
}

uint8_t LocoNetSystemVariable::readSVStorage(uint16_t Offset ) {
  if( Offset == SV_ADDR_EEPROM_SIZE)
#if (E2END==0x0FF)	/* E2END is defined in processor include */
								return SV_EE_SZ_256;
#elif (E2END==0x1FF)
								return SV_EE_SZ_512;
#elif (E2END==0x3FF)
								return SV_EE_SZ_1024;
#elif (E2END==0x7FF)
								return SV_EE_SZ_2048;
#elif (E2END==0xFFF)
								return SV_EE_SZ_4096;
#else
								return 0xFF;
#endif
  if( Offset == SV_ADDR_SW_VERSION ) {
    return swVersion;
  } else {
    Offset -= 2;    // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
    return eeprom_read_byte((uint8_t*)Offset);
  }
}

uint8_t LocoNetSystemVariable::writeSVStorage(uint16_t Offset, uint8_t Value) {
  Offset -= 2;      // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
  uint8_t oldValue = eeprom_read_byte((uint8_t*)Offset);
  if(oldValue != Value) {
    eeprom_write_byte((uint8_t*)Offset, Value);
    if(_svChangeCallback) {
      _svChangeCallback(Offset+2, Value, oldValue);
    }
  }
  return eeprom_read_byte((uint8_t*)Offset);
}

void LocoNetSystemVariable::reconfigure() {
  wdt_enable(WDTO_15MS);  // prepare for reset
  while (1) {}            // stop and wait for watchdog to knock us out
}
#endif