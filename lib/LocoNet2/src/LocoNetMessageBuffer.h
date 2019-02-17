/****************************************************************************
 * 	Copyright (C) 2015 Alex Shepherd
 *
 * 	Portions Copyright (C) Digitrax Inc.
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
 * 	and are used with permission as part of the EmbeddedLocoNet project. That
 * 	permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	EmbeddedLocoNet, please contact Digitrax Inc, for specific permission.
 *
 * 	Note: The sale any LocoNet device hardware (including bare PCB's) that
 * 	uses this or any other LocoNet software, requires testing and certification
 * 	by Digitrax Inc. and will be subject to a licensing agreement.
 *
 * 	Please contact Digitrax Inc. for details.
 *
 *****************************************************************************
 *
 * 	Title :   LocoNet Message Buffer header file
 * 	Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 * 	Date:     13-Feb-2015
 *
 * 	DESCRIPTION
 * 	This module provides functions that manage the buffering and validity checking of
 *	a single LocoNet Message.
 *
 * 	The LocoNet Data Link Layer will receive bytes from the LocoNet and probably store
 *  them in a circular buffer. Then in the programs main loop, those bytes can be read
 *	and passed into this LocoNet Message Buffer class until a complete LocoNet Message
 *  has been formed. Then the CheckSum is checked and if valid a pointer to the complete
 *	lnMsg structure is returned.
 *
 * 	Statistics are maintained for both the receiving of packets.
 *
 * 	Any invalid packets that are received are discarded and the stats are
 * 	updated approproately.
 *
 *****************************************************************************/

#pragma once

#include "ln_opc.h"

#ifndef LN_BUF_SIZE
#define LN_BUF_SIZE (sizeof(lnMsg))
#endif

class LocoNetMessageBuffer {
  private:
	uint8_t     		buffer[LN_BUF_SIZE];
  	uint8_t     		index;
  	uint8_t     		checkSum;
  	uint8_t     		expLen;
	lnMsg 			*getMsg();

  public:
  	LocoNetMessageBuffer();
	lnMsg 			*addByte(uint8_t newByte);

  	LnRxStats  		stats ;
};
