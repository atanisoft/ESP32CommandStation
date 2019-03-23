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

#pragma once

#include "LocoNet.h"
#include <esp32-hal-gpio.h>
#include <esp32-hal-uart.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

class LocoNetESP32Uart: public LocoNet {
	public:
		LocoNetESP32Uart(uint8_t rxPin=16, uint8_t txPin=15, uint8_t uartNum=1, bool inverted=false, bool enablePullup=false, const BaseType_t preferedCore=tskNO_AFFINITY);
		virtual bool begin();
		virtual void end();
		static void taskEntryPoint(void *param) {
			static_cast<LocoNetESP32Uart *>(param)->rxtxTask();
		}
	protected:
		LN_STATUS sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay);
	private:
		void startCollisionTimer();
		bool checkCollisionTimer();
		void startCDBackoffTimer();
		bool checkCDBackoffTimer();
		void rxtxTask();
		typedef enum {
			IDLE = 0,		// net is free for anyone to start transmission
			CD_BACKOFF,	// timer interrupt is counting backoff bits
			TX_COLLISION,	// just sending break after creating a collision
			TX,			// transmitting a packet
			RX			// receiving bytes
		} LN_TX_RX_STATUS;
		const uint8_t _rxPin, _txPin;
		const bool _inverted;
		const BaseType_t _preferedCore;
		uart_t *_uart;
		LN_TX_RX_STATUS _state;
		TaskHandle_t _rxtxTask;
		uint64_t _cdBackoffStart;
		uint64_t _cdBackoffTimeout;
		uint64_t _collisionTimeout;
		QueueHandle_t _txQueue;
		SemaphoreHandle_t _txQueuelock;
};
