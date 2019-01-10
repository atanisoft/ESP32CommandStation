#include "LocoNet2/LocoNetESP32UART.h"

constexpr UBaseType_t LocoNetRXTXThreadPriority = 2;
constexpr uint32_t LocoNetRXTXThreadStackSize = 1600;

// number of microseconds for one bit
constexpr uint8_t LocoNetTickTime = 60;

// number of microseconds to remain in a collision state
constexpr uint32_t CollisionTimeoutIncrement = 15 * LocoNetTickTime;

// number of microseconds to remain in a CD BACKOFF state
constexpr uint32_t CDBackoffTimeoutIncrement = LocoNetTickTime * LN_CARRIER_TICKS;

#define LOCONET_TX_LOCK()    do {} while (xSemaphoreTake(_txQueuelock, portMAX_DELAY) != pdPASS)
#define LOCONET_TX_UNLOCK()  xSemaphoreGive(_txQueuelock)


LocoNetESP32Uart::LocoNetESP32Uart(uint8_t rxPin, uint8_t txPin, uint8_t uartNum, bool inverted, const BaseType_t preferedCore) :
	LocoNet(), _rxPin(rxPin), _txPin(txPin), _inverted(inverted), _preferedCore(preferedCore), _state(IDLE) {
	_uart = uartBegin(uartNum, 16667, SERIAL_8N1, _rxPin, _txPin, 256, _inverted);
	_rxtxTask = nullptr;
}

bool LocoNetESP32Uart::begin() {
	DEBUG("Creating LocoNet TX Queue");
	_txQueue = xQueueCreate(256, sizeof(uint8_t));
	if(_txQueue == NULL) {
		printf("LocoNet ERROR: Failed to create TX queue!\n");
		return false;
	}
	DEBUG("Creating LocoNet TX Lock");
	_txQueuelock = xSemaphoreCreateMutex();
	if(_txQueuelock == NULL) {
		printf("LocoNet ERROR: Failed to create TX Lock!\n");
		return false;
	}
	DEBUG("Starting LocoNet RX/TX Task");
	if(xTaskCreatePinnedToCore(LocoNetESP32Uart::taskEntryPoint, "LocoNet RX/TX Task", LocoNetRXTXThreadStackSize,
		(void *)this, LocoNetRXTXThreadPriority, &_rxtxTask, _preferedCore) != pdPASS) {
		printf("LocoNet ERROR: Failed to start LocoNet RX/TX task!\n");
		return false;
	}
	return true;
}

void LocoNetESP32Uart::end() {
	if(_rxtxTask) {
		DEBUG("Suspending LocoNet RX/TX Task");
		vTaskSuspend(_rxtxTask);
		DEBUG("Deleting LocoNet RX/TX Task");
		vTaskDelete(_rxtxTask);
	}
	_rxtxTask = nullptr;
	if(_txQueue) {
		DEBUG("Deleting LocoNet TX Queue");
		vQueueDelete(_txQueue);
	}
	_txQueue = nullptr;
	if(_txQueuelock) {
		DEBUG("Deleting LocoNet TX Queue Lock");
		vSemaphoreDelete(_txQueuelock);
	}
	_txQueuelock = nullptr;
}

void LocoNetESP32Uart::startCollisionTimer() {
	DEBUG("LocoNet Collision!!!");
	uartFlush(_uart);
	_state = TX_COLLISION;
	txStats.collisions++;
	_collisionTimeout = (uint64_t)esp_timer_get_time() + CollisionTimeoutIncrement;
}

bool LocoNetESP32Uart::checkCollisionTimer() {
	return _collisionTimeout <= (uint64_t)esp_timer_get_time();
}

void LocoNetESP32Uart::startCDBackoffTimer() {
	_state = CD_BACKOFF;
	_cdBackoffStart = (uint64_t)esp_timer_get_time();
	_cdBackoffTimeout = _cdBackoffStart + CDBackoffTimeoutIncrement;
}

bool LocoNetESP32Uart::checkCDBackoffTimer() {
	return _cdBackoffTimeout <= (uint64_t)esp_timer_get_time();
}

void LocoNetESP32Uart::rxtxTask() {
	while(true) {
		// process incoming first
		if(uartAvailable(_uart)) {
			// start RX to consume available data
			_state = RX;
			while(uartAvailable(_uart)) {
				consume(uartRead(_uart));
			}
			// successful RX, switch to CD_BACKOFF
			startCDBackoffTimer();
		} else if(_state == CD_BACKOFF && checkCDBackoffTimer()) {
			_state = IDLE;
		} else if(_state == IDLE) {
			LOCONET_TX_LOCK();
			if(_txQueue && uxQueueMessagesWaiting(_txQueue) > 0) {
				// last chance check for TX_COLLISION before starting TX
				if(digitalRead(_rxPin) == !_inverted ? LOW : HIGH) {
					startCollisionTimer();
				} else {
					// no collision, start TX
					_state = TX;
					while(uxQueueMessagesWaiting(_txQueue) > 0 && _state == TX) {
						uint8_t out;
						if(xQueueReceive(_txQueue, &out, (portTickType)1)) {
							uartWrite(_uart, out);
							// wait for echo byte before sending next byte
							while(!uartAvailable(_uart)) {
								vPortYield();
							}
							// check echoed byte for collision
							if(uartRead(_uart) != out) {
								startCollisionTimer();
							}
						}
					}
					if(_state == TX) {
						// TX done, switch to CD_BACKOFF
						startCDBackoffTimer();
					} else {
						// discard TX queue as we had collision
						xQueueReset(_txQueue);
					}
				}
			}
			LOCONET_TX_UNLOCK();
		} else if(_state == TX_COLLISION && checkCollisionTimer()) {
			digitalWrite(_txPin, !_inverted ? LOW : HIGH);
			startCDBackoffTimer();
		} else {
			digitalWrite(_txPin, _inverted ? LOW : HIGH);
		}
		vPortYield();
	}
}

LN_STATUS LocoNetESP32Uart::sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay)
{
	if(_txQueue) {
		if (_state == CD_BACKOFF) {
			if(micros() < _cdBackoffStart + (LocoNetTickTime * ucPrioDelay)) {
				_state = IDLE;
			} else if(!checkCDBackoffTimer()) {
				return LN_CD_BACKOFF;
			} else {
				return LN_PRIO_BACKOFF;
			}
		} else if(_state != IDLE) {
			return LN_NETWORK_BUSY;
		}
		LOCONET_TX_LOCK();
		for(uint8_t index = 0; index < packetLen && (_state == IDLE || _state == TX); index++) {
			while(xQueueSendToBack(_txQueue, &packetData[index], (portTickType)5) != pdPASS) {
				vPortYield();
			}
		}
		LOCONET_TX_UNLOCK();
		// wait for TX to complete
		while(_state == IDLE || _state == TX) {
			vPortYield();
		}
		if(_state == IDLE || _state == CD_BACKOFF) {
			return LN_DONE;
		} else if(_state == TX_COLLISION) {
			return LN_COLLISION;
		}
	}
	return LN_UNKNOWN_ERROR;
}