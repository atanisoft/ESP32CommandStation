#include "LocoNetESP32UART.h"
#include <esp_task_wdt.h>
#include <soc/uart_struct.h>

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

// declare the full struct as it is not exported from esp32-hal-uart.c
struct uart_struct_t {
    uart_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    xQueueHandle queue;
    intr_handle_t intr_handle;
};

LocoNetESP32Uart::LocoNetESP32Uart(uint8_t rxPin, uint8_t txPin, uint8_t uartNum, bool inverted, bool enablePullup, const BaseType_t preferedCore) :
	LocoNet(), _rxPin(rxPin), _txPin(txPin), _inverted(inverted), _preferedCore(preferedCore), _state(IDLE) {
	DEBUG("Initializing UART(%d) with RX:%d, TX:%d", uartNum, _rxPin, _txPin);
	_uart = uartBegin(uartNum, 16667, SERIAL_8N1, _rxPin, _txPin, 256, _inverted);
	_rxtxTask = nullptr;
	// note: this needs to be done after uartBegin which will set the pin mode to INPUT only.
	if(enablePullup) {
		pinMode(_rxPin, INPUT_PULLUP);
	}
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
	if(xTaskCreatePinnedToCore(LocoNetESP32Uart::taskEntryPoint, "LocoNet RX/TX", LocoNetRXTXThreadStackSize,
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
	// add this thread to the WDT
	esp_task_wdt_add(NULL);

	while(true) {
		esp_task_wdt_reset();
		// process incoming first
		if(uartAvailable(_uart)) {
			DEBUG("RX Begin");
			// start RX to consume available data
			_state = RX;
			while(uartAvailable(_uart)) {
				esp_task_wdt_reset();
				consume(uartRead(_uart));
			}
			DEBUG("RX End");
			// successful RX, switch to CD_BACKOFF
			startCDBackoffTimer();
		} else if(_state == CD_BACKOFF && checkCDBackoffTimer()) {
			DEBUG("Switching to IDLE");
			_state = IDLE;
		} else if(_state == IDLE) {
			LOCONET_TX_LOCK();
			if(_txQueue && uxQueueMessagesWaiting(_txQueue) > 0) {
				DEBUG("TX Begin");
				// last chance check for TX_COLLISION before starting TX
				// st_urx_out contains the status of the UART RX state machine,
				// any value other than zero indicates it is active.
				if(uartRxActive(_uart) ||
					digitalRead(_rxPin) == !_inverted ? LOW : HIGH) {
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
								esp_task_wdt_reset();
								delay(1);
							}
							// check echoed byte for collision
							if(uartRead(_uart) != out) {
								startCollisionTimer();
							}
						}
						esp_task_wdt_reset();
					}
					if(_state == TX) {
						// TX done, switch to CD_BACKOFF
						startCDBackoffTimer();
						DEBUG("TX complete");
					} else {
						// discard TX queue as we had collision
						xQueueReset(_txQueue);
						DEBUG("TX queue reset");
					}
				}
				DEBUG("TX End");
			}
			LOCONET_TX_UNLOCK();
		} else if(_state == TX_COLLISION && checkCollisionTimer()) {
			digitalWrite(_txPin, !_inverted ? LOW : HIGH);
			startCDBackoffTimer();
			DEBUG("TX COLLISION TIMER");
		} else {
			digitalWrite(_txPin, _inverted ? LOW : HIGH);
		}
		esp_task_wdt_reset();
		delay(1);
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
				esp_task_wdt_reset();
				delay(1);
			}
		}
		LOCONET_TX_UNLOCK();
		// wait for TX to complete
		while(_state == IDLE || _state == TX) {
			esp_task_wdt_reset();
			delay(1);
		}
		if(_state == IDLE || _state == CD_BACKOFF) {
			return LN_DONE;
		} else if(_state == TX_COLLISION) {
			return LN_COLLISION;
		}
	}
	return LN_UNKNOWN_ERROR;
}