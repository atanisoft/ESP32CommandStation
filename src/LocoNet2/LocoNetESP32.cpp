#include <Arduino.h>
#include <functional>
#include <algorithm>
#include "LocoNet2/LocoNet.h"
#include "LocoNet2/LocoNetESP32.h"

#include <EEPROM.h>

static LocoNetESP32 *locoNetInstance = nullptr;

void locoNetTimerCallback() {
    locoNetInstance->loconetBitTimer();
}

void locoNetStartBitCallback() {
    locoNetInstance->loconetStartBit();
}

LocoNetESP32::LocoNetESP32(uint8_t rxPin, uint8_t txPin, uint8_t timerId) : LocoNet(), _rxPin(rxPin), _txPin(txPin), _timerId(timerId) {
    // stash away a pointer to this instance for callback functions
    locoNetInstance = this;
}

bool LocoNetESP32::begin() {
    /* Use one of the four ESP32 hardware timers.
     * Set divider for prescaler (see ESP32 Technical Reference Manual for more
     * info) This  should give 10 ticks per bit period for a 16.66kbps link.
     * Assuming clock frequency is 80Mhz, this will be 480    */
    DEBUG("Configuring HW Timer %d as bit timer", _timerId);
    _lnTimer = timerBegin(_timerId, 480, true);
    timerStop(_lnTimer);

    /* Attach onTimer function to our timer. */
    DEBUG("Attaching ISR callback for HW Timer %d", _timerId);
    timerAttachInterrupt(_lnTimer, locoNetTimerCallback, true);

    timerAlarmEnable(_lnTimer);

    /* set up the TX and RX pins */
    DEBUG("Configuring pin %d for RX", _rxPin);
    pinMode(_rxPin, INPUT_PULLUP);
    DEBUG("Configuring pin %d for TX", _txPin);
    pinMode(_txPin, OUTPUT);

    /* attach the startbit interrup to the rx pin */
    DEBUG("Attaching ISR for RX pin %d", _rxPin);
    attachInterrupt(digitalPinToInterrupt(_rxPin), locoNetStartBitCallback, FALLING);
    return true;
}

void LocoNetESP32::end() {
    portENTER_CRITICAL_ISR(&_timerMux);
    // shutdown the timer and associated callbacks
    DEBUG("Stopping HW Timer %d", _timerId);
    timerEnd(_lnTimer);

    // remove ISR if present
    DEBUG("Removing ISR for RX pin %d (if present)", _rxPin);
    detachInterrupt(digitalPinToInterrupt(_rxPin));

    portEXIT_CRITICAL_ISR(&_timerMux);
}

/**************************************************************************
 *
 * Start Bit Interrupt Routine
 *
 * DESCRIPTION
 * This routine is executed when a falling edge on the incoming serial
 * signal is detected. It disables further interrupts and enables
 * timer interrupts (bit-timer) because the UART must now receive the
 * incoming data.
 *
 **************************************************************************/
void IRAM_ATTR LocoNetESP32::loconetStartBit() {
    /* declare a critical section so we can alter the data that is shared
       outside of the iSR */
    portENTER_CRITICAL_ISR(&_timerMux);
    DEBUG_ISR("StartBit");

    DEBUG_ISR("Disabling StartBit ISR");
    // Disable the Start Bit interrupt
    detachInterrupt(digitalPinToInterrupt(_rxPin));

    // make sure we sample the next bit
    timerStop(_lnTimer);
    timerAlarmWrite(_lnTimer, 6, true);
    timerStart(_lnTimer);

    // Set the State to indicate that we have begun to Receive
    _state = LN_ST_RX;

    // Reset the bit counter so that on first increment it is on 0
    _currentBit = 0;
    _lnCurrentRxByte = 0;

    portEXIT_CRITICAL_ISR(&_timerMux);
}

/**
 * LocoNetESP32::loconetBitTimer()
 * This function gets called every bit time
 * i.e. once every 1/16660 seconds.
 */
void IRAM_ATTR LocoNetESP32::loconetBitTimer() {
    portENTER_CRITICAL_ISR(&_timerMux);

    // Make sure the timer is set correctly for the next bit
    timerAlarmWrite(_lnTimer, 10, true);

    _currentBit++;

    if(_state == LN_ST_RX) {
        if(_currentBit < 9) {
            DEBUG_ISR("RX: bit:%d, byte:%02x", _currentBit, _lnCurrentRxByte);
            _lnCurrentRxByte >>= 1;
            if(digitalRead(_rxPin) == LOCONET_RX_HIGH) {
                _lnCurrentRxByte |= 0x80;
            }
        } else {
            DEBUG_ISR("RX: StopBit Phase, byte: %02x", _lnCurrentRxByte);
            // Check that the stop bit has been received, if it is not HIGH
            // then it is a framing error
            if(digitalRead(_rxPin) == LOCONET_RX_LOW) {
                DEBUG_ISR("RX: ERROR");
                rxBuffer.stats.rxErrors++;
            } else {
                // Send of the received byte for processing
                consume(_lnCurrentRxByte);
            }
            _currentBit = 0;
            _state = LN_ST_CD_BACKOFF;
        }
    } else if(_state == LN_ST_TX) {
        // To get to this point we have already begun the TX cycle so we need to
        // first check for a Collision.
        if(digitalRead(_rxPin) == LOCONET_RX_LOW) {
            DEBUG_ISR("Collision!!");
            _currentBit = 0;
            _state = LN_ST_TX_COLLISION;
        } else if(_currentBit < 9) {
            DEBUG_ISR("TX (%d), byte:%02x ", _currentBit, _lnCurrentTxByte);
            // transmit next bit
            if(_lnCurrentTxByte & 0x01) {
                digitalWrite(_txPin, LOCONET_TX_HIGH);
            } else {
                digitalWrite(_txPin, LOCONET_TX_LOW);
            }
            _lnCurrentTxByte >>= 1;
        } else if(_currentBit == 9) {
            DEBUG_ISR("STOP BIT");
            digitalWrite(_txPin, LOCONET_TX_HIGH);
        } else if(!_txBuffer.empty()) {
            DEBUG_ISR("Next Byte");
            _lnCurrentTxByte = _txBuffer.front();
            _txBuffer.pop_front();
            _currentBit = 0;
            DEBUG_ISR("Sending StartBit");
            digitalWrite(_txPin, LOCONET_TX_LOW);
        } else {
            DEBUG_ISR("TX Complete");
            _currentBit = 0;
            _state = LN_ST_CD_BACKOFF;
        }
    }

    if(_state == LN_ST_TX_COLLISION) {
        if(!_currentBit) {
            // Pull the TX Line low to indicate Collision
            digitalWrite(_txPin, LOCONET_TX_LOW);
        } else if(_currentBit >= LN_COLLISION_TICKS) {
            // we have waited for ~15 ticks release the TX line
            digitalWrite(_txPin, LOCONET_TX_HIGH);
            _currentBit = 0;
            _state = LN_ST_CD_BACKOFF;
            txStats.collisions++;
        }
    }

    if(_state == LN_ST_CD_BACKOFF) {
        if(!_currentBit) {
            DEBUG_ISR("Enabling StartBit ISR");
            // Re-enable the start bit detection now that backoff is active
            attachInterrupt(digitalPinToInterrupt(_rxPin), locoNetStartBitCallback, FALLING);
        } else if(_currentBit >= LN_BACKOFF_MAX) {
            DEBUG_ISR("Switching to IDLE state");
            _state = LN_ST_IDLE;
            // disable bit timer
            timerStop(_lnTimer);
        } 
    }
    /* Re-enable interrupts */
    portEXIT_CRITICAL_ISR(&_timerMux);
}

/**
 * LocoNetESP32::sendLocoNetPacketTry
 *
 * Attempts to send a loconet packet.
 *
 * @param txData - a pointer to an lnMsg packet to transmit
 * @param ucPrioDelay - the delay to add to wait for the bus
 *                      to remain clear before transmission
 * @return LN_STATUS - the current status of the Loconet
 *                   transmission.
 */
LN_STATUS LocoNetESP32::sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay) {
    _txBuffer.clear();
    std::copy(packetData, packetData + packetLen, back_inserter(_txBuffer));
    DEBUG("Queued %d bytes for TX", _txBuffer.size());

    // Load the first Byte
    _lnCurrentTxByte = _txBuffer.front();
    _txBuffer.pop_front();

    if(_state == LN_ST_CD_BACKOFF) {
        if(_currentBit >= ucPrioDelay) {
            DEBUG("Switching to IDLE state");
            portENTER_CRITICAL(&_timerMux);
            timerStop(_lnTimer);
            _state = LN_ST_IDLE;
            portEXIT_CRITICAL(&_timerMux);
        }
    }
    // check if we received a start bit before proceeding
    if(_state == LN_ST_CD_BACKOFF) {
        if(_currentBit < LN_CARRIER_TICKS) {
            return LN_CD_BACKOFF;
        } else {
            return LN_PRIO_BACKOFF;
        }
    }

    if(_state != LN_ST_IDLE) {
        // neither idle nor backoff -> busy
        return LN_NETWORK_BUSY;
    }

    // Disable the start bit interrupt, we don't want to think our TX is an RX.
    DEBUG("Disabling StartBit ISR");
    detachInterrupt(digitalPinToInterrupt(_rxPin));
    delay(2);

    DEBUG("Sending StartBit");
    /* Send the start bit */
    digitalWrite(_txPin, LOCONET_TX_LOW);

    portENTER_CRITICAL(&_timerMux);
    _currentBit = 0;
    _state = LN_ST_TX;
    DEBUG("Enabling Bit Timer");
    timerStop(_lnTimer);
    timerAlarmWrite(_lnTimer, 10, true);
    timerStart(_lnTimer);
    portEXIT_CRITICAL(&_timerMux);

    while(_state == LN_ST_TX) {
        // now busy - wait until the interrupts do the rest of the transmitting
        delay(2);
    }
    if(_state == LN_ST_CD_BACKOFF || _state == LN_ST_IDLE) {
        txStats.txPackets++;
        return LN_DONE;
    } else if(_state == LN_ST_TX_COLLISION) {
        return LN_COLLISION;
    }
    return LN_UNKNOWN_ERROR; // everything else is an error
}

uint8_t LocoNetSystemVariable::readSVStorage(uint16_t offset) {
    if(offset == SV_ADDR_EEPROM_SIZE) {
        return 0xFF;
    } else if(offset == SV_ADDR_SW_VERSION) {
        return _swVersion;
    }
    offset -= 2;    // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
    return EEPROM.read(offset);
}

uint8_t LocoNetSystemVariable::writeSVStorage(uint16_t offset, uint8_t value) {
    offset -= 2;      // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
    uint8_t oldValue = EEPROM.read(offset);
    if(oldValue != value) {
        EEPROM.write(offset, value);
        if(_svChangeCallback) {
            _svChangeCallback(offset + 2, value, oldValue);
        }
    }
    return EEPROM.read(offset);
}

void LocoNetSystemVariable::reconfigure() {
    // shutdown the LocoNet subsystems (required for ESP32 restart is a soft restart and ISRs will not be shutdown)
    _locoNet.end();
    if(_reconfigureCallback) {
        _reconfigureCallback();
    }
    // reset the esp32
    esp_restart();
}