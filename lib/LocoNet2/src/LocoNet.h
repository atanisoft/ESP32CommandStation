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

#pragma once

#include <map>
#include <vector>
#include <functional>

#include "ln_opc.h"
#include "LocoNetMessageBuffer.h"

#ifdef DEBUG_OUTPUT
#if defined(ESP32) && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
#include <esp32-hal-log.h>
#define DEBUG(format, ...) log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#define DEBUG_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#else
#define DEBUG printf
#define DEBUG_ISR printf
#endif
#else
#define DEBUG(format, ...)
#define DEBUG_ISR(format, ...)
#endif

typedef enum
{
    LN_CD_BACKOFF = 0, LN_PRIO_BACKOFF, LN_NETWORK_BUSY, LN_DONE, LN_COLLISION, LN_UNKNOWN_ERROR, LN_RETRY_ERROR
} LN_STATUS;

// CD Backoff starts after the Stop Bit (Bit 9) and has a minimum or 20 Bit Times
// but initially starts with an additional 20 Bit Times
constexpr uint8_t LN_CARRIER_TICKS      = 20; // carrier detect backoff - all devices have to wait this
constexpr uint8_t LN_MASTER_DELAY       = 6;  // non master devices have to wait this additionally
constexpr uint8_t LN_INITIAL_PRIO_DELAY = 20; // initial attempt adds priority delay
constexpr uint8_t LN_BACKOFF_MIN        = (LN_CARRIER_TICKS + LN_MASTER_DELAY);      // not going below this
constexpr uint8_t LN_BACKOFF_INITIAL    = (LN_BACKOFF_MIN + LN_INITIAL_PRIO_DELAY);  // for the first normal tx attempt
constexpr uint8_t LN_BACKOFF_MAX        = (LN_BACKOFF_INITIAL + 10);                 // lower priority is not supported

//
// LNCV error codes
// Used by the LNCV callbacks to signal what kind of error has occurred.
//

// Error-codes for write-requests
#define LNCV_LACK_ERROR_GENERIC (0)
// Unsupported/non-existing CV
#define LNCV_LACK_ERROR_UNSUPPORTED (1)
// CV is read only
#define LNCV_LACK_ERROR_READONLY (2)
// Value out of range
#define LNCV_LACK_ERROR_OUTOFRANGE (3)
// Everything OK
#define LNCV_LACK_OK (127)

// the valid range for module addresses (CV0) as per the LNCV spec.
#define LNCV_MIN_MODULEADDR (0)
#define LNCV_MAX_MODULEADDR (65534)

#define LN_TX_RETRIES_MAX  25

constexpr uint8_t CALLBACK_FOR_ALL_OPCODES=0xFF;

#define LOCONET_PACKET_SIZE(command, size) \
    ((command & 0x60 ) == 0x60 ) ? size : ((command & 0x60 ) >> 4) + 2

class LocoNet {
    public:
        LocoNet();
        virtual bool begin();
        virtual void end();
        LN_STATUS send(lnMsg *TxPacket);
        LN_STATUS send(lnMsg *TxPacket, uint8_t PrioDelay);
        LN_STATUS send(uint8_t OpCode, uint8_t Data1, uint8_t Data2);
        LN_STATUS send(uint8_t OpCode, uint8_t Data1, uint8_t Data2, uint8_t PrioDelay);

        LnRxStats* getRxStats(void);
        LnTxStats* getTxStats(void);

        const char* getStatusStr(LN_STATUS Status);

        LN_STATUS requestSwitch(uint16_t Address, uint8_t Output, uint8_t Direction);
        LN_STATUS reportSwitch(uint16_t Address);
        LN_STATUS reportSensor(uint16_t Address, uint8_t State);
        LN_STATUS reportPower(bool state);

        void onPacket(uint8_t OpCode, std::function<void(lnMsg *)> callback);
        /**
         * Registers a callback for when a sensor changes state
         *                                     address   state
         */
        void onSensorChange(std::function<void(uint16_t, bool)> callback);
        /**
         * Registers a callback for when a switch is requested
         *                                     address   output direction
         */
        void onSwitchRequest(std::function<void(uint16_t, bool, bool)> callback);
        /**
         * Registers a callback for when a switch/sensor status is reported
         *                                     address   state switch/sensor
         */
        void onSwitchReport(std::function<void(uint16_t, bool, bool)> callback);
        /**
         * Registers a callback for when a switch is requested
         *                                     address   output direction
         */
        void onSwitchState(std::function<void(uint16_t, bool, bool)> callback);

        /**
         * Registers a callback for when power status changes
         *                                    on/off
         */
        void onPowerChange(std::function<void(bool)> callback);

        /**
         * Registers a callback for when a MultiSense device reports status
         *                                             id       index   AR/CB  Active
         * AR = Auto-Reversing (true)
         * CB = Circuit Breaker (false)
         */
        void onMultiSenseDeviceInfo(std::function<void(uint8_t, uint8_t, bool, bool)> callback);

        /**
         * Registers a callback for when a MultiSense Transponder event is triggered
         *                                              address    zone    locoaddr  presense
         */
        void onMultiSenseTransponder(std::function<void(uint16_t, uint8_t, uint16_t, bool)> callback);
    protected:
        void consume(uint8_t newByte);

        virtual LN_STATUS sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay) = 0;
        LocoNetMessageBuffer rxBuffer;
        LnTxStats txStats;

    private:
        bool processSwitchSensorMessage(lnMsg *LnPacket);
        std::map<uint8_t, std::vector<std::function<void(lnMsg *)>>> callbacks;
};

#define TH_OP_DEFERRED_SPEED 0x01

typedef enum {
    TH_ST_FREE = 0,
    TH_ST_ACQUIRE,
    TH_ST_SELECT,
    TH_ST_DISPATCH,
    TH_ST_SLOT_MOVE,
    TH_ST_SLOT_FREE,
    TH_ST_SLOT_RESUME,
    TH_ST_IN_USE
} TH_STATE;

typedef enum {
    TH_ER_OK = 0, TH_ER_SLOT_IN_USE, TH_ER_BUSY, TH_ER_NOT_SELECTED, TH_ER_NO_LOCO, TH_ER_NO_SLOTS
} TH_ERROR;

class LocoNetThrottle {
    public:
        LocoNetThrottle(LocoNet &locoNet, uint8_t userData, uint8_t options, uint16_t throttleId);
        bool processMessage(lnMsg *LnPacket);
        void process100msActions(void);

        /**
         * Registers a callback for when the address for this throttle changes
         */
        void onAddressChange(std::function<void(LocoNetThrottle *, uint16_t, uint16_t)> callback) {
            addressChangeCallback = callback;
        }
        /**
         * Registers a callback for when the speed for this throttle changes
         */
        void onSpeedChange(std::function<void(LocoNetThrottle *, uint8_t)> callback) {
            speedChangeCallback = callback;
        }
        /**
         * Registers a callback for when the direction for this throttle changes
         */
        void onDirectionChange(std::function<void(LocoNetThrottle *, uint8_t)> callback) {
            directionChangeCallback = callback;
        }
        /**
         * Registers a callback for when a function for this throttle changes
         */
        void onFunctionChange(std::function<void(LocoNetThrottle *, uint8_t, bool)> callback) {
            functionChangeCallback = callback;
        }
        /**
         * Registers a callback for when this throttle changes slots
         */
        void onSlotStateChange(std::function<void(LocoNetThrottle *, uint8_t)> callback) {
            throttleSlotStateCallback = callback;
        }
        /**
         * Registers a callback for when a this throttle has an error
         */
        void onError(std::function<void(LocoNetThrottle *, TH_ERROR)> callback) {
            throttleErrorCallback = callback;
        }
        /**
         * Registers a callback for when this throttle changes status
         *                                                             Old Status  New Status
         */
        void onThrottleStateChange(std::function<void(LocoNetThrottle *, TH_STATE, TH_STATE)> callback) {
            throttleStateCallback = callback;
        }

        uint16_t getAddress(void);
        TH_ERROR setAddress(uint16_t Address);
        TH_ERROR resumeAddress(uint16_t Address, uint8_t LastSlot);
        TH_ERROR dispatchAddress(uint16_t Address);
        TH_ERROR acquireAddress(void);
        void releaseAddress(void);
        TH_ERROR freeAddress(uint16_t Address);

        uint8_t getSpeed(void);
        TH_ERROR setSpeed(uint8_t Speed);

        uint8_t getDirection(void);
        TH_ERROR setDirection(uint8_t Direction);

        uint8_t getFunction(uint8_t Function);
        TH_ERROR setFunction(uint8_t Function, uint8_t Value);
        TH_ERROR setDirFunc0to4Direct(uint8_t Value);
        TH_ERROR setFunc5to8Direct(uint8_t Value);

        TH_STATE getState(void);
        const char *getStateStr(TH_STATE State);
        const char *getErrorStr(TH_ERROR Error);
    private:
        LocoNet &_locoNet;
        TH_STATE _state;                // State of throttle
        uint16_t _ticksSinceLastAction;
        uint16_t _throttleId;           // Id of throttle
        uint8_t _slot;                  // Master Slot index
        uint16_t _address;              // Decoder Address
        uint8_t _speed;                 // Loco Speed
        uint8_t _deferredSpeed;         // Deferred Loco Speed setting
        uint8_t _status1;               // Stat1
        uint8_t _dirFunc0to4;           // Direction
        uint8_t _func5to8;              // Direction
        uint8_t _userData;
        uint8_t _options;

        void updateAddress(uint16_t Address, uint8_t ForceNotify);
        void updateSpeed(uint8_t Speed, uint8_t ForceNotify);
        void updateState(TH_STATE State, uint8_t ForceNotify);
        void updateStatus1(uint8_t Status, uint8_t ForceNotify);
        void updateDirectionAndFunctions(uint8_t DirFunc0to4, uint8_t ForceNotify);
        void updateFunctions5to8(uint8_t Func5to8, uint8_t ForceNotify);
        std::function<void(LocoNetThrottle *, uint16_t, uint16_t)> addressChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t)> speedChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t, bool)> functionChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t)> directionChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t)> throttleSlotStateCallback;
        std::function<void(LocoNetThrottle *, TH_STATE, TH_STATE)> throttleStateCallback;
        std::function<void(LocoNetThrottle *, TH_ERROR)> throttleErrorCallback;
};

/************************************************************************************
 The LocoNet fast clock in the Command Station is driven from a 65.535 ms
 time base. A normal minute takes approximately 915 x 65.535 ms ticks.

 The LocoNet fast clock values are stored in a special slot in the Command
 Station called the fast clock slot which is slot number 0x7B or 123

 Each of the fields in the slot are supposed to count up until the most significant bit
 is 0x80 and then rollover the appropriate values and reset however this behaviour
 does not seem to hold for all fields and so some corrction factors are needed

 An important part of syncing to the Fast Clock master is to interpret the current
 FRAC_MINS fields so that a Fast Clock Slave can sync to the part minute and then
 rollover it's accumulators in sync with the master. The FRAC_MINS counter is a
 14 bit counter that is stored in the two 7 bit FRAC_MINSL & FRAC_MINSH fields.
 It counts up the FRAC_MINSL field until it rolls over to 0x80 and then increments
 the FRAC_MINSH high field until it rolls over to 0x80 and then increments the minute,
 hour and day fields as appropriate and then resets the FRAC_MINS fields to 0x4000 - 915
 which is stored in each of the 7 bit fields.

 HOWEVER when the DCS100 resets FRAC_MINS fields to 0x4000 - 915, it then immediately
 rolls over a 128 count and so the minute is short by 915 - 128 65.535 ms ticks, so it
 runs too fast. To correct this problem the fast clock slot can be overwritten with
 corrected FRAC_MINS field values that the DCS100 will then increment correctly.

 This implementation of a LocoNet Fast Clock Slave has two features to correct these
 short commings:

 A) It has the option to reduce the FRAC_MINS count by 128 so that it keeps in step with
 the DCS100 Fast Clock which normally runs too fast. This is enabled by passing in the
 FC_FLAG_DCS100_COMPATIBLE_SPEED flag bit to the init() function.

 B) It has the option to overwrite the LocoNet Fast Clock Master slot values with corrected
 FRAC_MINS fields imediately after it rolls-over the fast minute, to make the DCS100 not run
 too fast as it normally does.

 There also seems to be problems with the hours field not rolling over correctly from 23
 back to 0 and so there is extra processing to work out the hours when it has rolled over
 to 0x80 or 0x00 by the time the bit 7 is cleared. This seems to cause the DT400 throttle
 problems as well and so when running in FC_FLAG_MINUTE_ROLLOVER_SYNC mode, this should
 be corrected.

 The DT400 throttle display seems to decode the minutes incorrectly by 1 count and so we
 have to make the same interpretation here which is why there is a 127 and not a 128
 roll-over for the minutes.
 ***********************************************************************************************/

typedef enum {
    FC_ST_IDLE, FC_ST_REQ_TIME, FC_ST_READY, FC_ST_DISABLED,
} FC_STATE;

class LocoNetFastClock {
    public:
        LocoNetFastClock(LocoNet & locoNet, bool DCS100CompatibleSpeed, bool CorrectDCS100Clock);
        void poll(void);
        void process66msActions(void);
        /**
         * Registers a callback for when the fast clock updates
         *                               Rate       Day      Hour   Minute   Sync
         */
        void onUpdate(std::function<void(uint8_t, uint8_t, uint8_t, uint8_t, bool)> callback) {
            _updateCallback = callback;
        }

        /**
         * Registers a callback for the fractional minute updates
         */
        void onFractionalMinUpdate(std::function<void(uint16_t)> callback) {
            _fractionalMinCallback = callback;
        }
    private:
        LocoNet &_locoNet;
        bool _DCS100CompatibleSpeed;
        bool _CorrectDCS100Clock;
        FC_STATE _state;
        lnMsg _data;

        void processMessage(lnMsg *LnPacket);
        std::function<void(uint8_t, uint8_t, uint8_t, uint8_t, bool)> _updateCallback;
        std::function<void(uint16_t)> _fractionalMinCallback;
};

/************************************************************************************
 SV (System Variable Handling
 ************************************************************************************/

typedef enum {
    SV_EE_SZ_256 = 0, SV_EE_SZ_512 = 1, SV_EE_SZ_1024 = 2, SV_EE_SZ_2048 = 3, SV_EE_SZ_4096 = 4, SV_EE_SZ_8192 = 5
} SV_EE_SIZE;

typedef enum {
    SV_WRITE_SINGLE = 0x01,
    SV_READ_SINGLE = 0x02,
    SV_WRITE_MASKED = 0x03,
    SV_WRITE_QUAD = 0x05,
    SV_READ_QUAD = 0x06,
    SV_DISCOVER = 0x07,
    SV_IDENTIFY = 0x08,
    SV_CHANGE_ADDRESS = 0x09,
    SV_RECONFIGURE = 0x0F
} SV_CMD;

typedef enum {
    SV_ADDR_EEPROM_SIZE = 1,
    SV_ADDR_SW_VERSION = 2,
    SV_ADDR_NODE_ID_L = 3,
    SV_ADDR_NODE_ID_H = 4,
    SV_ADDR_SERIAL_NUMBER_L = 5,
    SV_ADDR_SERIAL_NUMBER_H = 6,
    SV_ADDR_USER_BASE = 7,
} SV_ADDR;

typedef enum {
    SV_NOT_CONSUMED = 0, SV_CONSUMED_OK = 1, SV_ERROR = 2, SV_DEFERRED_PROCESSING_NEEDED = 3
} SV_STATUS;

#define SV_MANUFACTURER_DIY		13

class LocoNetSystemVariable {
    public:
        LocoNetSystemVariable(LocoNet &locoNet, uint8_t newMfgId, uint8_t newDevId, uint16_t newProductId,
                uint8_t newSwVersion);

        /**
         * Check whether a message is an SV programming message. If so, the message
         * is processed.
         * Call this message in your main loop to implement SV programming.
         *
         * TODO: This method should be updated to reflect whether the message has
         *	been consumed.
         *
         * Note that this method will not send out replies.
         *
         * Returns:
         *		SV_OK - the message was or was not an SV programming message.
         It may or may not have been consumed.
         *		SV_DEFERRED_PROCESSING_NEEDED - the message was an SV programming
         message and has been consumed. doDeferredProcessing() must be
         called to actually process the message.
         *		SV_ERROR - the message was an SV programming message and carried
         an unsupported OPCODE.
         *
         */
        SV_STATUS processMessage(lnMsg *LnPacket);

        /**
         * Attempts to send a reply to an SV programming message.
         * This method will repeatedly try to send the message, until it succeeds.
         *
         * Returns:
         *		SV_OK - Reply was successfully sent.
         *		SV_DEFERRED_PROCESSING_NEEDED - Reply was not sent, a later retry is needed.
         */
        SV_STATUS doDeferredProcessing(void);

        /**
         * Register SV Change callback
         *                                  SV       value    old value
         */
        void onSVChange(std::function<void(uint16_t, uint8_t, uint8_t)> callback) {
            _svChangeCallback = callback;
        }
        void reconfigureCallback(std::function<void()> callback) {
            _reconfigureCallback = callback;
        }
    private:
        LocoNet &_locoNet;
        uint8_t _mfgId;
        uint8_t _devId;
        uint16_t _productId;
        uint8_t _swVersion;
        bool _deferredProcessingRequired;
        uint8_t _deferredSrcAddr;
        std::function<void(uint16_t, uint8_t, uint8_t)> _svChangeCallback;
        std::function<void()> _reconfigureCallback;


        /** Read a value from the given EEPROM offset.
         *
         * There are two special values for the Offset parameter:
         *	SV_ADDR_EEPROM_SIZE - Return the size of the EEPROM
         *  SV_ADDR_SW_VERSION - Return the value of swVersion
         *  3 and on - Return the byte stored in the EEPROM at location (Offset - 2)
         *
         * Parameters:
         *		Offset: The offset into the EEPROM. Despite the value being passed as 2 Bytes, only the lower byte is respected.
         *
         * Returns:
         *		A Byte containing the EEPROM size, the software version or contents of the EEPROM.
         *
         */
        uint8_t readSVStorage(uint16_t Offset);

        /** Write the given value to the given Offset in EEPROM.
         *
         * TODO: Writes to Offset 0 and 1 will cause data corruption.
         *
         * Fires notifySVChanged(Offset), if the value actually chaned.
         *
         * Returns:
         *		A Byte containing the new EEPROM value (even if unchanged).
         */
        uint8_t writeSVStorage(uint16_t Offset, uint8_t Value);

        /** Checks whether the given Offset is a valid value.
         *
         * Returns:
         *		True - if the given Offset is valid. False Otherwise.
         */
        uint8_t isSVStorageValid(uint16_t Offset);

        /** Read the NodeId (Address) for SV programming of this module.
         *
         * This method accesses multiple special EEPROM locations.
         */
        uint16_t readSVNodeId(void);

        /** Write the NodeId (Address) for SV programming of this module.
         *
         * This method accesses multiple special EEPROM locations.
         */
        uint16_t writeSVNodeId(uint16_t newNodeId);

        /**
         * Checks whether all addresses of an address range are valid (defers to
         * isSVStorageValid()). Sends a notification for the first invalid address
         * (long Ack with a value of 42).
         *
         *	TODO: There is a Type error in this method. Return type is bool, but
         *		actual returned values are Integer.
         *
         * Returns:
         *		0 if at least one address of the range is not valid.
         *		1 if all addresses out of the range are valid.
         */
        bool CheckAddressRange(uint16_t startAddress, uint8_t Count);

        void reconfigure();
};

class LocoNetCV {
    public:
        //Call this method when you want to implement a module that can be configured via Uhlenbrock LNVC messages
        LocoNetCV(LocoNet &locoNet);

        /**
         * Notification that an Discover message was sent. If a module wants to react to this,
         * It should return LNCV_LACK_OK and set ArtNr and ModuleAddress accordingly.
         * A response just as in the case of ProgrammingStart will be generated.
         * If a module responds to a DiscoveryRequest, it should apparently enter programming mode immediately.
         *                                              artNr     Address
         */
        void onDiscoveryRequest(std::function<int8_t(uint16_t &, uint16_t &)> callback) {
            discoveryCallback = callback;
        }
        /**
         * Notification that a ProgrammingStart message was received. Application code should process this message and
         * set the return code to LNCV_LACK_OK in case this message was intended for this module (i.e., the addresses match).
         * In case ArtNr and/or ModuleAddress were Broadcast addresses, the Application Code should replace them by their
         * real values.
         * The calling code will then generate an appropriate ACK message.
         * A return code different than LACK_LNCV_OK will result in no response being sent.
         *                                              artNr     Address
         */
        void onProgrammingStart(std::function<int8_t(uint16_t &, uint16_t &)> callback) {
            progStartCallback = callback;
        }
        /**
         * Notification that an CV Programming Stop message was received.
         * This message is noch ACKed, thus does not require a result to be returned from the application.
         *                                          artNr     Address
         */
        void onProgrammingStop(std::function<int8_t(uint16_t, uint16_t)> callback) {
            progStopCallback = callback;
        }
        /**
         * Notification that a CV read request message was received. Application code should process this message,
         * set the cvValue (last param) to its respective value and set an appropriate return code.
         * return LNCV_LACK_OK leads the calling code to create a response containing lncvValue.
         * return code >= 0 leads to a NACK being sent.
         * return code < 0 will result in no reaction.
         *                                  artNr     CV       Return Value
         */
        void onCVRead(std::function<int8_t(uint16_t, uint16_t, uint16_t &)> callback) {
            cvReadCallback = callback;
        }
        /**
         * Notification that a CV value should be written. Application code should process this message and
         * set an appropriate return code.
         * Note 1: CV 0 is spec'd to be the ModuleAddress.
         * Note 2: Changes to CV 0 must be reflected IMMEDIATELY! E.g. the programmingStop command will
         * be sent using the new address.
         *
         * return codes >= 0 will result in a LACK containing the return code being sent.
         * return codes < 0 will result in no reaction.
         *                                  artNr     Address   CV value
         */
        void onCVWrite(std::function<int8_t(uint16_t, uint16_t, uint16_t)> callback) {
            cvWriteCallback = callback;
        }
    private:
        void makeLNCVresponse(UhlenbrockMsg & ub, uint8_t originalSource, uint16_t first, uint16_t second,
                uint16_t third, uint8_t last);
        // Computes the PXCT byte from the data bytes in the given UhlenbrockMsg.
        void computePXCTFromBytes(UhlenbrockMsg & ub);
        // Computes the correct data bytes using the containes PXCT byte
        void computeBytesFromPXCT(UhlenbrockMsg & ub);
        // Computes an address from a low- and a high-byte
        uint16_t getAddress(uint8_t lower, uint8_t higher);

        LocoNet &_locoNet;
        void processLNCVMessage(lnMsg *LnPacket);

        std::function<int8_t(uint16_t &, uint16_t &)> discoveryCallback;
        std::function<int8_t(uint16_t &, uint16_t &)> progStartCallback;
        std::function<int8_t(uint16_t, uint16_t)> progStopCallback;
        std::function<int8_t(uint16_t, uint16_t, uint16_t &)> cvReadCallback;
        std::function<int8_t(uint16_t, uint16_t, uint16_t)> cvWriteCallback;
};

/************************************************************************************
 Call-back functions
 ************************************************************************************/

// LNCV notify Call-back functions

// Negative return codes will result in no message being sent.
// Where a value response is appropriate, a return value of LNCV_LACK_OK will trigger the
// response being sent.
// Other values greater than 0 will result in a LACK message being sent.
// When no value result is appropriate, LNCV_LACK_OK will be sent as a LACK.

/**
 * TODO: General LNCV documentation
 * Pick an ArtNr
 * Implement your code to the following behaviour...
 */
