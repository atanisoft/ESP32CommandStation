/**********************************************************************
DCC COMMAND STATION FOR ESP32

COPYRIGHT (c) 2019 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#include "DCCppESP32.h"

#if LCC_ENABLED

// if both RX and TX pins are defined as valid pins enable the CAN interface
#if LCC_CAN_RX_PIN != NOT_A_PIN && LCC_CAN_TX_PIN != NOT_A_PIN
#define LCC_CAN_ENABLED true
#else
#define LCC_CAN_ENABLED false
#endif

#include <ESPmDNS.h>

#include <openlcb/TcpDefs.hxx>
#include <openlcb/DccAccyConsumer.hxx>
#include <openlcb/DccAccyProducer.hxx>
#include <openlcb/CallbackEventHandler.hxx>
#include <dcc/PacketFlowInterface.hxx>
#include <openlcb/ConfiguredTcpConnection.hxx>

#include "LCCCDI.h"

using dcc::PacketFlowInterface;
using openlcb::DccAccyConsumer;
using openlcb::Defs;
using openlcb::EventRegistry;
using openlcb::EventRegistryEntry;
using openlcb::EventReport;
using openlcb::Node;
using openlcb::NodeID;
using openlcb::ConfigDef;
using openlcb::CallbackEventHandler;
using openlcb::SimpleEventHandler;
using openlcb::WriteHelper;

static constexpr NodeID COMMAND_STATION_NODE_ID = UINT64_C(LCC_NODE_ID);

OpenMRN openmrn(COMMAND_STATION_NODE_ID);
// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// ConfigDef comes from LCCCDI.h and is specific to this particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr ConfigDef cfg(0);

// WiFi manager instance, this will cover the LCC uplink etc.
Esp32WiFiManager wifi_mgr(openmrn.stack(), cfg.seg().wifi());

// when the command station starts up the first time the config is blank
// and needs to be reset to factory settings. This class being declared here
// takes care of that.
class FactoryResetHelper : public DefaultConfigUpdateListener {
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE {
        AutoNotify n(done);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        LOG(INFO, "Factory Reset Helper invoked");
        cfg.userinfo().name().write(fd, "DCC++ESP32 Command Station");
        cfg.userinfo().description().write(fd, "");
    }
} factory_reset_helper;

class SimpleEventCallbackHandler : public CallbackEventHandler {
public:
    SimpleEventCallbackHandler(uint64_t eventID, uint32_t callbackType,
        Node *node, CallbackEventHandler::EventReportHandlerFn report_handler,
        CallbackEventHandler::EventStateHandlerFn state_handler) :
        CallbackEventHandler(node, report_handler, state_handler) {
            add_entry(eventID, callbackType);
        }
};

SimpleEventCallbackHandler emergencyPowerOffHandler(Defs::EMERGENCY_OFF_EVENT,
    CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
    openmrn.stack()->node(),
    [](const EventRegistryEntry &registry_entry, EventReport *report, BarrierNotifiable *done) {
        stopDCCSignalGenerators();
        // shutdown all track power outputs
        MotorBoardManager::powerOffAll();
    }, nullptr);

SimpleEventCallbackHandler emergencyPowerOffClearHandler(Defs::CLEAR_EMERGENCY_OFF_EVENT,
    CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
    openmrn.stack()->node(),
    [](const EventRegistryEntry &registry_entry, EventReport *report, BarrierNotifiable *done) {
        startDCCSignalGenerators();
        // Note this will not power on the PROG track as that is only managed via the programming interface
        MotorBoardManager::powerOnAll();
    }, nullptr);

SimpleEventCallbackHandler emergencyStopHandler(Defs::EMERGENCY_STOP_EVENT,
    CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
    openmrn.stack()->node(),
    [](const EventRegistryEntry &registry_entry, EventReport *report, BarrierNotifiable *done) {
        LocomotiveManager::emergencyStop();
    }, nullptr);

class DccPacketQueueInjector : public PacketFlowInterface {
    public:
        void send(Buffer<dcc::Packet> *b, unsigned prio)
        {
            dcc::Packet *pkt = b->data();
            dccSignal[DCC_SIGNAL_OPERATIONS]->loadBytePacket(pkt->payload, pkt->dlc, pkt->packet_header.rept_count);
            // check if the packet looks like an accessories decoder packet
            if(!pkt->packet_header.is_marklin && pkt->dlc == 2 && pkt->payload[0] & 0x80 && pkt->payload[1] & 0x80) {
                // the second byte of the payload contains part of the address and is stored in ones complement format
                uint8_t onesComplementByteTwo = (pkt->payload[1] ^ 0xF8);
                // decode the accessories decoder address and update the TurnoutManager metadata
                uint16_t boardAddress = (pkt->payload[0] & 0x3F) + ((onesComplementByteTwo >> 4) & 0x07);
                uint8_t boardIndex = ((onesComplementByteTwo >> 1) % 4);
                bool state = onesComplementByteTwo & 0x01;
                // with the board address and index decoded from the packet we can assemble a 12bit decoder address
                uint16_t decoderAddress = (boardAddress * 4 + boardIndex) - 3;
                auto turnout = TurnoutManager::getTurnoutByAddress(decoderAddress);
                if(turnout) {
                    turnout->set(state, false);
                }
            }
            b->unref();
        }
};
DccPacketQueueInjector dccPacketInjector;

DccAccyConsumer dccAccessoryConsumer{openmrn.stack()->node(), &dccPacketInjector};

#if LCC_USE_SPIFFS
#define CDI_CONFIG_PREFIX "/spiffs/"
#define LCC_FS SPIFFS
#elif LCC_USE_SD
#define CDI_CONFIG_PREFIX "/sdcard/"
#define LCC_FS SD_MMC
#endif

namespace openlcb
{
    // Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = CDI_CONFIG_PREFIX LCC_CDI_FILE;

    // This will stop openlcb from exporting the CDI memory space upon start.
    const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    const char *const CONFIG_FILENAME = CDI_CONFIG_PREFIX LCC_CONFIG_FILE;

    // The size of the memory space to export over the above device.
    const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

LCCInterface lccInterface;

LCCInterface::LCCInterface() {
}

void LCCInterface::init() {
    LCC_FS.mkdir(LCC_CONFIG_DIR);
    // uncomment the next two lines to force a factory reset on startup
    //LCC_FS.remove(LCC_CDI_FILE);
    //LCC_FS.remove(LCC_CONFIG_FILE);

    // Create the CDI.xml dynamically
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // Start the OpenMRN stack
    openmrn.begin();
    openmrn.start_executor_thread();
#if LCC_CAN_ENABLED
    // Add the hardware CAN device as a bridge
    openmrn.add_can_port(
        new Esp32HardwareCan("esp32can", (gpio_num_t)LCC_CAN_RX_PIN, (gpio_num_t)LCC_CAN_TX_PIN, false));
#endif
}

void LCCInterface::update() {
    // Call into the OpenMRN stack for its periodic updates
    openmrn.loop();
}

#endif

