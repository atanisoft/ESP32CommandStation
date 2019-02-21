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

// if either the RX or TX pin is set to -1 default to no CAN support
#if LCC_CAN_RX_PIN != -1 || LCC_CAN_TX_PIN != -1
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

#if !defined(LCC_ENABLE_GC_TCP_HUB)
#define LCC_ENABLE_GC_TCP_HUB false
#endif

#include "LCCCDI.h"

static constexpr uint64_t COMMAND_STATION_NODE_ID = UINT64_C(LCC_NODE_ID);

#if LCC_ENABLE_GC_TCP_HUB
WiFiServer openMRNServer(openlcb::TcpClientDefaultParams::DEFAULT_PORT);
#endif

OpenMRN openmrn(COMMAND_STATION_NODE_ID);
// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// ConfigDef comes from LCCCDI.h and is specific to this particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

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
        log_i("Factory Reset Helper invoked");
        cfg.userinfo().name().write(fd, "DCC++ESP32 Command Station");
        cfg.userinfo().description().write(fd, "");
    }
} factory_reset_helper;

class SimpleEventCallbackHandler : public openlcb::CallbackEventHandler {
public:
    SimpleEventCallbackHandler(uint64_t eventID, uint32_t callbackType,
        openlcb::Node *node, openlcb::CallbackEventHandler::EventReportHandlerFn report_handler,
        openlcb::CallbackEventHandler::EventStateHandlerFn state_handler) :
        openlcb::CallbackEventHandler(node, report_handler, state_handler) {
            add_entry(eventID, callbackType);
        }
};

SimpleEventCallbackHandler emergencyPowerOffHandler(openlcb::Defs::EMERGENCY_OFF_EVENT,
    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
    openmrn.stack()->node(),
    [](const openlcb::EventRegistryEntry &registry_entry, openlcb::EventReport *report, BarrierNotifiable *done) {
        stopDCCSignalGenerators();
        // shutdown all track power outputs
        MotorBoardManager::powerOffAll();
    }, nullptr);

SimpleEventCallbackHandler emergencyPowerOffClearHandler(openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT,
    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
    openmrn.stack()->node(),
    [](const openlcb::EventRegistryEntry &registry_entry, openlcb::EventReport *report, BarrierNotifiable *done) {
        startDCCSignalGenerators();
        // Note this will not power on the PROG track as that is only managed via the programming interface
        MotorBoardManager::powerOnAll();
    }, nullptr);

SimpleEventCallbackHandler emergencyStopHandler(openlcb::Defs::EMERGENCY_STOP_EVENT,
    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
    openmrn.stack()->node(),
    [](const openlcb::EventRegistryEntry &registry_entry, openlcb::EventReport *report, BarrierNotifiable *done) {
        LocomotiveManager::emergencyStop();
    }, nullptr);

class DccPacketQueueInjector : public dcc::PacketFlowInterface {
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

openlcb::DccAccyConsumer dccAccessoryConsumer{openmrn.stack()->node(), &dccPacketInjector};

namespace openlcb
{
    // Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = "/spiffs/LCC/cdi.xml";

    // This will stop openlcb from exporting the CDI memory space upon start.
    const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    const char *const CONFIG_FILENAME = "/spiffs/LCC/config";

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
    SPIFFS.mkdir("/LCC");
    // uncomment the next two lines to force a factory reset on startup
    //SPIFFS.remove("/LCC/config");
    //SPIFFS.remove("/LCC/cdi.xml");

    // Create the CDI.xml dynamically
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // Start the OpenMRN stack
    openmrn.begin();
#if LCC_CAN_ENABLED
    // Add the hardware CAN device as a bridge
    openmrn.add_can_port(
        new Esp32HardwareCan("esp32can", (gpio_num_t)LCC_CAN_RX_PIN, (gpio_num_t)LCC_CAN_TX_PIN, false));
#endif
}

void LCCInterface::startWiFiDependencies() {
#if LCC_ENABLE_GC_TCP_HUB
    // Advertise the Command Station as a GC TCP hub
    MDNS.addService(openlcb::TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN,
        openlcb::TcpDefs::MDNS_PROTOCOL_TCP,
        openlcb::TcpClientDefaultParams::DEFAULT_PORT);
    // start the TCP/IP listener
    openMRNServer.setNoDelay(true);
    openMRNServer.begin();
#elif !(LCC_CAN_ENABLED)
    // CAN is diabled, search for a hub to connect to
    // TODO: implement this after implementing generic support inside OpenMRN
#endif
}

void LCCInterface::update() {
#if LCC_ENABLE_GC_TCP_HUB
    // if the TCP/IP listener has a new client accept it and add it
    // as a new GridConnect port.
    if (openMRNServer.hasClient())
    {
        WiFiClient client = openMRNServer.available();
        if (client)
        {
            openmrn.add_gridconnect_port(new Esp32WiFiClientAdapter(client));
        }
    }
#endif
    // Call into the OpenMRN stack for its periodic updates
    openmrn.loop();
}

#endif
