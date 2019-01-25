/**********************************************************************
DCC++ BASE STATION FOR ESP32

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

#if defined(LCC_ENABLED) && LCC_ENABLED

#include <OpenMRN.h>
#include <openlcb/DccAccyConsumer.hxx>
#include <openlcb/DccAccyProducer.hxx>
#include <dcc/PacketFlowInterface.hxx>

#include "LCCCDI.h"

// This is the TCP/IP port which the ESP32 will listen on for incoming
// GridConnect formatted CAN frames.
constexpr uint16_t OPENMRN_TCP_PORT = 12021L;

// This is the node id to assign to this device, this must be unique
// on the CAN bus.
static constexpr uint64_t NODE_ID = UINT64_C(0x050101013F01);

// This is the TCP/IP listener on the ESP32.
WiFiServer openMRNServer(OPENMRN_TCP_PORT);

// This is the primary entrypoint for the OpenMRN/LCC stack.
OpenMRN openmrn(NODE_ID);

// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// ConfigDef comes from LCCCDI.h and is specific to this particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

class EmergencyShutOffBitInterface : public openlcb::BitEventInterface {
    public:
        EmergencyShutOffBitInterface(openlcb::Node *node) : BitEventInterface(openlcb::Defs::EMERGENCY_OFF_EVENT, openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT), node_(node) {
        }

        openlcb::EventState get_current_state() override {
            if(MotorBoardManager::isTrackPowerOn()) {
                return openlcb::EventState::VALID;
            }
            return openlcb::EventState::INVALID;
        }

        void set_state(bool new_value) override {
            if(new_value) {
                MotorBoardManager::powerOffAll();
            } else {
                MotorBoardManager::powerOnAll();
            }
        }
        openlcb::Node *node() override {
            return node_;
        }
    private:
        openlcb::Node *node_;
        EmergencyShutOffBitInterface();
        DISALLOW_COPY_AND_ASSIGN(EmergencyShutOffBitInterface);
};

EmergencyShutOffBitInterface emergencyShutOffBitInterface(openmrn.stack()->node());
openlcb::BitEventConsumer emergencyPowerOffConsumer(&emergencyShutOffBitInterface);

void loadBytePacket(SignalGenerator &, uint8_t *, uint8_t, uint8_t , bool);
class DccPacketQueueInjector : public dcc::PacketFlowInterface {
    public:
        void send(Buffer<dcc::Packet> *b, unsigned prio)
        {
            dcc::Packet *pkt = b->data();
            loadBytePacket(dccSignal[DCC_SIGNAL_OPERATIONS], pkt->payload, pkt->dlc, pkt->packet_header.rept_count, false);
            // check if the packet looks like an accessories decoder packet
            if(pkt->packet_header.is_marklin == 0 && pkt->dlc == 2 && pkt->payload[0] & 0x80 && pkt->payload[1] & 0x80) {
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

/*openlcb::DccAccyProducer dccAccessoryProducer{openmrn.stack()->node(), [](unsigned address, bool state){

}};*/

namespace openlcb
{
    // Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = "/spiffs/lcc-cdi.xml";

    // This will stop openlcb from exporting the CDI memory space upon start.
    const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    const char *const CONFIG_FILENAME = "/spiffs/lcc_config";

    // The size of the memory space to export over the above device.
    const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

LCCInterface lccInterface;

LCCInterface::LCCInterface()
{
}

void LCCInterface::begin()
{
    // Create the CDI.xml dynamically
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // Start the OpenMRN stack
    openmrn.begin();

    // Start the background task for OpenMRN functionality
    openmrn.start_background_task();

#if defined(LCC_HARDWARE_CAN_ENABLED) && LCC_HARDWARE_CAN_ENABLED
    // Add the hardware CAN device as a bridge
    openmrn.add_can_port(
        new Esp32HardwareCan("esp32can", (gpio_num_t)LCC_CAN_RX_PIN, (gpio_num_t)LCC_CAN_TX_PIN));
#endif
}

void LCCInterface::update()
{
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
}
#endif