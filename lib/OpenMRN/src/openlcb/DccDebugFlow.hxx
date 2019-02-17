/** \copyright
 * Copyright (c) 2015, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file DccDebugFlow.hxx
 *
 * Component useful for debugging DCC packet reception/decoding code.
 *
 * @author Balazs Racz
 * @date 13 Dec 2015
 */

#ifndef _OPENLCB_DCCDEBUGFLOW_HXX_
#define _OPENLCB_DCCDEBUGFLOW_HXX_

namespace openlcb {

/// Sends every incoming DCC packet as a custom OpenLCB message to the OpenLCB
/// bus. Useful only for debugging, as it generates a lot of noise on the
/// bus. Also the packets used are totally non-standard.
class DccPacketDebugFlow : public StateFlow<Buffer<dcc::Packet>, QList<1>> {
 public:
  DccPacketDebugFlow(openlcb::Node* node)
      : StateFlow<Buffer<dcc::Packet>, QList<1>>(
            node->iface()->dispatcher()->service()),
        node_(node) {}

 private:
  Action entry() override {
    Debug::DccPacketDelay::set(false);
    return allocate_and_call(node_->iface()->global_message_write_flow(),
                             STATE(msg_allocated));
  }

  Action msg_allocated() {
    auto* b =
        get_allocation_result(node_->iface()->global_message_write_flow());

    b->data()->reset(
        static_cast<openlcb::Defs::MTI>(openlcb::Defs::MTI_XPRESSNET + 1),
        node_->node_id(),
        string((char*)message()->data()->payload, message()->data()->dlc));
    node_->iface()->global_message_write_flow()->send(b);
    return release_and_exit();
  }

  openlcb::Node* node_;
};

/// Global reference to the packet debug flow.
extern DccPacketDebugFlow g_packet_debug_flow;

/// Debug flow for the DCC decoder driver. The flow stores all captured timing
/// information in an inmemory ring buffer, and takes all captured DCC and MM
/// packets and forwards them to the g_packet_debug_flow to be put onto the
/// OpenLCB bus.
class DccDebugDecodeFlow : public dcc::DccDecodeFlow {
 public:
  DccDebugDecodeFlow(Service* service, const char* path) : dcc::DccDecodeFlow(service, path) {}

 private:
  void dcc_packet_finished(const uint8_t* payload, size_t len) override {
    auto* b = g_packet_debug_flow.alloc();
    b->data()->dlc = len;
    memcpy(b->data()->payload, payload, len);
    g_packet_debug_flow.send(b);
  }

  void mm_packet_finished(const uint8_t* payload, size_t len) override {
    auto* b = g_packet_debug_flow.alloc();
    b->data()->dlc = len;
    memcpy(b->data()->payload, payload, len);
    b->data()->payload[0] |= 0xFC;
    g_packet_debug_flow.send(b);
  }

  void debug_data(uint32_t value) override {
    value /= (configCPU_CLOCK_HZ / 1000000);
    log(decoder_.state());
    log(value);
  }

  void log(uint8_t value) {
    dbuffer[ptr] = value;
    ++ptr;
    if (ptr >= sizeof(dbuffer)) ptr = 0;
  }
  uint8_t dbuffer[1024];
  uint16_t ptr = 0;
};


} // namespace

#endif // _OPENLCB_DCCDEBUGFLOW_HXX_
