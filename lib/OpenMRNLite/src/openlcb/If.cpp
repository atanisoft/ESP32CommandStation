/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file If.cxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#include "openlcb/If.hxx"

/// Ensures that the largest bucket in the main buffer pool is exactly the size
/// of a GenMessage.
const unsigned LARGEST_BUFFERPOOL_BUCKET = sizeof(Buffer<openlcb::GenMessage>);

namespace openlcb
{

string node_id_to_buffer(NodeID id)
{
    id = htobe64(id);
    const char *src = reinterpret_cast<const char *>(&id);
    return string(src + 2, 6);
}

void node_id_to_data(NodeID id, void* buf)
{
    id = htobe64(id);
    const char *src = reinterpret_cast<const char *>(&id);
    memcpy(buf, src + 2, 6);
}

NodeID data_to_node_id(const void* buf)
{
    uint64_t d = 0;
    memcpy(reinterpret_cast<uint8_t *>(&d) + 2, buf, 6);
    return be64toh(d);
}

NodeID buffer_to_node_id(const string &buf)
{
    HASSERT(buf.size() == 6);
    return data_to_node_id(buf.data());
}

Payload eventid_to_buffer(uint64_t eventid)
{
    eventid = htobe64(eventid);
    return string(reinterpret_cast<char*>(&eventid), 8);
}

void error_to_data(uint16_t error_code, void* data) {
    uint8_t* p = (uint8_t*) data;
    p[0] = error_code >> 8;
    p[1] = error_code & 0xff;
}

uint16_t data_to_error(const void *data)
{
    const uint8_t *p = (const uint8_t *)data;
    return (((uint16_t)p[0]) << 8) | p[1];
}

string error_to_buffer(uint16_t error_code, uint16_t mti)
{
    string ret(4, '\0');
    error_to_data(error_code, &ret[0]);
    ret[2] = mti >> 8;
    ret[3] = mti & 0xff;
    return ret;
}

string error_to_buffer(uint16_t error_code)
{
    string ret(2, '\0');
    error_to_data(error_code, &ret[0]);
    return ret;
}

void append_error_to_buffer(uint16_t error_code, Payload* p) {
    p->push_back(error_code >> 8);
    p->push_back(error_code & 0xff);
}


void buffer_to_error(const Payload &payload, uint16_t *error_code,
    uint16_t *mti, string *error_message)
{
    if (mti)
        *mti = 0;
    if (error_code)
        *error_code = Defs::ERROR_PERMANENT;
    if (error_message)
        error_message->clear();
    if (payload.size() >= 2 && error_code)
    {
        *error_code = (((uint16_t)payload[0]) << 8) | payload[1];
    }
    if (payload.size() >= 4 && mti)
    {
        *mti = (((uint16_t)payload[2]) << 8) | payload[3];
    }
    if (payload.size() > 4 && error_message)
    {
        error_message->assign(&payload[4], payload.size() - 4);
    }
}

void send_event(Node* src_node, uint64_t event_id)
{
    auto *b = src_node->iface()->global_message_write_flow()->alloc();
    b->data()->reset(Defs::MTI_EVENT_REPORT, src_node->node_id(),
                     eventid_to_buffer(event_id));
    src_node->iface()->global_message_write_flow()->send(b);
}


string EMPTY_PAYLOAD;

/*Buffer *node_id_to_buffer(NodeID id)
{
    Buffer *ret = buffer_alloc(6);
    id = htobe64(id);
    uint8_t *src = reinterpret_cast<uint8_t *>(&id);
    memcpy(ret->start(), src + 2, 6);
    ret->advance(6);
    return ret;
}

NodeID buffer_to_node_id(Buffer *buf)
{
    HASSERT(buf);
    HASSERT(buf->used() == 6);
    uint64_t d = 0;
    memcpy(reinterpret_cast<uint8_t *>(&d) + 2, buf->start(), 6);
    return be64toh(d);
    }*/

/// @TODO(balazs.racz): make the map size parametrizable.
If::If(ExecutorBase *executor, int local_nodes_count)
    : Service(executor)
    , globalWriteFlow_(nullptr)
    , addressedWriteFlow_(nullptr)
    , dispatcher_(this)
    , localNodes_(local_nodes_count)
{
}

} // namespace openlcb
