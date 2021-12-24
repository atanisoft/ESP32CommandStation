/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

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

#include "CDIClient.hxx"
#include <StringUtils.hxx>
#include <HttpStringUtils.h>
#include <utils/Base64.hxx>
#include "StringUtils.hxx"

using openlcb::DatagramClient;
using openlcb::Defs;
using openlcb::MemoryConfigClient;
using openlcb::MemoryConfigClientRequest;
using openlcb::MemoryConfigHandler;
using openlcb::Node;

CDIClient::CDIClient(Service *service, Node *node, MemoryConfigHandler *memcfg)
    : CallableFlow<CDIClientRequest>(service), client_(node, memcfg)
{
}

StateFlowBase::Action CDIClient::entry()
{
  request()->resultCode = DatagramClient::OPERATION_PENDING;
  switch (request()->cmd)
  {
  case CDIClientRequest::CMD_READ:
    LOG(VERBOSE,
        "[CDI:%s] Requesting %zu bytes from offset %zu",
        esp32cs::node_id_to_string(request()->target_node.id).c_str(),
        request()->size, request()->offs);
    return invoke_subflow_and_wait(&client_, STATE(read_complete),
                                   MemoryConfigClientRequest::READ_PART,
                                   request()->target_node,
                                   request()->space_id,
                                   request()->offs, request()->size);
  case CDIClientRequest::CMD_WRITE:
    LOG(VERBOSE, "[CDI:%s] Writing %zu bytes to offset %zu",
        esp32cs::node_id_to_string(request()->target_node.id).c_str(),
        request()->size, request()->offs);
    return invoke_subflow_and_wait(&client_, STATE(write_complete),
                                   MemoryConfigClientRequest::WRITE,
                                   request()->target_node,
                                   request()->space_id,
                                   request()->offs, request()->value);
  case CDIClientRequest::CMD_UPDATE_COMPLETE:
    LOG(VERBOSE, "[CDI:%s] Sending update-complete",
        esp32cs::node_id_to_string(request()->target_node.id).c_str());
    return invoke_subflow_and_wait(&client_, STATE(update_complete),
                                   MemoryConfigClientRequest::UPDATE_COMPLETE,
                                   request()->target_node);
  case CDIClientRequest::CMD_REBOOT:
    LOG(VERBOSE, "[CDI:%s] Sending request to reboot",
        esp32cs::node_id_to_string(request()->target_node.id).c_str());
    invoke_subflow_and_ignore_result(&client_,
                                     MemoryConfigClientRequest::REBOOT, request()->target_node);
    return return_ok();
  }
  return return_with_error(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
}

StateFlowBase::Action CDIClient::read_complete()
{
  auto b = get_buffer_deleter(full_allocation_result(&client_));
  LOG(VERBOSE, "[CDI:%s] read bytes request returned with code: %d",
      esp32cs::node_id_to_string(request()->target_node.id).c_str(),
      b->data()->resultCode);
  string response;
  if (b->data()->resultCode)
  {
    LOG(VERBOSE, "[CDI:%s] non-zero result code, sending error response.",
        esp32cs::node_id_to_string(request()->target_node.id).c_str());
    response =
        StringPrintf(
            R"!^!({"res":"error","error":"request failed: %d","id":%d})!^!",
            b->data()->resultCode, request()->req_id);
  }
  else
  {
    LOG(VERBOSE, "[CDI:%s] Received %zu bytes from offset %zu",
        esp32cs::node_id_to_string(request()->target_node.id).c_str(),
        request()->size, request()->offs);
    if (request()->type == "str")
    {
      esp32cs::remove_nulls_and_FF(b->data()->payload);
      response =
          StringPrintf(
              R"!^!({"res":"field","tgt":"%s","val":"%s","type":"%s","id":%d})!^!",
              request()->target.c_str(),
              base64_encode(b->data()->payload).c_str(),
              request()->type.c_str(), request()->req_id);
    }
    else if (request()->type == "int")
    {
      uint32_t data = b->data()->payload.data()[0];
      if (request()->size == 2)
      {
        uint16_t data16 = 0;
        memcpy(&data16, b->data()->payload.data(), sizeof(uint16_t));
        data = be16toh(data16);
      }
      else if (request()->size == 4)
      {
        uint32_t data32 = 0;
        memcpy(&data32, b->data()->payload.data(), sizeof(uint32_t));
        data = be32toh(data32);
      }
      response =
          StringPrintf(
              R"!^!({"res":"field","tgt":"%s","val":"%d","type":"%s","id":%d})!^!", request()->target.c_str(), data, request()->type.c_str(), request()->req_id);
    }
    else if (request()->type == "evt")
    {
      uint64_t event_id = 0;
      memcpy(&event_id, b->data()->payload.data(), sizeof(uint64_t));
      response =
          StringPrintf(
              R"!^!({"res":"field","tgt":"%s","val":"%s","type":"%s","id":%d})!^!", request()->target.c_str(), uint64_to_string_hex(be64toh(event_id)).c_str(), request()->type.c_str(), request()->req_id);
    }
  }
  LOG(VERBOSE, "[CDI-READ] %s", response.c_str());
  response += "\n";
  request()->socket->send_text(response);
  return return_with_error(b->data()->resultCode);
}

StateFlowBase::Action CDIClient::write_complete()
{
  auto b = get_buffer_deleter(full_allocation_result(&client_));
  LOG(VERBOSE, "[CDI:%s] write bytes request returned with code: %d",
      esp32cs::node_id_to_string(request()->target_node.id).c_str(),
      b->data()->resultCode);
  string response;
  if (b->data()->resultCode)
  {
    LOG(VERBOSE, "[CDI:%s] non-zero result code, sending error response.",
        esp32cs::node_id_to_string(request()->target_node.id).c_str());
    response =
        StringPrintf(
            R"!^!({"res":"error","error":"request failed: %d","id":%d})!^!", b->data()->resultCode, request()->req_id);
  }
  else
  {
    LOG(VERBOSE, "[CDI:%s] Write request processed successfully.",
        esp32cs::node_id_to_string(request()->target_node.id).c_str());
    response =
        StringPrintf(R"!^!({"res":"saved","tgt":"%s","id":%d})!^!", request()->target.c_str(), request()->req_id);
  }
  LOG(VERBOSE, "[CDI-WRITE] %s", response.c_str());
  response += "\n";
  request()->socket->send_text(response);
  return return_with_error(b->data()->resultCode);
}

StateFlowBase::Action CDIClient::update_complete()
{
  auto b = get_buffer_deleter(full_allocation_result(&client_));
  LOG(VERBOSE, "[CDI:%s] update-complete request returned with code: %d",
      esp32cs::node_id_to_string(request()->target_node.id).c_str(),
      b->data()->resultCode);
  string response;
  if (b->data()->resultCode)
  {
    LOG(VERBOSE, "[CDI:%s] non-zero result code, sending error response.",
        esp32cs::node_id_to_string(request()->target_node.id).c_str());
    response =
        StringPrintf(
            R"!^!({"res":"error","error":"request failed: %d","id":%d})!^!", b->data()->resultCode, request()->req_id);
  }
  else
  {
    LOG(VERBOSE, "[CDI:%s] update-complete request processed successfully.",
        esp32cs::node_id_to_string(request()->target_node.id).c_str());
    response =
        StringPrintf(R"!^!({"res":"update-complete","id":%d})!^!", request()->req_id);
  }
  LOG(VERBOSE, "[CDI-UPDATE-COMPLETE] %s", response.c_str());
  response += "\n";
  request()->socket->send_text(response);
  return return_with_error(b->data()->resultCode);
}
