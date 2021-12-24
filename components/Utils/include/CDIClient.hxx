/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2021 Mike Dunston

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

#include <Httpd.h>
#include <openlcb/MemoryConfigClient.hxx>
#include <utils/StringPrintf.hxx>

#ifndef CDI_CLIENT_HXX_
#define CDI_CLIENT_HXX_

struct CDIClientRequest : public CallableFlowRequestBase
{
  enum ReadCmd
  {
    READ
  };
  
  enum WriteCmd
  {
    WRITE
  };

  enum UpdateCompleteCmd
  {
    UPDATE_COMPLETE
  };

  enum RebootCmd
  {
    REBOOT
  };

  void reset(ReadCmd, openlcb::NodeHandle target_node,
             http::WebSocketFlow *socket, uint32_t req_id, size_t offs,
             size_t size, string target, string type, uint8_t space)
  {
    reset_base();
    cmd = CMD_READ;
    this->target_node = target_node;
    this->space_id = space;
    this->socket = socket;
    this->req_id = req_id;
    this->offs = offs;
    this->size = size;
    this->target = target;
    this->type = type;
    value.clear();
  }

  void reset(WriteCmd, openlcb::NodeHandle target_node,
             http::WebSocketFlow *socket, uint32_t req_id, size_t offs,
             size_t size, string target, string value, uint8_t space)
  {
    reset_base();
    cmd = CMD_WRITE;
    this->target_node = target_node;
    this->space_id = space;
    this->socket = socket;
    this->req_id = req_id;
    this->offs = offs;
    this->size = size;
    this->target = target;
    type.clear();
    this->value = std::move(value);
  }

  void reset(UpdateCompleteCmd, openlcb::NodeHandle target_node, http::WebSocketFlow *socket
           , uint32_t req_id)
  {
    reset_base();
    cmd = CMD_UPDATE_COMPLETE;
    this->target_node = target_node;
    this->socket = socket;
    this->req_id = req_id;
    type.clear();
    value.clear();
  }

  void reset(RebootCmd, openlcb::NodeHandle target_node, uint32_t req_id)
  {
    reset_base();
    cmd = CMD_REBOOT;
    this->target_node = target_node;
    this->socket = nullptr;
    this->req_id = req_id;
    type.clear();
    value.clear();
  }

  enum Command : uint8_t
  {
      CMD_READ,
      CMD_WRITE,
      CMD_UPDATE_COMPLETE,
      CMD_REBOOT
  };

  Command cmd;
  http::WebSocketFlow *socket;
  openlcb::NodeHandle target_node;
  uint8_t space_id;
  uint32_t req_id;
  size_t offs;
  size_t size;
  string target;
  string type;
  string value;
};

class CDIClient : public CallableFlow<CDIClientRequest>
{
public:
  CDIClient(Service *service, openlcb::Node *node,
            openlcb::MemoryConfigHandler *memcfg);

private:
  openlcb::MemoryConfigClient client_;

  StateFlowBase::Action entry() override;
  StateFlowBase::Action read_complete();
  StateFlowBase::Action write_complete();
  StateFlowBase::Action update_complete();

};

#endif // CDI_CLIENT_HXX_