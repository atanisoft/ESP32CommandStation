/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020-2021 Mike Dunston

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

#include "CSConfigDescriptor.h"

#include <utils/Singleton.hxx>

namespace openlcb
{
  class SimpleStackBase;
  class Node;
  class SimpleInfoFlow;
  class MemoryConfigClient;
  class MemoryConfigHandler;
}

class AutoSyncFileFlow;
class ExecutorBase;
class Service;

namespace esp32cs
{

class LCCStackManager : public Singleton<LCCStackManager>
{
public:
  LCCStackManager(const esp32cs::Esp32ConfigDef &cfg, const uint64_t node_id
                , bool factory_reset);
  openlcb::SimpleStackBase *stack();
  ExecutorBase *executor();
  Service *service();
  openlcb::Node *node();
  openlcb::SimpleInfoFlow *info_flow();
  openlcb::MemoryConfigClient *memory_config_client();
  openlcb::MemoryConfigHandler *memory_config_handler();
  void start(bool is_sd, bool reset_event_ids);
  void shutdown();
  void reboot_node();
  void reset_events();
  void send_event(uint64_t event);
private:
  const Esp32ConfigDef cfg_;
  const uint64_t nodeID_;
  int fd_;
  openlcb::SimpleStackBase *stack_;
  openlcb::MemoryConfigClient *memory_client_;
  AutoSyncFileFlow *configAutoSync_;
};

} // namespace esp32cs