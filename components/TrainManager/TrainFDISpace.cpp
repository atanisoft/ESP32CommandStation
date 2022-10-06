/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */
#include "TrainFDISpace.hxx"

#include "TrainManager.hxx"
#include <FdiXmlGenerator.hxx>
#include <locodb/LocoDatabase.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/Node.hxx>


namespace trainmanager
{
using openlcb::Defs;
using openlcb::MemorySpace;
using openlcb::Node;
using locodb::LocoDatabase;

TrainFDISpace::TrainFDISpace(TrainManager *parent) : parent_(parent)
{
}

bool TrainFDISpace::set_node(Node *node)
{
  if (node_ && node_ == node)
  {
    // same node.
    return true;
  }
  if (parent_->is_valid_train_node(node))
  {
    node_ = node;
    reset_file();
    return true;
  }
  return false;
}

MemorySpace::address_t TrainFDISpace::max_address()
{
  // We don't really know how long this space is; 16 MB is an upper bound.
  return 16 << 20;
}

size_t TrainFDISpace::read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
                            Notifiable *again)
{
  if (source <= gen_.file_offset())
  {
    reset_file();
  }
  ssize_t result = gen_.read(source, dst, len);
  if (result < 0)
  {
    LOG_ERROR("[TrainFDI] Read failure: %u, %zu: %zu (%s)", source, len, result, strerror(errno));
    *error = Defs::ERROR_PERMANENT;
    return 0;
  }
  if (result == 0)
  {
    LOG(CONFIG_TSP_LOGGING_LEVEL, "[TrainFDI] Out-of-bounds read: %u, %zu",
        source, len);
    *error = openlcb::MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
  }
  else
  {
    *error = 0;
  }
  return result;
}

void TrainFDISpace::reset_file()
{
  auto e = Singleton<LocoDatabase>::instance()->get_entry(node_->node_id());
  e->start_read_functions();
  gen_.reset(std::move(e));
}

} // namespace trainmanager