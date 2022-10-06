/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include "TrainManager.hxx"
#include "TrainCDISpace.hxx"

#include <openlcb/MemoryConfig.hxx>
#include <openlcb/Node.hxx>
#include <locodb/LocoDatabase.hxx>

namespace trainmanager
{

using locodb::LocoDatabase;

using openlcb::MemorySpace;

TrainCDISpace::TrainCDISpace(TrainManager* parent) : parent_(parent)
{

}

bool TrainCDISpace::set_node(openlcb::Node* node)
{
  if (!parent_->is_valid_train_node(node))
  {
    return false;
  }
  if (!Singleton<LocoDatabase>::instance()->is_valid_train(node->node_id()))
  {
    return false;
  }
  if (Singleton<LocoDatabase>::instance()->get_entry_offset(node->node_id()) < 0)
  {
    proxySpace_ = &parent_->ro_tmp_train_cdi_;
  }
  else
  {
    proxySpace_ = &parent_->ro_train_cdi_;
  }
  return true;
}

openlcb::MemorySpace::address_t TrainCDISpace::max_address()
{
  return proxySpace_->max_address();
}

size_t TrainCDISpace::read(address_t source, uint8_t* dst, size_t len, errorcode_t* error,
            Notifiable* again)
{
  return proxySpace_->read(source, dst, len, error, again);
}

} // namespace trainmanager