/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2022 Mike Dunston (atanisoft)
 *
 */

#include "TrainManager.hxx"
#include "TrainSnipHandler.hxx"

#include <locodb/LocoDatabase.hxx>
#include <openlcb/SimpleNodeInfo.hxx>

namespace trainmanager
{

using locodb::LocoDatabase;
using openlcb::Defs;
using openlcb::IncomingMessageStateFlow;
using openlcb::SimpleInfoDescriptor;
using openlcb::SimpleInfoFlow;
using openlcb::SNIP_STATIC_DATA;

TrainSnipHandler::TrainSnipHandler(TrainManager* parent, SimpleInfoFlow* info_flow)
    : IncomingMessageStateFlow(parent->train_service()->iface()),
      parent_(parent),
      responseFlow_(info_flow)
{
  iface()->dispatcher()->register_handler(
      this, Defs::MTI::MTI_IDENT_INFO_REQUEST, Defs::MTI::MTI_EXACT);
}

TrainSnipHandler::~TrainSnipHandler()
{
  iface()->dispatcher()->unregister_handler(
      this, Defs::MTI::MTI_IDENT_INFO_REQUEST, Defs::MTI::MTI_EXACT);
}

StateFlowBase::Action TrainSnipHandler::entry()
{
  // Let's find the train ID.
  if (!parent_->is_valid_train_node(nmsg()->dstNode))
  {
    return release_and_exit();
  }
  return allocate_and_call(responseFlow_, STATE(send_response_request));
}

StateFlowBase::Action TrainSnipHandler::send_response_request()
{
  auto* b = get_allocation_result(responseFlow_);
  auto entry =
    Singleton<LocoDatabase>::instance()->get_entry(nmsg()->dstNode->node_id());
  if (entry)
  {
    snipName_ = entry->get_train_name();
    snipDesc_ = entry->get_train_description();
  }
  else
  {
    snipName_.clear();
    snipDesc_.clear();
  }
  snipResponse_[6].data = snipName_.c_str();
  snipResponse_[7].data = snipDesc_.c_str();
  b->data()->reset(nmsg(), snipResponse_, Defs::MTI_IDENT_INFO_REPLY);
  // We must wait for the data to be sent out because we have a static member
  // that we are changing constantly.
  b->set_done(n_.reset(this));
  responseFlow_->send(b);
  release();
  return wait_and_call(STATE(send_done));
}

StateFlowBase::Action TrainSnipHandler::send_done()
{
  return exit();
}

openlcb::SimpleInfoDescriptor TrainSnipHandler::snipResponse_[] =
{
  {SimpleInfoDescriptor::LITERAL_BYTE, 4, 0, nullptr},
  {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.manufacturer_name},
  {SimpleInfoDescriptor::C_STRING, 41, 0, "Virtual train node"},
  {SimpleInfoDescriptor::C_STRING, 0, 0, "n/a"},
  {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.software_version},
  {SimpleInfoDescriptor::LITERAL_BYTE, 2, 0, nullptr}, // version
  {SimpleInfoDescriptor::C_STRING, 63, 1, nullptr},    // name
  {SimpleInfoDescriptor::C_STRING, 64, 64, nullptr},   // description
  {SimpleInfoDescriptor::END_OF_DATA, 0, 0, 0}
};

} // namespace trainmanager