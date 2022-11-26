/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */
#include "TrainIdentifyHandler.hxx"

#include "TrainManager.hxx"

#include <openlcb/Defs.hxx>
#include <openlcb/If.hxx>
#include <openlcb/TractionDefs.hxx>
#include <utils/logging.h>

namespace trainmanager
{

#ifndef TRAINIDENT_LOGLEVEL
#ifdef CONFIG_LOCOMGR_IDENT_LOGGING_LEVEL
#define TRAINIDENT_LOGLEVEL CONFIG_LOCOMGR_IDENT_LOGGING_LEVEL
#else
#define TRAINIDENT_LOGLEVEL VERBOSE
#endif // CONFIG_LOCOMGR_IDENT_LOGGING_LEVEL
#endif // TRAINIDENT_LOGLEVEL

using openlcb::Defs;
using openlcb::IncomingMessageStateFlow;
using openlcb::TractionDefs;

TrainIdentifyHandler::TrainIdentifyHandler(TrainManager *parent)
  : IncomingMessageStateFlow(parent->train_service()->iface())
  , parent_(parent)
{
  iface()->dispatcher()->register_handler(
          this, Defs::MTI::MTI_VERIFY_NODE_ID_GLOBAL, Defs::MTI::MTI_EXACT);
}

TrainIdentifyHandler::~TrainIdentifyHandler()
{
  iface()->dispatcher()->unregister_handler(
          this, Defs::MTI_VERIFY_NODE_ID_GLOBAL, Defs::MTI::MTI_EXACT);
}

StateFlowBase::Action TrainIdentifyHandler::entry()
{
  openlcb::GenMessage *m = message()->data();
  if (!m->payload.empty() && m->payload.size() == 6)
  {
    target_ = openlcb::buffer_to_node_id(m->payload);
#if TRAINIDENT_LOGLEVEL >= VERBOSE
    LOG(TRAINIDENT_LOGLEVEL,
        "[TrainIdent] received global identify for node %s",
        utils::node_id_to_string(target_).c_str());
#endif // TRAINIDENT_LOGLEVEL >= VERBOSE
    openlcb::NodeID masked = target_ & TractionDefs::NODE_ID_MASK;
    if ((masked == TractionDefs::NODE_ID_DCC ||
         masked == TractionDefs::NODE_ID_MARKLIN_MOTOROLA ||
         masked == locomgr::NODE_ID_OLCB) &&
         parent_->is_valid_train_node(target_))

    {
      LOG(TRAINIDENT_LOGLEVEL, "[TrainIdent] matched a known train db entry");
      release();
      return allocate_and_call(iface()->global_message_write_flow(),
                               STATE(send_train_ident));
    }
  }
  return release_and_exit();
}

StateFlowBase::Action TrainIdentifyHandler::send_train_ident()
{
  auto *b =
      get_allocation_result(iface()->global_message_write_flow());
  openlcb::GenMessage *m = b->data();
  m->reset(Defs::MTI_VERIFIED_NODE_ID_NUMBER, target_
          , openlcb::node_id_to_buffer(target_));
  iface()->global_message_write_flow()->send(b);
  return exit();
}

} // namespace trainmanager