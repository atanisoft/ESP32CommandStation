/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2023 Mike Dunston (atanisoft)
 *
 */

#include "TrainPipHandler.hxx"
#include "TrainManager.hxx"
namespace trainmanager
{

using openlcb::Defs;
using openlcb::IncomingMessageStateFlow;

TrainPipHandler::TrainPipHandler(TrainManager* parent)
    : IncomingMessageStateFlow(parent->train_service()->iface()),
      parent_(parent)
{
  iface()->dispatcher()->register_handler(
      this, Defs::MTI::MTI_PROTOCOL_SUPPORT_INQUIRY, Defs::MTI::MTI_EXACT);
}

TrainPipHandler::~TrainPipHandler()
{
  iface()->dispatcher()->unregister_handler(
      this, Defs::MTI::MTI_PROTOCOL_SUPPORT_INQUIRY, Defs::MTI::MTI_EXACT);
}

StateFlowBase::Action TrainPipHandler::entry()
{
  if (!parent_->is_valid_train_node(nmsg()->dstNode))
  {
    return release_and_exit();
  }

  return allocate_and_call(iface()->addressed_message_write_flow(),
                            STATE(fill_response_buffer));
}

StateFlowBase::Action TrainPipHandler::fill_response_buffer()
{
  // Grabs our allocated buffer.
  auto* b = get_allocation_result(iface()->addressed_message_write_flow());
  auto reply = pipReply_;
  // Fills in response. We use node_id_to_buffer because that converts a
  // 48-bit value to a big-endian byte string.
  b->data()->reset(Defs::MTI_PROTOCOL_SUPPORT_REPLY,
                    nmsg()->dstNode->node_id(), nmsg()->src,
                    openlcb::node_id_to_buffer(reply));

  // Passes the response to the addressed message write flow.
  iface()->addressed_message_write_flow()->send(b);

  return release_and_exit();
}

} // namespace trainmanager