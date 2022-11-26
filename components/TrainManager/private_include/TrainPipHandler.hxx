/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TRAINPIPHANDLER_HXX_
#define TRAINPIPHANDLER_HXX_

#include <executor/Notifiable.hxx>
#include <executor/StateFlow.hxx>
#include <openlcb/Defs.hxx>
#include <openlcb/If.hxx>

namespace trainmanager
{
class TrainManager;

class TrainPipHandler : public openlcb::IncomingMessageStateFlow
{
public:
    TrainPipHandler(TrainManager* parent);
    ~TrainPipHandler();
private:
    static constexpr uint64_t pipReply_ =
      openlcb::Defs::SIMPLE_PROTOCOL_SUBSET | openlcb::Defs::DATAGRAM |
      openlcb::Defs::MEMORY_CONFIGURATION | openlcb::Defs::EVENT_EXCHANGE |
      openlcb::Defs::SIMPLE_NODE_INFORMATION | openlcb::Defs::TRACTION_CONTROL |
      openlcb::Defs::TRACTION_FDI | openlcb::Defs::CDI;
    TrainManager* parent_;

    StateFlowBase::Action entry() override;
    STATE_FLOW_STATE(fill_response_buffer);
};
} // namespace trainmanager

#endif // TRAINPIPHANDLER_HXX_