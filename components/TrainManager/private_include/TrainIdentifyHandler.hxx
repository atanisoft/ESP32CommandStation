/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TRAINIDENTIFYHANDLER_HXX_
#define TRAINIDENTIFYHANDLER_HXX_

#include <executor/StateFlow.hxx>
#include <openlcb/Defs.hxx>
#include <openlcb/If.hxx>

namespace trainmanager
{

class TrainManager;

class TrainIdentifyHandler : public openlcb::IncomingMessageStateFlow
{
public:
    TrainIdentifyHandler(TrainManager *parent);
    ~TrainIdentifyHandler();
    /// Handler callback for incoming messages.
    StateFlowBase::Action entry() override;

private:
    TrainManager *parent_;
    openlcb::NodeID target_;
    STATE_FLOW_STATE(send_train_ident);
};

} // namespace trainmanager

#endif // TRAINIDENTIFYHANDLER_HXX_