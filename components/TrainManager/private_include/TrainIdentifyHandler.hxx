/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * SPDX-FileContributor: 2020-2023 Mike Dunston (atanisoft)
 *
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