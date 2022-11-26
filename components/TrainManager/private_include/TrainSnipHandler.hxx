/*
 * SPDX-FileCopyrightText: 2020 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TRAINSNIPHANDLER_HXX_
#define TRAINSNIPHANDLER_HXX_

#include <executor/StateFlow.hxx>

namespace openlcb
{
    class IncomingMessageStateFlow;
    class SimpleInfoFlow;
    struct SimpleInfoDescriptor;
}

namespace trainmanager
{
class TrainManager;

class TrainSnipHandler : public openlcb::IncomingMessageStateFlow
{
public:
    TrainSnipHandler(TrainManager* parent, openlcb::SimpleInfoFlow* info_flow);
    ~TrainSnipHandler();
private:
    TrainManager* parent_;
    openlcb::SimpleInfoFlow* responseFlow_;
    BarrierNotifiable n_;
    string snipName_;
    string snipDesc_;
    static openlcb::SimpleInfoDescriptor snipResponse_[];

    StateFlowBase::Action entry() override;
    STATE_FLOW_STATE(send_response_request);
    STATE_FLOW_STATE(send_done);
};
} // namespace trainmanager

#endif // TRAINSNIPHANDLER_HXX_