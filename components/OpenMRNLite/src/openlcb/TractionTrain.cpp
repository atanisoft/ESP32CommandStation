/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file TractionTrain.cxx
 *
 * Defines an NMRAnet Train node.
 *
 * @author Balazs Racz
 * @date 5 May 2014
 */

//#define LOGLEVEL VERBOSE

#include "openlcb/TractionTrain.hxx"

#include "utils/logging.h"
#include "openlcb/If.hxx"

namespace openlcb
{

DefaultTrainNode::DefaultTrainNode(TrainService *service, TrainImpl *train)
    : service_(service)
    , train_(train)
    , isInitialized_(0)
    , controllerNodeId_({0, 0})
{
}

TrainNode::~TrainNode()
{
}

TrainNodeWithConsist::~TrainNodeWithConsist()
{
    while (!consistSlaves_.empty())
    {
        delete consistSlaves_.pop_front();
    }
}

DefaultTrainNode::~DefaultTrainNode()
{
}

TrainNodeForProxy::TrainNodeForProxy(TrainService *service, TrainImpl *train)
    : DefaultTrainNode(service, train)
{
    service->register_train(this);
}

TrainNodeForProxy::~TrainNodeForProxy()
{
    /// @todo enable this line of code. It currently breaks unit tests due to
    /// bugs
    // service_->unregister_train(this);
}

TrainNodeWithId::TrainNodeWithId(
    TrainService *service, TrainImpl *train, NodeID node_id)
    : DefaultTrainNode(service, train)
    , nodeId_(node_id)
{
    service->register_train(this);
}

TrainNodeWithId::~TrainNodeWithId()
{
    /// @todo enable this line of code. It currently breaks unit tests due to
    /// bugs
    // service_->unregister_train(this);
}

NodeID TrainNodeForProxy::node_id()
{
    return TractionDefs::train_node_id_from_legacy(
        train_->legacy_address_type(), train_->legacy_address());
}

If *DefaultTrainNode::iface()
{
    return service_->iface();
}

/// Implementation structure for TrainService. Holds ownership of the various
/// flows that are necessary for the correct operation, but do not need to be
/// exposed on the external API.
struct TrainService::Impl
{
    class TractionRequestFlow;

    Impl(TrainService *parent) : traction_(parent)
    {
    }

    /// Handler for incoming OpenLCB messages of MTI == Traction Protocol
    /// Request.
    class TractionRequestFlow : public IncomingMessageStateFlow
    {
    public:
        TractionRequestFlow(TrainService *service)
            : IncomingMessageStateFlow(service->iface())
            , reserved_(0)
            , trainService_(service)
            , response_(nullptr)
        {
            iface()->dispatcher()->register_handler(
                this, Defs::MTI_TRACTION_CONTROL_COMMAND, 0xffff);
        }

        ~TractionRequestFlow()
        {
            iface()->dispatcher()->unregister_handler(
                this, Defs::MTI_TRACTION_CONTROL_COMMAND, 0xffff);
        }

    protected:
        TrainNode *train_node()
        {
            return static_cast<TrainNode *>(nmsg()->dstNode);
        }

        Action maybe_alloc_response(Callback c)
        {
            if (response_)
            {
                return call_immediately(c);
            }
            else
            {
                return allocate_and_call(
                    iface()->addressed_message_write_flow(), c);
            }
        }

        void ensure_response_exists()
        {
            if (!response_)
            {
                response_ = get_allocation_result(
                    iface()->addressed_message_write_flow());
            }
        }

        Action entry() OVERRIDE
        {
            // If the message is not for a local node, ignore.
            if (!nmsg()->dstNode)
            {
                LOG(VERBOSE, "Traction message for unknown node.");
                return release_and_exit();
            }
            // Checks if destination is a local traction-enabled node.
            if (!trainService_->nodes_->is_node_registered(train_node()))
            {
                LOG(VERBOSE, "Traction message for node %p that is not "
                             "traction enabled.",
                    train_node());
                /** @TODO(balazs.racz): This is probably not the good solution
                 * here; since this is an addressed message we should rather
                 * send a reject response. */
                return release_and_exit();
            }
            train_node()->command_hook(nmsg()->src, nmsg()->payload);
            // No command byte?
            if (size() < 1)
            {
                LOG(VERBOSE, "Traction message with no command byte.");
                return reject_permanent();
            }
            uint8_t cmd = payload()[0] & TractionDefs::REQ_MASK;
            switch (cmd)
            {
                /** @TODO(balazs.racz) need to validate caller of mutating
                 * functions. The mutating options should be factored into a
                 * separate flow state. */
                case TractionDefs::REQ_SET_SPEED:
                {
                    SpeedType sp = fp16_to_speed(payload() + 1);
                    train_node()->train()->set_speed(sp);
                    nextConsistIndex_ = 0;
                    return call_immediately(STATE(maybe_forward_consist));
                }
                case TractionDefs::REQ_SET_FN:
                {
                    uint32_t address = payload()[1];
                    address <<= 8;
                    address |= payload()[2];
                    address <<= 8;
                    address |= payload()[3];
                    uint16_t value = payload()[4];
                    value <<= 8;
                    value |= payload()[5];
                    bn_.reset(this);
                    bool should_apply =
                        train_node()->function_policy(nmsg()->src, payload()[0],
                            address, value, bn_.new_child());
                    // The function_policy call may have completed inline. We
                    // can inquire from the barrier. If it was not completed
                    // inline, we have to wait for the notification and re-try
                    // the call.
                    if (!bn_.abort_if_almost_done())
                    {
                        // Not notified inline.
                        bn_.notify(); // consumes our share
                        return wait();
                    }
                    if (should_apply)
                    {
                        train_node()->train()->set_fn(address, value);
                    }
                    nextConsistIndex_ = 0;
                    return call_immediately(STATE(maybe_forward_consist));
                }
                case TractionDefs::REQ_EMERGENCY_STOP:
                {
                    train_node()->train()->set_emergencystop();
                    nextConsistIndex_ = 0;
                    return call_immediately(STATE(maybe_forward_consist));
                }
                case TractionDefs::REQ_QUERY_SPEED: // fall through
                case TractionDefs::REQ_QUERY_FN:
                {
                    // Need a response message first.
                    return maybe_alloc_response(STATE(handle_query));
                }
                case TractionDefs::REQ_CONTROLLER_CONFIG:
                {
                    return maybe_alloc_response(
                        STATE(handle_controller_config));
                }
                case TractionDefs::REQ_CONSIST_CONFIG:
                {
                    return maybe_alloc_response(
                        STATE(handle_consist_config));
                }
                case TractionDefs::REQ_TRACTION_MGMT:
                {
                    return maybe_alloc_response(STATE(handle_traction_mgmt));
                }
                default:
                {
                    LOG(VERBOSE, "Rejecting unknown traction message.");
                    return reject_permanent();
                }
            }
        }

        Action handle_query()
        {
            Payload *p = initialize_response();
            uint8_t cmd = payload()[0];
            switch (cmd)
            {
                case TractionDefs::REQ_QUERY_SPEED:
                {
                    p->resize(8);
                    uint8_t *d = reinterpret_cast<uint8_t *>(&(*p)[0]);
                    d[0] = TractionDefs::RESP_QUERY_SPEED;
                    speed_to_fp16(train_node()->train()->get_speed(), d + 1);
                    uint8_t status = 0;
                    if (train_node()->train()->get_emergencystop())
                    {
                        status |= TractionDefs::SPEEDRESP_STATUS_IS_ESTOP;
                    }
                    d[3] = status;
                    speed_to_fp16(train_node()->train()->get_commanded_speed(),
                                  d + 4);
                    speed_to_fp16(train_node()->train()->get_actual_speed(),
                                  d + 6);
                    return send_response();
                }
                case TractionDefs::REQ_QUERY_FN:
                {
                    p->resize(6);
                    uint8_t *d = reinterpret_cast<uint8_t *>(&(*p)[0]);
                    d[0] = TractionDefs::RESP_QUERY_FN;
                    d[1] = payload()[1];
                    d[2] = payload()[2];
                    d[3] = payload()[3];
                    uint32_t address = payload()[1];
                    address <<= 8;
                    address |= payload()[2];
                    address <<= 8;
                    address |= payload()[3];
                    uint16_t fn_value = train_node()->train()->get_fn(address);
                    d[4] = fn_value >> 8;
                    d[5] = fn_value & 0xff;
                    return send_response();
                }
            }
            DIE("unexpected call to handle_query.");
        }

        Action handle_controller_config()
        {
            Payload &p = *initialize_response();
            uint8_t subcmd = payload()[1];
            switch (subcmd)
            {
                case TractionDefs::CTRLREQ_ASSIGN_CONTROLLER:
                {
                    p.resize(3);
                    p[0] = TractionDefs::RESP_CONTROLLER_CONFIG;
                    p[1] = TractionDefs::CTRLRESP_ASSIGN_CONTROLLER;
                    NodeHandle supplied_controller = {0, 0};
                    if (size() < 9)
                        return reject_permanent();
                    supplied_controller.id = data_to_node_id(payload() + 3);
                    if (size() >= 11 && (payload()[2] & 0x01))
                    {
                        uint16_t alias = payload()[9];
                        alias <<= 8;
                        alias |= payload()[10];
                        supplied_controller.alias = alias;
                    }
                    NodeHandle existing_controller =
                        train_node()->get_controller();
                    if (false && // TODO(balazs.racz) this will automatically "steal" the loco but forgets to notify the old controller that it's stolen.
                        existing_controller.id &&
                        !iface()->matching_node(existing_controller,
                                                supplied_controller))
                    {
                        /** @TODO (balazs.racz): we need to implement stealing
                         * a train from the existing controller. */
                        p[2] = TractionDefs::CTRLRESP_ASSIGN_ERROR_CONTROLLER;
                        return send_response();
                    }
                    train_node()->set_controller(supplied_controller);
                    p[2] = 0;
                    return send_response();
                }
                case TractionDefs::CTRLREQ_QUERY_CONTROLLER:
                {
                    NodeHandle h = train_node()->get_controller();
                    p.reserve(11);
                    p.resize(9);
                    p[0] = TractionDefs::RESP_CONTROLLER_CONFIG;
                    p[1] = TractionDefs::CTRLRESP_QUERY_CONTROLLER;
                    p[2] = 0;
                    node_id_to_data(h.id, &p[3]);
                    if (h.alias)
                    {
                        p[2] |= 1;
                        p.push_back(h.alias >> 8);
                        p.push_back(h.alias & 0xff);
                    }
                    return send_response();
                }
                case TractionDefs::CTRLREQ_RELEASE_CONTROLLER:
                {
                    NodeHandle supplied_controller = {0, 0};
                    if (size() < 9)
                        return reject_permanent();
                    supplied_controller.id = data_to_node_id(payload() + 3);
                    if (size() >= 11 && (payload()[2] & 0x01))
                    {
                        uint16_t alias = payload()[9];
                        alias <<= 8;
                        alias |= payload()[10];
                        supplied_controller.alias = alias;
                    }
                    NodeHandle existing_controller =
                        train_node()->get_controller();
                    if (!iface()->matching_node(existing_controller,
                                                    supplied_controller))
                    {
                        LOG(WARNING,
                            "Tried to release a train that was not held: "
                            "train's controller %012" PRIx64
                            ", release command's controller %012" PRIx64,
                            existing_controller.id, supplied_controller.id);
                        return release_and_exit();
                    }
                    existing_controller = {0, 0};
                    train_node()->set_controller(existing_controller);
                    return release_and_exit();
                }
            }
            LOG(VERBOSE, "Rejecting unknown traction message.");
            return reject_permanent();
        }

        Action handle_consist_config()
        {
            uint8_t cmd = payload()[1];
            switch (cmd)
            {
                case TractionDefs::CNSTREQ_ATTACH_NODE:
                {
                    if (size() < 9)
                        return reject_permanent(
                            Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
                    NodeID target = data_to_node_id(payload() + 3);
                    uint8_t flags = payload()[2];
                    if (target == train_node()->node_id())
                    {
                        return init_and_send_response(
                            TractionDefs::consist_add_response(
                                target, Defs::ERROR_OPENMRN_ALREADY_EXISTS));
                    }
                    train_node()->add_consist(target, flags);
                    return init_and_send_response(
                        TractionDefs::consist_add_response(target, 0));
                }
                case TractionDefs::CNSTREQ_DETACH_NODE:
                {
                    if (size() < 9)
                        return reject_permanent(
                            Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
                    NodeID target = data_to_node_id(payload() + 3);
                    bool resp = train_node()->remove_consist(target);
                    return init_and_send_response(
                        TractionDefs::consist_del_response(
                            target, resp ? 0 : Defs::ERROR_OPENMRN_NOT_FOUND));
                }
                case TractionDefs::CNSTREQ_QUERY_NODES:
                {
                    int sz = train_node()->query_consist_length();
                    if (sz > 255) sz = 255;
                    if (size() > 2) {
                        uint8_t id = payload()[2];
                        if (id < sz) {
                            uint8_t flags = 0;
                            NodeID rp = train_node()->query_consist(id, &flags);
                            return init_and_send_response(
                                TractionDefs::consist_qry_response_long(
                                    sz, id, flags, rp));
                        }
                    }
                    return init_and_send_response(
                        TractionDefs::consist_qry_response_short(sz));
                }
                default:
                    LOG(VERBOSE, "Unknown Traction consist subcommand %x", cmd);
                    return reject_permanent(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
            }
        }

        Action maybe_forward_consist()
        {
            auto* train_node = this->train_node();
            unsigned count = train_node->query_consist_length();
            if (count <= nextConsistIndex_)
                return release_and_exit();
            uint8_t flags = 0;
            NodeID dst = train_node->query_consist(nextConsistIndex_, &flags);
            if (iface()->matching_node(nmsg()->src, NodeHandle(dst)))
            {
                ++nextConsistIndex_;
                return again();
            }
            uint8_t cmd = payload()[0] & TractionDefs::REQ_MASK;
            bool flip_speed = false;
            if (cmd == TractionDefs::REQ_SET_SPEED) {
                if (flags & TractionDefs::CNSTFLAGS_REVERSE) {
                    flip_speed = true;
                }
            } else if (cmd == TractionDefs::REQ_SET_FN) {
                uint32_t address = payload()[1];
                address <<= 8;
                address |= payload()[2];
                address <<= 8;
                address |= payload()[3];
                if (address == 0) {
                    if ((flags & TractionDefs::CNSTFLAGS_LINKF0) == 0) {
                        // skip
                        ++nextConsistIndex_;
                        return again();
                    }
                } else {
                    if ((flags & TractionDefs::CNSTFLAGS_LINKFN) == 0) {
                        // skip
                        ++nextConsistIndex_;
                        return again();
                    }
                }
            }
            if (count == nextConsistIndex_ + 1u)
            {
                // last node: we can transfer the message.
                auto *b = transfer_message();
                b->data()->src = NodeHandle(train_node->node_id());
                b->data()->dst = NodeHandle(dst);
                b->data()->dstNode = nullptr;
                if (flip_speed) {
                    b->data()->payload[1] ^= 0x80;
                }
                b->data()->payload[0] |= TractionDefs::REQ_LISTENER;
                iface()->addressed_message_write_flow()->send(b);
                return exit();
            }
            else
            {
                return allocate_and_call(
                    iface()->addressed_message_write_flow(),
                    STATE(forward_consist));
            }
        }

        Action forward_consist()
        {
            auto *b =
                get_allocation_result(iface()->addressed_message_write_flow());
            uint8_t flags;
            NodeID dst = train_node()->query_consist(nextConsistIndex_, &flags);
            if (!dst)
            {
                // Strange. The consist destination should exist and never be
                // zero once we got here.
                b->unref();
                return release_and_exit();
            }
            b->data()->reset(message()->data()->mti, train_node()->node_id(),
                             NodeHandle(dst), message()->data()->payload);
            b->data()->payload[0] |= TractionDefs::REQ_LISTENER;
            if (((payload()[0] & TractionDefs::REQ_MASK) ==
                    TractionDefs::REQ_SET_SPEED) &&
                (flags & TractionDefs::CNSTFLAGS_REVERSE))
            {
                b->data()->payload[1] ^= 0x80;
            }
            iface()->addressed_message_write_flow()->send(b);
            ++nextConsistIndex_;
            return call_immediately(STATE(maybe_forward_consist));
        }

        Action handle_traction_mgmt()
        {
            Payload &p = *initialize_response();
            uint8_t cmd = payload()[1];
            switch (cmd)
            {
                case TractionDefs::MGMTREQ_RESERVE:
                {
                    uint8_t code = 0xff;
                    if (reserved_)
                    {
                        code = 0x1;
                    }
                    else
                    {
                        code = 0;
                        reserved_ = 1;
                    }
                    p.push_back(TractionDefs::RESP_TRACTION_MGMT);
                    p.push_back(TractionDefs::MGMTRESP_RESERVE);
                    p.push_back(code);
                    return send_response();
                }
                case TractionDefs::MGMTREQ_RELEASE:
                {
                    reserved_ = 0;
                    return release_and_exit();
                }
                case TractionDefs::MGMTREQ_NOOP:
                {
                    // Nothing to do.
                    return release_and_exit();
                }
                default:
                    LOG(VERBOSE, "Unknown Traction management subcommand %x",
                        cmd);
                    return reject_permanent();
            }
        }

        /** Takes the allocation result of a response buffer (addressed write
         * flow) and fills in src, dest as a response message for traction
         * protocol. The caller only needs to provide the payload.
         */
        Payload *initialize_response()
        {
            ensure_response_exists();
            response_->data()->reset(Defs::MTI_TRACTION_CONTROL_REPLY,
                                     train_node()->node_id(), nmsg()->src,
                                     EMPTY_PAYLOAD);
            return &response_->data()->payload;
        }

        Action init_and_send_response(Payload p)
        {
            ensure_response_exists();
            response_->data()->reset(Defs::MTI_TRACTION_CONTROL_REPLY,
                                     train_node()->node_id(), nmsg()->src,
                                     std::move(p));
            return send_response();
        }

        /** Sends off the response buffer to the client. */
        Action send_response()
        {
            iface()->addressed_message_write_flow()->send(response_);
            response_ = nullptr;
            return release_and_exit();
        }

        /** Returns the size of the incoming message payload. */
        size_t size()
        {
            return nmsg()->payload.size();
        }

        /** Returns the incoming message payload (bytes). */
        const uint8_t *payload()
        {
            return reinterpret_cast<const uint8_t *>(nmsg()->payload.data());
        }

        /** Rejects the incoming message with a permanent error. */
        Action reject_permanent(uint16_t code = Defs::ERROR_PERMANENT)
        {
            return maybe_alloc_response(STATE(send_reject_permanent));
        }

        Action send_reject_permanent()
        {
            ensure_response_exists();
            // An alternative would be to send TERMINATE_DUE_TO_ERROR here.
            response_->data()->reset(
                Defs::MTI_OPTIONAL_INTERACTION_REJECTED,
                nmsg()->dstNode->node_id(), nmsg()->src,
                error_to_buffer(errorCode_, nmsg()->mti));
            return send_response();
        }

    private:
        /// error code for reject_permanent().
        unsigned errorCode_ : 16;
        unsigned nextConsistIndex_ : 8;
        /// 1 if the voluntary lock protocol has set this train to be reserved.
        unsigned reserved_ : 1;
        TrainService *trainService_;
        Buffer<GenMessage> *response_;
        BarrierNotifiable bn_;
    };

    TractionRequestFlow traction_;
};

TrainService::TrainService(If *iface, NodeRegistry *train_node_registry)
    : Service(iface->executor())
    , iface_(iface)
    , nodes_(train_node_registry)
{
    impl_ = new Impl(this);
}

TrainService::~TrainService()
{
    delete impl_;
}

void TrainService::register_train(TrainNode *node)
{
    iface_->add_local_node(node);
    extern void StartInitializationFlow(Node * node);
    StartInitializationFlow(node);
    AtomicHolder h(this);
    nodes_->register_node(node);
    LOG(VERBOSE, "Registered node %p for traction.", node);
}

void TrainService::unregister_train(TrainNode *node)
{
    HASSERT(nodes_->is_node_registered(node));
    iface_->delete_local_node(node);
    AtomicHolder h(this);
    nodes_->unregister_node(node);
}

} // namespace openlcb
