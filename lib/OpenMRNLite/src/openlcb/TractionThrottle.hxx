/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file TractionThrottle.hxx
 *
 * Client API for the Traction protocol.
 *
 * @author Balazs Racz
 * @date 20 May 2014
 */

#ifndef _OPENLCB_TRACTIONTHROTTLE_HXX_
#define _OPENLCB_TRACTIONTHROTTLE_HXX_

#include "openlcb/TractionClient.hxx"
#include "openlcb/TractionDefs.hxx"
#include "openlcb/TrainInterface.hxx"
#include "executor/CallableFlow.hxx"

namespace openlcb
{

struct TractionThrottleInput;

/// C++ Namespace for collecting all commands that can be sent to the
/// TractionThrottle flow.
struct TractionThrottleCommands
{
    enum SetDst
    {
        SET_DST,
    };

    enum AssignTrain
    {
        ASSIGN_TRAIN,
    };

    enum ReleaseTrain
    {
        RELEASE_TRAIN,
    };

    enum LoadState
    {
        LOAD_STATE,
    };

    enum ConsistAdd
    {
        CONSIST_ADD,
    };

    enum ConsistDel
    {
        CONSIST_DEL,
    };

    enum ConsistQry
    {
        CONSIST_QRY,
    };
};

/// Request structure used to send requests to the TractionThrottle
/// class. Contains parametrized reset calls for properly supporting
/// @ref StateFlowBase::invoke_subflow_and_wait() syntax.
struct TractionThrottleInput : public CallableFlowRequestBase
{
    enum Command
    {
        CMD_SET_DST,
        CMD_ASSIGN_TRAIN,
        CMD_RELEASE_TRAIN,
        CMD_LOAD_STATE,
        CMD_CONSIST_ADD,
        CMD_CONSIST_DEL,
        CMD_CONSIST_QRY,
    };

    /// Sets the destination node to send messages to without sending assign
    /// commands to that train node.
    void reset(const TractionThrottleCommands::SetDst &, const NodeID &dst)
    {
        cmd = CMD_SET_DST;
        this->dst = dst;
    }

    void reset(const TractionThrottleCommands::AssignTrain &, const NodeID &dst,
        bool listen)
    {
        cmd = CMD_ASSIGN_TRAIN;
        this->dst = dst;
        this->flags = listen ? 1 : 0;
    }

    void reset(const TractionThrottleCommands::ReleaseTrain &)
    {
        cmd = CMD_RELEASE_TRAIN;
    }

    void reset(const TractionThrottleCommands::LoadState &)
    {
        cmd = CMD_LOAD_STATE;
    }

    void reset(const TractionThrottleCommands::ConsistAdd &, NodeID slave, uint8_t flags)
    {
        cmd = CMD_CONSIST_ADD;
        dst = slave;
        this->flags = flags;
    }

    void reset(const TractionThrottleCommands::ConsistDel &, NodeID slave)
    {
        cmd = CMD_CONSIST_DEL;
        dst = slave;
    }

    void reset(const TractionThrottleCommands::ConsistQry &)
    {
        cmd = CMD_CONSIST_QRY;
        replyCause = 0xff;
    }

    void reset(const TractionThrottleCommands::ConsistQry &, uint8_t ofs)
    {
        cmd = CMD_CONSIST_QRY;
        consistIndex = ofs;
        replyCause = 0;
    }

    Command cmd;
    /// For assign, this carries the destination node ID. For consisting
    /// requests, this is an in-out argument.
    NodeID dst;
    /// Contains the flags for the consist listener. Specified for Attach
    /// requests, and filled for Query responses.
    uint8_t flags;

    /// For assign controller reply REJECTED, this is 1 for controller refused
    /// connection, 2 fortrain refused connection.
    uint8_t replyCause;
    /// Total number of entries in the consisting list.
    uint8_t consistCount;
    /// Index of the entry in the consisting list that needs to be returned.
    uint8_t consistIndex;
};

class TractionThrottleInterface
    : public openlcb::TrainImpl
{
public:
    virtual void toggle_fn(uint32_t fn) = 0;

    /// Determine if a train is currently assigned to this trottle.
    /// @return true if a train is assigned, else false
    virtual bool is_train_assigned() = 0;

    /// @return the controlling node (virtual node of the throttle, i.e., us.)
    /// @todo this function should not be here
    virtual openlcb::Node* throttle_node() = 0;

    /// Sets up a callback for listening for remote throttle updates. When a
    /// different throttle modifies the train node's state, and the
    /// ASSIGN_TRAIN command was executed with "listen==true" parameter, we
    /// will get notifications about those remote changes. The notifications
    /// update the cached state in TractionThrottle, and call this update
    /// callback. Repeat with nullptr if the callbacks are not desired anymore.
    /// @param update_callback will be executed when a different throttle
    /// changes the train state. fn is the function number changed, or -1 for
    /// speed update.
    virtual void set_throttle_listener(std::function<void(int fn)> update_callback) = 0;

    /// @return the controlled node (the train node) ID.
    /// @todo this function should not be here
    virtual openlcb::NodeID target_node() = 0;
};

/** Interface for a single throttle for running a train node.
 *
 */
class TractionThrottle
    : public CallableFlow<TractionThrottleInput>,
      public TractionThrottleInterface
{
public:
    /// @param node is the openlcb node from which this throttle will be
    /// sending its messages.
    TractionThrottle(Node *node)
        : CallableFlow<TractionThrottleInput>(node->iface())
        , node_(node)
    {
        clear_cache();
    }

    ~TractionThrottle() {
    }

    using Command = TractionThrottleInput::Command;

    enum
    {
        /// Timeout for assign controller request.
        TIMEOUT_NSEC = SEC_TO_NSEC(1),
        /// Returned from get_fn() when we don't have a cahced value for a
        /// function.
        FN_NOT_KNOWN = 0xffff,
        /// Upon a load state request, how far do we go into the function list?
        MAX_FN_QUERY = 28,
        ERROR_UNASSIGNED = 0x4000000,
        ERROR_ASSIGNED = 0x4010000,
    };

    void set_speed(SpeedType speed) override
    {
        send_traction_message(TractionDefs::speed_set_payload(speed));
        lastSetSpeed_ = speed;
        estopActive_ = false;
    }

    SpeedType get_speed() override
    {
        // TODO: if we don't know the current speed, we should probably go and
        // ask.
        return lastSetSpeed_;
    }

    void set_emergencystop() override
    {
        send_traction_message(TractionDefs::estop_set_payload());
        estopActive_ = true;
        lastSetSpeed_.set_mph(0);
    }

    /// Get the current E-Stop state.
    /// @return true if the train is E-Stopped, else false
    bool get_emergencystop() override
    {
        return estopActive_;
    }

    void set_fn(uint32_t address, uint16_t value) override
    {
        send_traction_message(TractionDefs::fn_set_payload(address, value));
        lastKnownFn_[address] = value;
    }

    uint16_t get_fn(uint32_t address) override
    {
        auto it = lastKnownFn_.find(address);
        if (it != lastKnownFn_.end())
        {
            return it->second;
        }
        return FN_NOT_KNOWN;
    }

    void toggle_fn(uint32_t fn) override
    {
        auto fnstate = get_fn(fn);
        if (fnstate == FN_NOT_KNOWN)
        {
            fnstate = 1;
        }
        else
        {
            fnstate = !fnstate;
        }
        set_fn(fn, fnstate);        
    }
    
    uint32_t legacy_address() override
    {
        return 0;
    }

    dcc::TrainAddressType legacy_address_type() override
    {
        return dcc::TrainAddressType::DCC_SHORT_ADDRESS;
    }

    /// Determine if a train is currently assigned to this trottle.
    /// @return true if a train is assigned, else false
    bool is_train_assigned() override
    {
        return assigned_;
    }

    /// @return the controlling node (virtual node of the throttle, i.e., us.)
    openlcb::Node* throttle_node() override
    {
        return node_;
    }

    /// @return the controlled node (the train node) ID.
    openlcb::NodeID target_node() override
    {
        return dst_;
    }

    /// Sets up a callback for listening for remote throttle updates. When a
    /// different throttle modifies the train node's state, and the
    /// ASSIGN_TRAIN command was executed with "listen==true" parameter, we
    /// will get notifications about those remote changes. The notifications
    /// update the cached state in TractionThrottle, and call this update
    /// callback. Repeat with nullptr if the callbacks are not desired anymore.
    /// @param update_callback will be executed when a different throttle
    /// changes the train state. fn is the function number changed, or -1 for
    /// speed update.
    void set_throttle_listener(std::function<void(int fn)> update_callback) override
    {
        updateCallback_ = std::move(update_callback);
    }

private:
    Action entry() override
    {
        switch (message()->data()->cmd)
        {
            case Command::CMD_SET_DST:
            {
                if (assigned_)
                {
                    return return_with_error(ERROR_ASSIGNED);
                }
                dst_ = input()->dst;
                return return_ok();
            }
            case Command::CMD_ASSIGN_TRAIN:
            {
                if (assigned_)
                {
                    return call_immediately(STATE(release_train));
                }
                else
                {
                    return call_immediately(STATE(assign_train));
                }
                break;
            }
            case Command::CMD_RELEASE_TRAIN:
            {
                if (assigned_)
                {
                    return call_immediately(STATE(release_train));
                }
                else
                {
                    return return_ok();
                }
            }
            case Command::CMD_LOAD_STATE:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                return call_immediately(STATE(load_state));
            }
            case Command::CMD_CONSIST_ADD:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                if (!input()->dst)
                {
                    return return_with_error(Defs::ERROR_INVALID_ARGS);
                }
                return call_immediately(STATE(consist_add));
            }
            case Command::CMD_CONSIST_DEL:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                if (!input()->dst)
                {
                    return return_with_error(Defs::ERROR_INVALID_ARGS);
                }
                return call_immediately(STATE(consist_del));
            }
            case Command::CMD_CONSIST_QRY:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                return call_immediately(STATE(consist_qry));
            }
            default:
                LOG_ERROR("Unknown traction throttle command %d received.",
                    input()->cmd);
                return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
    }

    Action release_train()
    {
        if (listenConsist_)
        {
            handler_.wait_for_response(
                NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
            send_traction_message(
                TractionDefs::consist_del_payload(node_->node_id()));
            return sleep_and_call(
                &timer_, TIMEOUT_NSEC, STATE(release_listener_response));
        }
        else
        {
            return call_immediately(STATE(release_step_2));
        }
    }

    Action release_listener_response()
    {
        handler_.wait_timeout();
        if (handler_.response())
        {
            handler_.response()->unref();
        }
        clear_listening();
        return call_immediately(STATE(release_step_2));
    }

    Action release_step_2()
    {
        send_traction_message(TractionDefs::release_controller_payload(node_));
        clear_assigned();
        clear_cache();
        if (input()->cmd == Command::CMD_ASSIGN_TRAIN)
        {
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(50), STATE(assign_train));
        }
        else
        {
            return return_ok();
        }
    }

    Action assign_train()
    {
        dst_ = input()->dst;
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONTROLLER_CONFIG, &timer_);
        send_traction_message(TractionDefs::assign_controller_payload(node_));
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(assign_response));
    }

    Action assign_response()
    {
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        const string &payload = handler_.response()->data()->payload;
        if (payload.size() < 3)
        {
            return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
        if (payload[1] != TractionDefs::CTRLRESP_ASSIGN_CONTROLLER)
        {
            // spurious reply message
            return return_with_error(Defs::ERROR_OUT_OF_ORDER);
        }
        input()->replyCause = payload[2];
        if (payload[2] != 0)
        {
            return return_with_error(Defs::ERROR_REJECTED);
        }
        set_assigned();
        if (input()->flags)
        {
            // need to add consist listener
            handler_.wait_for_response(
                NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
            send_traction_message(TractionDefs::consist_add_payload(
                node_->node_id(),
                TractionDefs::CNSTFLAGS_HIDE | TractionDefs::CNSTFLAGS_LINKF0 |
                    TractionDefs::CNSTFLAGS_LINKFN));
            return sleep_and_call(
                &timer_, TIMEOUT_NSEC, STATE(assign_consist_response));
        }
        return return_ok();
    }

    Action assign_consist_response()
    {
        // All error responses are actually okay here; we succeeded in the
        // assignment but the listener setup didn't work.
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_ok();
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        // Marks that we are owning the listener.
        set_listening();
        return return_ok();
    }

    Action load_state()
    {
        pendingQueries_ = 1;
        send_traction_message(TractionDefs::speed_get_payload());
        for (int i = 0; i <= MAX_FN_QUERY; ++i)
        {
            pendingQueries_++;
            send_traction_message(TractionDefs::fn_get_payload(i));
        }
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(load_done));
    }

    Action load_done()
    {
        if (!timer_.is_triggered())
        {
            // timed out
            pendingQueries_ = 0;
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }
        else
        {
            return return_ok();
        }
    }

    void pending_reply_arrived()
    {
        if (pendingQueries_ > 0)
        {
            if (!--pendingQueries_)
            {
                timer_.trigger();
            }
        }
    }

    void speed_reply(Buffer<GenMessage> *msg)
    {
        AutoReleaseBuffer<GenMessage> rb(msg);
        if (!iface()->matching_node(msg->data()->src, NodeHandle(dst_)))
        {
            return;
        }
        const Payload &p = msg->data()->payload;
        if (p.size() < 1)
            return;
        switch (p[0])
        {
            case TractionDefs::RESP_QUERY_SPEED:
            {
                pending_reply_arrived();
                Velocity v;
                if (TractionDefs::speed_get_parse_last(p, &v))
                {
                    lastSetSpeed_ = v;
                    /// @todo (balazs.racz): call a callback for the client.

                    /// @todo (Stuart.Baker): Do we need to do anything with
                    /// estopActive_?
                }
                return;
            }
            case TractionDefs::RESP_QUERY_FN:
            {
                pending_reply_arrived();
                uint16_t v;
                unsigned num;
                if (TractionDefs::fn_get_parse(p, &v, &num))
                {
                    lastKnownFn_[num] = v;
                }
            }
        }
    }

    Action consist_add()
    {
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
        send_traction_message(
            TractionDefs::consist_add_payload(input()->dst, input()->flags));
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(consist_add_response));
    }

    Action consist_del()
    {
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
        send_traction_message(TractionDefs::consist_del_payload(input()->dst));
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(consist_add_response));
    }

    Action consist_add_response()
    {
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        const string &payload = handler_.response()->data()->payload;
        if (payload.size() < 9)
        {
            return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
        if (message()->data()->cmd == Command::CMD_CONSIST_ADD)
        {
            if (payload[1] != TractionDefs::CNSTRESP_ATTACH_NODE)
            {
                // spurious reply message
                return return_with_error(Defs::ERROR_OUT_OF_ORDER);
            }
        }
        else if (message()->data()->cmd == Command::CMD_CONSIST_DEL)
        {
            if (payload[1] != TractionDefs::CNSTRESP_DETACH_NODE)
            {
                // spurious reply message
                return return_with_error(Defs::ERROR_OUT_OF_ORDER);
            }
        } else if (data_to_node_id(&payload[2]) != input()->dst) {
            // spurious reply message
            return return_with_error(Defs::ERROR_OUT_OF_ORDER);
        }
        uint16_t e = payload[8];
        e <<= 8;
        e |= payload[9];
        input()->replyCause = e ? 1 : 0;
        return return_with_error(e);
    }

    Action consist_qry()
    {
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
        if (input()->replyCause == 0xff)
        {
            send_traction_message(TractionDefs::consist_qry_payload());
        }
        else
        {
            send_traction_message(
                TractionDefs::consist_qry_payload(input()->consistIndex));
        }
        return sleep_and_call(
            &timer_, TIMEOUT_NSEC, STATE(consist_qry_response));
    }

    Action consist_qry_response()
    {
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        const string &payload = handler_.response()->data()->payload;
        if (payload.size() < 3)
        {
            return return_with_error(Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
        }
        if (payload[1] != TractionDefs::CNSTRESP_QUERY_NODES)
        {
            // spurious reply message
            return return_with_error(Defs::ERROR_OUT_OF_ORDER);
        }
        input()->consistCount = payload[2];
        if (payload.size() >= 11) {
            input()->consistIndex = payload[3];
            input()->flags = payload[4];
            input()->dst = data_to_node_id(&payload[5]);
        } else {
            input()->consistIndex = 0xff;
            input()->flags = 0xff;
            input()->dst = 0;
        }
        input()->replyCause = 0;
        return return_ok();
    }

    void listen_reply(Buffer<GenMessage> *msg)
    {
        AutoReleaseBuffer<GenMessage> rb(msg);
        if (!iface()->matching_node(msg->data()->src, NodeHandle(dst_)))
        {
            return;
        }
        const Payload &p = msg->data()->payload;
        if (p.size() < 1)
            return;
        switch (p[0])
        {
            case TractionDefs::REQ_SET_SPEED:
            {
                Velocity v;
                // speed get and set have the same signature for what we care
                if (TractionDefs::speed_get_parse_last(p, &v))
                {
                    lastSetSpeed_ = v;
                    estopActive_ = false;
                    if (updateCallback_)
                    {
                        updateCallback_(-1);
                    }
                }
                return;
            }
            case TractionDefs::REQ_EMERGENCY_STOP:
            {
                estopActive_ = true;
                if (updateCallback_)
                {
                    updateCallback_(-1);
                }
                return;
            }
            case TractionDefs::REQ_SET_FN:
            {
                uint16_t v;
                unsigned num;
                // function get and set have the same signature
                if (TractionDefs::fn_get_parse(p, &v, &num))
                {
                    lastKnownFn_[num] = v;
                    if (updateCallback_)
                    {
                        updateCallback_(num);
                    }
                }
                return;
            }
        }
    }

    /** Allocates (synchronously) an outgoing openlcb buffer with traction
     * request MTI and the given payload and sends off the message to the bus
     * for dst_. */
    void send_traction_message(const Payload &payload)
    {
        HASSERT(dst_ != 0);
        auto *b = iface()->addressed_message_write_flow()->alloc();
        b->data()->reset(Defs::MTI_TRACTION_CONTROL_COMMAND, node_->node_id(),
            NodeHandle(dst_), payload);
        iface()->addressed_message_write_flow()->send(b);
    }

    void set_listening()
    {
        listenConsist_ = true;
        iface()->dispatcher()->register_handler(&listenReplyHandler_,
            Defs::MTI_TRACTION_CONTROL_COMMAND, Defs::MTI_EXACT);
    }

    void clear_listening()
    {
        listenConsist_ = false;
        iface()->dispatcher()->unregister_handler(&listenReplyHandler_,
            Defs::MTI_TRACTION_CONTROL_COMMAND, Defs::MTI_EXACT);
    }

    void set_assigned()
    {
        iface()->dispatcher()->register_handler(&speedReplyHandler_, Defs::MTI_TRACTION_CONTROL_REPLY, Defs::MTI_EXACT);
        assigned_ = true;
    }

    void clear_assigned()
    {
        if (!assigned_)
        {
            return;
        }
        assigned_ = false;
        iface()->dispatcher()->unregister_handler(&speedReplyHandler_, Defs::MTI_TRACTION_CONTROL_REPLY, Defs::MTI_EXACT);
    }

    void clear_cache()
    {
        lastSetSpeed_ = nan_to_speed();
        estopActive_ = false;
        lastKnownFn_.clear();
    }

    TractionThrottleInput *input()
    {
        return message()->data();
    }

    If *iface()
    {
        // We know that the service pointer is the node's interface from the
        // constructor.
        return static_cast<If *>(service());
    }

    MessageHandler::GenericHandler speedReplyHandler_{
        this, &TractionThrottle::speed_reply};
    MessageHandler::GenericHandler listenReplyHandler_{
        this, &TractionThrottle::listen_reply};
    /// How many speed/fn query requests I have sent off to the train node that
    /// have not yet seen a reply.
    unsigned pendingQueries_{0};
    StateFlowTimer timer_{this};
    /// True if the assign controller has returned positive.
    bool assigned_{false};
    /// True if we also have a consist link with the assigned loco.
    bool listenConsist_{false};
    /// keep track if E-Stop is active
    bool estopActive_{false};
    NodeID dst_;
    Node *node_;
    /// Helper class for stateful query/return flows.
    TractionResponseHandler handler_{iface(), node_};
    /// Function to call when a different controller updates the train.
    std::function<void(int fn)> updateCallback_;
    /// Cache: Velocity value that we last commanded to the train.
    SpeedType lastSetSpeed_;
    /// Cache: all known function values.
    std::map<uint32_t, uint16_t> lastKnownFn_;
};

} // namespace openlcb

#endif // _OPENLCB_TRACTIONTHROTTLE_HXX_
