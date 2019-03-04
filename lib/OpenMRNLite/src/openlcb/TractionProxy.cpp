/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file TractionProxy.cxx
 *
 * Service for automatically creating train nodes from addresses punched into
 * the throttle.
 *
 * @author Balazs Racz
 * @date 1 Feb 2015
 */

#include "openlcb/TractionProxy.hxx"

#include "openlcb/EventHandlerTemplates.hxx"
#include "dcc/Loco.hxx"
#include "openlcb/If.hxx"

namespace openlcb
{

/** @todo(balazs.racz) this is a kludgy interface but allows the linux traction
 * proxy implementation to override and create test trains. */
Node *allocate_train_node(uint8_t system, uint8_t addr_hi, uint8_t addr_lo,
    TrainService* traction_service)
    __attribute__((weak));

Node *allocate_train_node(uint8_t system, uint8_t addr_hi, uint8_t addr_lo, TrainService* traction_service)
{
    TrainImpl *impl = nullptr;
    if (system == TractionDefs::PROXYTYPE_MARKLIN_DIGITAL)
    {
        impl = new dcc::MMNewTrain(dcc::MMAddress(addr_lo));
    }
    else if (system == TractionDefs::PROXYTYPE_DCC)
    {
        /// @TODO(balazs.racz) take into account the speed step setting.
        if (addr_hi == 0 && addr_lo < 128)
        {
            impl = new dcc::Dcc28Train(dcc::DccShortAddress(addr_lo));
        }
        else
        {
            impl = new dcc::Dcc28Train(dcc::DccLongAddress(
                (static_cast<uint16_t>(addr_hi) << 8) | addr_lo));
        }
    }
    if (!impl) return nullptr;
    TrainNode *train_node = new TrainNodeForProxy(traction_service, impl);
    return train_node;
}

/// PImpl Implementation structure for the Traction Proxy service. Owns all
/// implementation flows, which are important for the correct function of the
/// service, but are not needed to be visible on the API.
struct TractionProxyService::Impl
{
public:
    Impl(TrainService *train_service, Node *proxy_node)
        : trainService_(train_service)
        , proxyNode_(proxy_node)
        , proxyEventProducer_(proxyNode_)
        , handlerFlow_(this)
    {
    }

    TrainService *traction_service()
    {
        return trainService_;
    }

    If *iface()
    {
        return traction_service()->iface();
    }

    Node *node()
    {
        return proxyNode_;
    }

    /// State flow handling incoming OpenLCB messages of MTI == Traction proxy
    /// Request.
    class ProxyRequestFlow : public IncomingMessageStateFlow
    {
    public:
        ProxyRequestFlow(Impl *impl)
            : IncomingMessageStateFlow(impl->iface())
            , impl_(impl)
            , response_(nullptr)
        {
            iface()->dispatcher()->register_handler(
                this, Defs::MTI_TRACTION_PROXY_COMMAND, 0xffff);
        }

        ~ProxyRequestFlow()
        {
            iface()->dispatcher()->unregister_handler(
                this, Defs::MTI_TRACTION_PROXY_COMMAND, 0xffff);
        }

    protected:
        Action entry() OVERRIDE
        {
            if (!nmsg()->dstNode)
            {
                LOG(VERBOSE, "Traction proxy message for unknown node.");
                return release_and_exit();
            }
            if (nmsg()->dstNode != impl_->node())
            {
                // Wrong node.
                return release_and_exit();
            }
            if (response_)
            {
                return call_immediately(STATE(handle_cmd));
            }
            else
            {
                return allocate_and_call(
                    impl_->iface()->addressed_message_write_flow(),
                    STATE(handle_cmd));
            }
        }

        Action handle_cmd()
        {
            if (!response_)
            {
                response_ = get_allocation_result(
                    impl_->iface()->addressed_message_write_flow());
            }
            // No command byte?
            if (size() < 1)
            {
                LOG(VERBOSE, "Traction proxy message with no command byte.");
                return reject_permanent();
            }
            uint8_t cmd = payload()[0];
            switch (cmd)
            {
                case TractionDefs::PROXYREQ_MANAGE:
                {
                    return call_immediately(STATE(handle_manage));
                }
                case TractionDefs::PROXYREQ_ALLOCATE:
                {
                    return call_immediately(STATE(handle_allocate));
                }
                default:
                {
                    LOG(VERBOSE, "Unknown traction proxy command %x.", cmd);
                    return reject_permanent();
                }
            }
        }

        Action handle_manage()
        {
            if (size() < 2)
            {
                return reject_permanent();
            }
            uint8_t cmd = payload()[1];
            switch (cmd)
            {
                case TractionDefs::PROXYREQ_MANAGE_RESERVE:
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
                    Payload p;
                    p.push_back(TractionDefs::PROXYRESP_MANAGE);
                    p.push_back(TractionDefs::PROXYRESP_MANAGE_RESERVE_REPLY);
                    p.push_back(code);
                    response_->data()->reset(Defs::MTI_TRACTION_PROXY_REPLY,
                                             nmsg()->dstNode->node_id(),
                                             nmsg()->src, p);
                    impl_->iface()->addressed_message_write_flow()->send(
                        response_);
                    response_ = nullptr;
                    return release_and_exit();
                }
                case TractionDefs::PROXYREQ_MANAGE_RELEASE:
                {
                    reserved_ = 0;
                    return release_and_exit();
                }
                default:
                    LOG(VERBOSE, "Unknown Traction proxy manage subcommand %x",
                        cmd);
                    return reject_permanent();
            }
        }

        Action handle_allocate()
        {
            if (size() < 5)
            {
                LOG(VERBOSE, "proxy allocate with too short message or not DCC "
                             "legacy technology ID.");
                return reject_permanent();
            }
            uint8_t system = payload()[1];
            uint8_t addr_hi = payload()[2];
            uint8_t addr_lo = payload()[3];
            Node *train_node =
                allocate_train_node(system, addr_hi, addr_lo, impl_->traction_service());

            if (!train_node)
            {
                LOG(VERBOSE, "proxy allocate could not create impl.");
                return reject_permanent();
            }
            Payload b;
            b.push_back(TractionDefs::PROXYRESP_ALLOCATE);
            b.push_back(0);
            b.push_back(system);
            b.push_back(addr_hi);
            b.push_back(addr_lo);
            auto node_id = train_node->node_id();
            for (int i = 56; i >= 0; i -= 8)
            {
                b.push_back((node_id >> i) & 0xff);
            }
            response_->data()->reset(Defs::MTI_TRACTION_PROXY_REPLY,
                                     nmsg()->dstNode->node_id(), nmsg()->src,
                                     b);
            impl_->iface()->addressed_message_write_flow()->send(response_);
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
        Action reject_permanent()
        {
            // An alternative would be to send TERMINATE_DUE_TO_ERROR here.
            response_->data()->reset(
                Defs::MTI_OPTIONAL_INTERACTION_REJECTED,
                nmsg()->dstNode->node_id(), nmsg()->src,
                error_to_buffer(Defs::ERROR_PERMANENT, nmsg()->mti));
            impl_->iface()->addressed_message_write_flow()->send(response_);
            response_ = nullptr;
            return release_and_exit();
        }

    private:
        Impl *impl_;
        Buffer<GenMessage> *response_;
        unsigned reserved_ : 1;
    };

    TrainService *trainService_;
    Node *proxyNode_;
    FixedEventProducer<TractionDefs::IS_PROXY_EVENT> proxyEventProducer_;
    ProxyRequestFlow handlerFlow_;
};

TractionProxyService::TractionProxyService(TrainService *train_service,
                                           Node *proxy_node)
    : Service(train_service->iface()->executor())
{
    impl_ = new Impl(train_service, proxy_node);
}

TractionProxyService::~TractionProxyService()
{
    delete impl_;
}

} // namespace openlcb
