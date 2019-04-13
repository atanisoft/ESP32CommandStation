/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file IfTcp.cxx
 *
 * OpenLCB interface implementation for native TCP connection.
 *
 * @author Balazs Racz
 * @date 16 Apr 2017
 */

#include "openlcb/IfTcp.hxx"
#include "openlcb/IfImpl.hxx"
#include "openlcb/IfTcpImpl.hxx"

namespace openlcb
{

void IfTcp::delete_local_node(Node *node)
{
    remove_local_node_from_map(node);
}

void IfTcp::add_owned_flow(Executable *e)
{
    ownedFlows_.emplace_back(e);
}

void IfTcp::delete_owned_flow(Destructable *d)
{
    for (auto it = ownedFlows_.begin(); it != ownedFlows_.end(); ++it)
    {
        if (it->get() == d)
        {
            ownedFlows_.erase(it);
            return;
        }
    }
    LOG_ERROR("Deleting nonexistent owned flow.");
}

bool IfTcp::matching_node(NodeHandle expected, NodeHandle actual)
{
    if (expected.id && actual.id)
    {
        return expected.id == actual.id;
    }
    // Cannot reconcile.
    LOG(VERBOSE, "Cannot reconcile expected and actual NodeHandles for "
                 "equality testing.");
    return false;
}

void IfTcp::add_network_fd(int fd, Notifiable *on_error)
{
    // What are the cases we need to support:
    //
    // - the remote server closing the socket. on_error shall be called and the
    //   port destructed.
    //
    // - the IfTcp being destructed (presumably not on the main executor). We
    //   need to unregister the port, shutdown the socket, flush the data, then
    //   delete this. The destructor of HubDeviceSelect triggers the barrier
    //   callback inline.
    //
    // The contract with HubDeviceSelect is that by the time the on_error
    // notifiable is called, the HubDeviceSelect has been unregistered,
    // flushed, and can be deleted, even if we are on the main executor.
    //
    struct RemotePort : public Executable
    {
        RemotePort(IfTcp *parent, Notifiable *on_error)
            : parent_(parent)
            , onError_(on_error)
        {
        }
        ~RemotePort()
        {
            // auto* p = port_->write_port();
            // LOG_ERROR("remoteport::delete %p %p", p, this);
            deleting_ = true;
            port_.reset();
            // LOG_ERROR("remoteport::delete done %p", p);
        }
        IfTcp *parent_;
        std::unique_ptr<TcpHubDeviceSelect> port_;
        Notifiable *onError_;
        bool deleting_{false};
        void notify() override
        {
            // auto* p = port_->write_port();
            // LOG(VERBOSE, "remoteport::notify %d %p %p", deleting_, p, this);
            if (onError_)
            {
                onError_->notify();
                onError_ = nullptr;
            }
            if (!deleting_) // avoids duplicate destruction.
            {
                parent_->executor()->add(this);
            }
        }

        void run() override
        {
            if (deleting_ || !port_)
            {
                return;
            }
            if (!port_->write_done())
            {
                // yield
                parent_->executor()->add(this);
                return;
            }
            parent_->delete_owned_flow(this);
        }
    };
    RemotePort *p = new RemotePort(this, on_error);
    p->port_.reset(new TcpHubDeviceSelect(device_, fd, p));
    add_owned_flow(p);
}

/// Component that drops incoming addressed messages that are not destined for
/// a local node.
class LocalMessageFilter : public MessageHandler, public Destructable
{
public:
    LocalMessageFilter(If *iface)
        : iface_(iface)
    {
    }

    void send(Buffer<GenMessage> *b, unsigned prio) override
    {
        auto id = b->data()->dst.id;
        if (id)
        {
            auto *dst = iface_->lookup_local_node(id);
            if (dst)
            {
                b->data()->dstNode = dst;
            }
            else
            {
                b->unref();
                return;
            }
        }
        iface_->dispatcher()->send(b, prio);
    }

private:
    If *iface_;
};

IfTcp::IfTcp(NodeID gateway_node_id, HubFlow *device, int local_nodes_count)
    : If(device->service()->executor(), local_nodes_count)
    , device_(device)
{
    add_owned_flow(new VerifyNodeIdHandler(this));
    seq_ = new ClockBaseSequenceNumberGenerator;
    add_owned_flow(seq_);
    auto filter = new LocalMessageFilter(this);
    add_owned_flow(filter);
    recvFlow_ = new TcpRecvFlow(filter);
    add_owned_flow(recvFlow_);
    sendFlow_ = new TcpSendFlow(this, gateway_node_id, device, recvFlow_, seq_);
    add_owned_flow(sendFlow_);
    globalWriteFlow_ = sendFlow_;
    addressedWriteFlow_ = sendFlow_;
    device_->register_port(recvFlow_);
}

IfTcp::~IfTcp()
{
    device_->unregister_port(recvFlow_);
    while (!ownedFlows_.empty())
    {
        ownedFlows_.resize(ownedFlows_.size() - 1);
    }
}

} // namespace openlcb
