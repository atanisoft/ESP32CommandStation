/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file NodeBrowser.hxx
 * Module that allows monitoring the list of nodes on the network.
 *
 * @author Balazs Racz
 * @date 8 March 2019
 */

#ifndef _OPENLCB_NODEBROWSER_HXX_
#define _OPENLCB_NODEBROWSER_HXX_

#include <functional>

#include "openlcb/If.hxx"

namespace openlcb
{

/// This class helps establishing a list of all (live) nodes on the
/// network. Useful for configuration tools.
class NodeBrowser
{
public:
    /// Function prototype for the callback. This function will be called on
    /// the interface's executor.
    /// @param n the node id of the discovered remote node.
    typedef std::function<void(NodeID n)> CallbackFunction;

    /// Constructor.
    /// @param node is the *current* node, from which we can send traffic to the
    /// bus.
    /// @param cb will be called for each newly arriving node, and for each
    /// existing node after the refresh command is executed.
    NodeBrowser(Node *node, CallbackFunction cb);

    /// Destructor. After calling it is guaranteed not to receive any more
    /// callbacks.
    ~NodeBrowser();

    /// Requests a pong from every live node. This function will return
    /// immediately, then callbacks will be called for each reply that arrives.
    void refresh();

private:
    /// Helper class to register in the dispatcher. Incomng response messages
    /// will be routed to this object.
    class VerifiedHandler : public MessageHandler
    {
    public:
        /// @param parent is the NodeBrowser that owns *this
        VerifiedHandler(NodeBrowser *parent);
        /// @param b incoming message
        void send(Buffer<GenMessage> *b, unsigned) override;

    private:
        /// NodeBrowser that owns *this.
        NodeBrowser *parent_;
    };
    friend class VerifiedHandler;

    /// Register with the interface for messages we want to listen to.
    void register_callbacks();
    /// Remove callbacks from the interface.
    void unregister_callbacks();
    /// Me-node.
    Node *node_;
    /// Callback registerd in the interface.
    VerifiedHandler handler_ {this};
    /// Client callback for live or new nodes.
    CallbackFunction callback_;
};

} // namespace openlcb

#endif // _OPENLCB_NODEBROWSER_HXX_
