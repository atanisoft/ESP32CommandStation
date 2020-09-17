/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file DefaultNodeRegistry.hxx
 *
 * Default implementations for data structures keyed by virtual nodes.
 *
 * @author Balazs Racz
 * @date 12 Sep 2020
 */

#ifndef _OPENLCB_DEFAULTNODEREGISTRY_HXX_
#define _OPENLCB_DEFAULTNODEREGISTRY_HXX_

#include <set>

#include "openlcb/NodeRegistry.hxx"

namespace openlcb
{

class Node;

class DefaultNodeRegistry : public NodeRegistry
{
public:
    /// Adds a node to the list of registered nodes.
    /// @param node a virtual node.
    void register_node(openlcb::Node *node) override
    {
        nodes_.insert(node);
    }

    /// Removes a node from the list of registered nodes.
    /// @param node a virtual node.
    void unregister_node(openlcb::Node *node) override
    {
        nodes_.erase(node);
    }

    /// Checks if a node is registered.
    /// @param node a virtual node.
    /// @return true if this node has been registered.
    bool is_node_registered(openlcb::Node *node) override
    {
        return nodes_.find(node) != nodes_.end();
    }

private:
    std::set<Node *> nodes_;
};

} // namespace openlcb

#endif // _OPENLCB_DEFAULTNODEREGISTRY_HXX_
