/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file DefaultNode.hxx
 *
 * Default AsyncNode implementation for a fat virtual node.
 *
 * @author Balazs Racz
 * @date 7 December 2013
 */

#ifndef _OPENLCB_DEFAULTNODE_HXX_
#define _OPENLCB_DEFAULTNODE_HXX_

#include "openlcb/Node.hxx"

namespace openlcb
{

/// Trivial implementation of a virtual Node. Stores all dynamic information in
/// class member variables.
class DefaultNode: public Node
{
public:
    DefaultNode(If* iface, NodeID node_id);
    virtual ~DefaultNode();

    NodeID node_id() OVERRIDE
    {
        return nodeId_;
    }
    If* iface() OVERRIDE
    {
        return iface_;
    }
    bool is_initialized() OVERRIDE
    {
        return isInitialized_;
    }

    // Sets the initialized status to true.
    void set_initialized() OVERRIDE
    {
        isInitialized_ = 1;
    }

    // Used for restarting the stack.
    void clear_initialized() OVERRIDE
    {
        isInitialized_ = 0;
    }

private:
    /** 48-bit node identifier of this node. */
    NodeID nodeId_ : 48;
    /** 1 if the node has reached initialized state. */
    unsigned isInitialized_ : 1;
    /** Interface this node is bound to. */
    If* iface_;
};

} // namespace openlcb

#endif // _OPENLCB_DEFAULTNODE_HXX_
