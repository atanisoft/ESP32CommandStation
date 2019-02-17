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
 * \file NodeHandlerMap.hxx
 *
 * A reusable data structure for registering global or per-node handlers
 * parametrized by an ID.
 *
 * @author Balazs Racz
 * @date 25 Jan 2014
 */

#ifndef _UTILS_NODEHANDLERMAP_HXX_
#define _UTILS_NODEHANDLERMAP_HXX_

#include <stdint.h>
#include <utility>

#include "utils/StlMap.hxx"
#include "utils/LinearMap.hxx"
#include "utils/SysMap.hxx"


#if UINTPTR_MAX > UINT32_MAX
#define NODEHANDLER_USE_PAIR
#endif


/** A map that allows registration and lookup or per-node handler of a
 *  particular message ID.
 *
 *  Regular handlers are registered for a node and messsageID pair.
 *
 *  The map supports registering handlers for a message ID globally by
 *  supplying nullptr as the node. These will be returned for nodes that have
 *  no specific handler for that particular message ID.
 */
class NodeHandlerMapBase
{
private:
#ifdef NODEHANDLER_USE_PAIR
    /// Standard payload type.
    typedef std::pair<void*, uint32_t> key_type;
#else
    /// Compacted payload type.
    typedef uint64_t key_type;
#endif
    /// Generic handler type that we'll keep.
    typedef void* value_type;
    /// Type of the storage object.
    typedef StlMap<key_type, value_type> map_type;

public:
    NodeHandlerMapBase()
    {
    }

    /// Creates a map with @param entries capacity.
    NodeHandlerMapBase(size_t entries) : entries_(entries)
    {
    }

    /** Inserts a handler into the map.
     * @param node is the node for which to register the handler.
     * @param id is the message ID for which to register.
     * @param value is the handler to register.
     */
    void insert(void* node, uint32_t id, void* value)
    {
        entries_[make_key(node, id)] = value;
    }

    /// Removes a handlerfrom this map.
    ///
    /// @param node is the node to unregister handler for (nullptr for all
    /// nodes)
    /// @param id is the message ID for which to unregister for.
    /// @param value is the pointer to the handler.
    void erase(void *node, uint32_t id, void *value)
    {
        auto it = entries_.find(make_key(node, id));
        if (it == entries_.end()) return;
        if (it->second == value) {
            entries_.erase(it);
        }
    }
    
    /** Finds a handler for a particular node and particular messageID.
     * @return a handler or nullptr if no node-specific and no global handler
     * for that ID is found.
     * @param node what to look up for
     * @param id is the message ID to look up */
    void* lookup(void* node, uint32_t id)
    {
        auto it = entries_.find(make_key(node, id));
        if (it == entries_.end())
        {
            it = entries_.find(make_key(nullptr, id));
        }
        if (it == entries_.end())
        {
            return nullptr;
        }
        else
        {
            return it->second;
        }
    }

    /// Iterator type.
    typedef typename map_type::Iterator iterator;
    /// @return begin iterator
    iterator begin()
    {
        return entries_.begin();
    }

    /// @return end iterator
    iterator end()
    {
        return entries_.end();
    }

    /// Decodes a compact key into a pair. @param key is what to
    /// decode. @return decoded key (classic iterator pair value).
    static pair<void *, uint32_t> read_key(key_type key)
    {
#ifdef NODEHANDLER_USE_PAIR
        return key;
#else
        uint32_t id = key & 0xFFFFFFFFU;
        uint32_t n = key >> 32;
        return std::make_pair(reinterpret_cast<void *>(n), id);
#endif
    }

private:
    /// Combines the node pointer and the message ID into a lookup key.
    key_type make_key(void* node, uint32_t id)
    {
#ifdef NODEHANDLER_USE_PAIR
        return std::make_pair(node, id);
#else
        uint64_t key = reinterpret_cast<uint32_t>(node);
        key <<= 32;
        key |= id;
        return key;
#endif
    }

    /// The actual storage object.
    map_type entries_;
};

/** A type-safe map that allows registration and lookup or per-node handler of
 *  a particular message ID. see @ref NodeHandlerMapBase for details. */
template <class Node, class Handler>
class TypedNodeHandlerMap : private NodeHandlerMapBase
{
public:
    TypedNodeHandlerMap()
    {
    }

    /// @param entries is the number of maximum entries in this map (will
    /// statically allocate).
    TypedNodeHandlerMap(size_t entries) : NodeHandlerMapBase(entries)
    {
    }

    /** Inserts a handler into the map.
     * @param node is the node for which to register the handler.
     * @param id is the message ID for which to register.
     * @param value is the handler to register.
     */
    void insert(Node* node, uint32_t id, Handler* handler)
    {
        NodeHandlerMapBase::insert(node, id, handler);
    }

    /** Removes a handler from the map. If the mapping does not currently point
     * to that handler, does nothing.
     *
     * @param node is the node for which to unregister the handler.
     * @param id is the message ID for which to unregister.
     * @param value is the handler to unregister.
     */
    void erase(Node* node, uint32_t id, Handler* handler)
    {
        NodeHandlerMapBase::erase(node, id, handler);
    }

    /** Finds a handler for a particular node and particular messageID.
     * @param node is the node that received the message
     * @param id is the message's ID
     * @return a handler or nullptr if no node-specific and no global handler
     * for that ID is found. */
    Handler* lookup(Node* node, uint32_t id)
    {
        return static_cast<Handler*>(NodeHandlerMapBase::lookup(node, id));
    }

    /// Type-safe iterator for NodeHandlerMap.
    class iterator {
    public:
        /// @param i untyped iterator
        iterator(NodeHandlerMapBase::iterator i)
            : impl_(i) {}
        
        /// advance
        void operator++() {
            ++impl_;
        }

        /// @return comparison
        bool operator!=(const iterator& o) {
            return impl_ != o.impl_;
        }

        /// Dereference. @return pair
        std::pair<std::pair<Node*, uint32_t>, Handler*> operator*() {
            auto p = NodeHandlerMapBase::read_key(impl_->first);
            return std::make_pair(std::make_pair((Node*) p.first, p.second),
                                  (Handler*)impl_->second);
        }

    private:
        /// untyped iterator
        NodeHandlerMapBase::iterator impl_;
    };

    /// @return begin iterator
    iterator begin() {
        return iterator(NodeHandlerMapBase::begin());
    }

    /// @return end iterator
    iterator end() {
        return iterator(NodeHandlerMapBase::end());
    }
};

#endif // _UTILS_NODEHANDLERMAP_HXX_
