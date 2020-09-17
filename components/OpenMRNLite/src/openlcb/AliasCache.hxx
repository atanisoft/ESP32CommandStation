/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file AliasCache.hxx
 * This file provides an Alias Caching mechanism.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#ifndef _OPENLCB_ALIASCACHE_HXX_
#define _OPENLCB_ALIASCACHE_HXX_

#include "openlcb/Defs.hxx"
#include "utils/Map.hxx"
#include "utils/SortedListMap.hxx"
#include "utils/macros.h"

namespace openlcb
{

/** Cache of alias to node id mappings.  The cache is limited to a fixed number
 * of entries at construction.  All the memory for the cache will be allocated
 * at construction time, limited by the maximum number of entries.  Note, there
 * is no mutual exclusion locking mechanism built into this class.  Mutual
 * exclusion must be handled by the user as needed.
 *
 * This data structure is sometimes used with very large entry count
 * (hundreds), therefore we must be careful about memory efficiency!
 *
 * Theory of operation:
 *
 * The data structure has three access patterns:
 *   - lookup of alias -> ID
 *   - lookup of ID -> alias
 *   - eviction of oldest entry from the cache
 *
 * We have three data structures to match these use-cases:
 *
 * The struct Metadata stores the actual NodeID and NodeAlias values. This is
 * laid out in the pre-allocated C array `pool`. A freeList shows where unused
 * entries are.
 *
 * To support the eviction of oldest entry, an LRU doubly-linked list is
 * created in these Metadata entries. The links are represented by indexes into
 * the `pool` array.
 *
 * Indexes into the `pool` are encapsulated into the PoolIdx struct to make
 * them a unique C++ type. This is needed for template disambiguation. We also
 * have a dereference function on PoolIdx that turns it into a Metadata
 * pointer.
 *
 * To support lookup by alias, we have a SortedListSet which contains all used
 * indexes as PoolIdx, sorted by the alias property in the respective entry in
 * `pool`. This is achieved by a custom comparator that dereferences the
 * PoolIdx object and fetches the alias from the Metadata struct. The sorted
 * vector is maintained using the SortedListSet<> template, and takes a total
 * of only 2 bytes per entry.
 *
 * A similar sorted vector is kept sorted by the NodeID values. This also takes
 * only 2 bytes per entry.
 */
class AliasCache
{
public:
    /** Constructor.
     * @param seed starting seed for generation of aliases
     * @param entries maximum number of entries in this cache
     * @param remove_callback callback to call when we remove a mapping from
     *        the cache however it will not be called in the remove() method
     * @param context context pointer to pass to remove_callback
     */
    AliasCache(NodeID seed, size_t _entries,
        void (*remove_callback)(NodeID id, NodeAlias alias, void *) = NULL,
        void *context = NULL)
        : pool(new Metadata[_entries])
        , aliasMap(this)
        , idMap(this)
        , seed(seed)
        , entries(_entries)
        , removeCallback(remove_callback)
        , context(context)
    {
        aliasMap.reserve(_entries);
        idMap.reserve(_entries);
        clear();
    }

    /// Sentinel entry for empty lists.
    static constexpr uint16_t NONE_ENTRY = 0xFFFFu;

    /** Reinitializes the entire map. */
    void clear();

    /** Add an alias to an alias cache.
     * @param id 48-bit NMRAnet Node ID to associate alias with
     * @param alias 12-bit alias associated with Node ID
     */
    void add(NodeID id, NodeAlias alias);
    
    /** Remove an alias from an alias cache.  This method does not call the
     * remove_callback method passed in at construction since it is a
     * deliberate call not requiring notification.
     * @param alias 12-bit alias associated with Node ID
     */
    void remove(NodeAlias alias);

    /** Lookup a node's alias based on its Node ID.
     * @param id Node ID to look for
     * @return alias that matches the Node ID, else 0 if not found
     */
    NodeAlias lookup(NodeID id);

    /** Lookup a node's ID based on its alias.
     * @param alias alias to look for
     * @return Node ID that matches the alias, else 0 if not found
     */
    NodeID lookup(NodeAlias alias);

    /** Call the given callback function once for each alias tracked.  The order
     * will be in last "touched" order.
     * @param callback method to call
     * @param context context pointer to pass to callback
     */
    void for_each(void (*callback)(void*, NodeID, NodeAlias), void *context);

    /** Returns the total number of aliases that can be cached. */
    size_t size()
    {
        return entries;
    }

    /** Retrieves an entry by index. Allows stable iteration in the face of
     * changes.
     * @param entry is between 0 and size() - 1.
     * @param node will be filled with the node ID. May be null.
     * @param alias will be filled with the alias. May be null.
     * @return true if the entry is valid, and node and alias were filled, otherwise false if the entry is not allocated.
     */
    bool retrieve(unsigned entry, NodeID* node, NodeAlias* alias);

    /** Generate a 12-bit pseudo-random alias for a given alias cache.
     * @return pseudo-random 12-bit alias, an alias of zero is invalid
     */
    NodeAlias generate();

    /** Default destructor */
    ~AliasCache()
    {
        delete [] pool;
    }

    /** Visible for testing. Check internal consistency. */
    int check_consistency();

private:
    enum
    {
        /** marks an unused mapping */
        UNUSED_MASK = 0x10000000
    };

    struct Metadata;
    class PoolIdx;
    friend class PoolIdx;

    /// Encapsulation of a pointer into the pool array.
    class PoolIdx
    {
    public:
        /// Constructor. Sets the pointer to invalid.
        PoolIdx()
            : idx_(NONE_ENTRY)
        {
        }
        /// @return true if this entry does not point anywhere.
        bool empty()
        {
            return idx_ == NONE_ENTRY;
        }
        /// Indexes the pool of the AliasCache.
        uint16_t idx_;
        /// Dereferences a pool index as if it was a pointer.
        /// @param parent the AliasCache whose pool to index.
        /// @return referenced Metadata pointer.
        Metadata *deref(AliasCache *parent)
        {
            DASSERT(idx_ != NONE_ENTRY);
            return parent->pool + idx_;
        }
    };

    /** Interesting information about a given cache entry. */
    struct Metadata
    {
        /// Sets the node ID field.
        /// @param id the node ID to set.
        void set_node_id(NodeID id)
        {
            nodeIdLow_ = id & 0xFFFFFFFFu;
            nodeIdHigh_ = (id >> 32) & 0xFFFFu;
        }

        /// @return the node ID field.
        NodeID get_node_id()
        {
            uint64_t h = nodeIdHigh_;
            h <<= 32;
            h |= nodeIdLow_;
            return h;
        }

        /// OpenLCB Node ID low 32 bits.
        uint32_t nodeIdLow_ = 0;
        /// OpenLCB Node ID high 16 bits.
        uint16_t nodeIdHigh_ = 0;
        /// OpenLCB-CAN alias
        NodeAlias alias_ = 0;
        /// Index of next-newer entry according to the LRU linked list.
        PoolIdx newer_;
        /// Index of next-older entry according to the LRU linked list.
        PoolIdx older_;
    };

    /** pointer to allocated Metadata pool */
    Metadata *pool;

    /// Comparator object comparing the aliases stored in the pool.
    class AliasComparator
    {
    public:
        /// Constructor
        /// @param parent owning AliasCache.
        AliasComparator(AliasCache *parent)
            : parent_(parent)
        {
        }

        /// Less-than action.
        /// @param e left hand side
        /// @param alias right hand side
        bool operator()(PoolIdx e, uint16_t alias) const
        {
            return e.deref(parent_)->alias_ < alias;
        }

        /// Less-than action.
        /// @param alias left hand side
        /// @param e right hand side
        bool operator()(uint16_t alias, PoolIdx e) const
        {
            return alias < e.deref(parent_)->alias_;
        }

        /// Less-than action.
        /// @param a left hand side
        /// @param b right hand side
        bool operator()(PoolIdx a, PoolIdx b) const
        {
            return a.deref(parent_)->alias_ < b.deref(parent_)->alias_;
        }

    private:
        /// AliasCache whose pool we are indexing into.
        AliasCache *parent_;
    };

    /// Comparator object comparing the aliases stored in the pool.
    class IdComparator
    {
    public:
        /// Constructor
        /// @param parent owning AliasCache.
        IdComparator(AliasCache *parent)
            : parent_(parent)
        {
        }

        /// Less-than action.
        /// @param e left hand side
        /// @param id right hand side
        bool operator()(PoolIdx e, NodeID id) const
        {
            return e.deref(parent_)->get_node_id() < id;
        }

        /// Less-than action.
        /// @param id left hand side
        /// @param e right hand side
        bool operator()(NodeID id, PoolIdx e) const
        {
            return id < e.deref(parent_)->get_node_id();
        }

        /// Less-than action.
        /// @param a left hand side
        /// @param b right hand side
        bool operator()(PoolIdx a, PoolIdx b) const
        {
            return a.deref(parent_)->get_node_id() <
                b.deref(parent_)->get_node_id();
        }

    private:
        /// AliasCache whose pool we are indexing into.
        AliasCache *parent_;
    };

    /** Short hand for the alias Map type */
    typedef SortedListSet<PoolIdx, AliasComparator> AliasMap;

    /** Short hand for the ID Map type */
    typedef SortedListSet<PoolIdx, IdComparator> IdMap;

    /** Map of alias to corresponding Metadata */
    AliasMap aliasMap;
    
    /** Map of Node ID to corresponding Metadata */
    IdMap idMap;

    /** list of unused mapping entries (index into pool) */
    PoolIdx freeList;

    /** oldest untouched entry (index into pool) */
    PoolIdx oldest;

    /** newest, most recently touched entry (index into pool) */
    PoolIdx newest;

    /** Seed for the generation of the next alias */
    NodeID seed;

    /** How many metadata entries have we allocated. */
    size_t entries;

    /** callback function to be used when we remove an entry from the cache */
    void (*removeCallback)(NodeID id, NodeAlias alias, void *);
    
    /** context pointer to pass in with remove_callback */
    void *context;

    /** Update the time stamp for a given entry.
     * @param  metadata metadata associated with the entry
     */
    void touch(Metadata* metadata);

    DISALLOW_COPY_AND_ASSIGN(AliasCache);
};

} /* namespace openlcb */

#endif  // _OPENLCB_ALIASCACHE_HXX_
