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
#include "utils/macros.h"
#include "utils/Map.hxx"
#include "utils/RBTree.hxx"

namespace openlcb
{

/** Cache of alias to node id mappings.  The cache is limited to a fixed number
 * of entries at construction.  All the memory for the cache will be allocated
 * at construction time, limited by the maximum number of entries.  Note, there
 * is no mutual exclusion locking mechanism built into this class.  Mutual
 * exclusion must be handled by the user as needed.
 *
 * @todo the class uses RBTree, consider a version that is a linear search for
 * a small number of entries.
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
        : pool(new Metadata[_entries]),
          freeList(NULL),
          aliasMap(_entries),
          idMap(_entries),
          oldest(NULL),
          newest(NULL),
          seed(seed),
          entries(_entries),
          removeCallback(remove_callback),
          context(context)
    {
        clear();
    }

    /** This NodeID will be used for reserved but unused local aliases. */
    static const NodeID RESERVED_ALIAS_NODE_ID;

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
     * @param aliad will be filles with the alias. May be null.
     * @return true if the entry is valid, and node and alias were filled, otherwise false if the entry is not allocated.
     */
    bool retrieve(unsigned entry, NodeID* node, NodeAlias* alias);

    /** Generate a 12-bit pseudo-random alias for a givin alias cache.
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

    /** Interesting information about a given cache entry. */
    struct Metadata
    {
        NodeID id = 0; /**< 48-bit NMRAnet Node ID */
        NodeAlias alias = 0; /**< NMRAnet alias */
        long long timestamp; /**< time stamp of last usage */
        union
        {
            Metadata *prev; /**< unused */
            Metadata *newer; /**< pointer to the next newest entry */
        };
        union
        {
            Metadata *next; /**< pointer to next freeList entry */
            Metadata *older; /**< pointer to the next oldest entry */
        };
    };

    /** pointer to allocated Metadata pool */
    Metadata *pool;
    
    /** list of unused mapping entries */
    Metadata *freeList;
    
    /** Short hand for the alias Map type */
    typedef Map <NodeAlias, Metadata*> AliasMap;
    
    /** Short hand for the ID Map type */
    typedef Map <NodeID, Metadata*> IdMap;

    /** Map of alias to corresponding Metadata */
    AliasMap aliasMap;
    
    /** Map of Node ID to corresponding Metadata */
    IdMap idMap;
    
    /** oldest untouched entry */
    Metadata *oldest;
    
    /** newest, most recently touched entry */
    Metadata *newest;

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

} /* namepace NMRAnet */

#endif /* _NMRAnetAliasCache_hxx */
