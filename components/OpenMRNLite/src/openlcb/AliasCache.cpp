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
 * \file AliasCache.cxx
 * This file provides an Alias Caching mechanism.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#include "openlcb/AliasCache.hxx"

#include <set>

#include "os/OS.hxx"

#ifdef GTEST
#define TEST_CONSISTENCY
#endif

namespace openlcb
{

#define CONSTANT 0x1B0CA37ABA9 /**< constant for random number generation */

#if defined(TEST_CONSISTENCY)
extern volatile int consistency_result;
volatile int consistency_result = 0;

int AliasCache::check_consistency()
{
    if (idMap.size() != aliasMap.size())
    {
        return 1;
    }
    if (aliasMap.size() == entries)
    {
        if (!freeList.empty())
        {
            return 2;
        }
    }
    else
    {
        if (freeList.empty())
        {
            return 3;
        }
    }
    if (aliasMap.size() == 0 && (!oldest.empty() || !newest.empty()))
    {
        return 4;
    }
    std::set<void *> free_entries;
    for (PoolIdx p = freeList; !p.empty(); p = p.deref(this)->older_)
    {
        Metadata *m = p.deref(this);
        if (free_entries.count(m))
        {
            return 5; // duplicate entry on freelist
        }
        free_entries.insert(m);
    }
    if (free_entries.size() + aliasMap.size() != entries)
    {
        return 6; // lost some metadata entries
    }
    for (auto kv : aliasMap)
    {
        if (free_entries.count(kv.deref(this)))
        {
            return 19;
        }
    }
    for (auto kv : idMap)
    {
        if (free_entries.count(kv.deref(this)))
        {
            return 20;
        }
    }
    if (aliasMap.size() == 0)
    {
        if (!oldest.empty())
        {
            return 7;
        }
        if (!newest.empty())
        {
            return 8;
        }
    }
    else
    {
        if (oldest.empty())
        {
            return 9;
        }
        if (newest.empty())
        {
            return 10;
        }
        if (free_entries.count(oldest.deref(this)))
        {
            return 11; // oldest is free
        }
        if (free_entries.count(newest.deref(this)))
        {
            return 12; // newest is free
        }
    }
    if (aliasMap.size() == 0)
    {
        return 0;
    }
    // Check linking.
    {
        PoolIdx prev = oldest;
        unsigned count = 1;
        if (!prev.deref(this)->older_.empty())
        {
            return 13;
        }
        while (!prev.deref(this)->newer_.empty())
        {
            auto next = prev.deref(this)->newer_;
            ++count;
            if (free_entries.count(next.deref(this)))
            {
                return 21;
            }
            if (next.deref(this)->older_.idx_ != prev.idx_)
            {
                return 14;
            }
            prev = next;
        }
        if (prev.idx_ != newest.idx_)
        {
            return 18;
        }
        if (count != aliasMap.size())
        {
            return 27;
        }
    }
    {
        PoolIdx next = newest;
        if (!next.deref(this)->newer_.empty())
        {
            return 15;
        }
        while (!next.deref(this)->older_.empty())
        {
            auto prev = next.deref(this)->older_;
            if (free_entries.count(prev.deref(this)))
            {
                return 22;
            }
            if (prev.deref(this)->newer_.idx_ != next.idx_)
            {
                return 16;
            }
            next = prev;
        }
        if (next.idx_ != oldest.idx_)
        {
            return 17;
        }
    }
    for (unsigned i = 0; i < entries; ++i)
    {
        if (free_entries.count(pool + i))
        {
            continue;
        }
        auto *e = pool + i;
        if (idMap.find(e->get_node_id()) == idMap.end())
        {
            return 23;
        }
        if (idMap.find(e->get_node_id())->idx_ != i)
        {
            return 24;
        }
        if (aliasMap.find(e->alias_) == aliasMap.end())
        {
            return 25;
        }
        if (aliasMap.find(e->alias_)->idx_ != i)
        {
            return 26;
        }
    }
    return 0;
}

#endif

void AliasCache::clear()
{
    idMap.clear();
    aliasMap.clear();
    oldest.idx_ = NONE_ENTRY;
    newest.idx_ = NONE_ENTRY;
    freeList.idx_ = NONE_ENTRY;
    /* initialize the freeList */
    for (size_t i = 0; i < entries; ++i)
    {
        pool[i].newer_.idx_ = NONE_ENTRY;
        pool[i].older_ = freeList;
        freeList.idx_ = i;
    }
}

/** Add an alias to an alias cache.
 * @param id 48-bit NMRAnet Node ID to associate alias with
 * @param alias 12-bit alias associated with Node ID
 */
void AliasCache::add(NodeID id, NodeAlias alias)
{
    HASSERT(id != 0);
    HASSERT(alias != 0);
    
    Metadata *insert;

    auto it = aliasMap.find(alias);
    if (it != aliasMap.end())
    {
        /* we already have a mapping for this alias, so lets remove it */
        insert = it->deref(this);
        remove(insert->alias_);

        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(insert->get_node_id(), insert->alias_, context);
        }
    }
    auto nit = idMap.find(id);
    if (nit != idMap.end())
    {
        /* we already have a mapping for this id, so lets remove it */
        insert = nit->deref(this);
        remove(insert->alias_);

        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(insert->get_node_id(), insert->alias_, context);
        }
    }

    if (!freeList.empty())
    {
        /* found an empty slot */
        insert = freeList.deref(this);
        freeList = insert->older_;
    }
    else
    {
        HASSERT(!oldest.empty() && !newest.empty());

        /* kick out the oldest mapping and re-link the oldest endpoint */
        insert = oldest.deref(this);
        auto second = insert->newer_;
        if (!second.empty())
        {
            second.deref(this)->older_.idx_ = NONE_ENTRY;
        }
        if (insert == newest.deref(this))
        {
            newest.idx_ = NONE_ENTRY;
        }
        oldest = second;

        aliasMap.erase(aliasMap.find(insert->alias_));
        idMap.erase(idMap.find(insert->get_node_id()));

        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(insert->get_node_id(), insert->alias_, context);
        }
    }

    insert->set_node_id(id);
    insert->alias_ = alias;

    PoolIdx n;
    n.idx_ = insert - pool;
    aliasMap.insert(PoolIdx(n));
    idMap.insert(PoolIdx(n));

    /* update the time based list */
    insert->newer_.idx_ = NONE_ENTRY;
    if (newest.empty())
    {
        /* if newest == NULL, then oldest must also be NULL */
        HASSERT(oldest.empty());

        insert->older_.idx_ = NONE_ENTRY;
        oldest = n;
    }
    else
    {
        insert->older_ = newest;
        newest.deref(this)->newer_ = n;
    }

    newest = n;

#if defined(TEST_CONSISTENCY)
    consistency_result = check_consistency();
    HASSERT(0 == consistency_result);
#endif
}

/** Remove an alias from an alias cache.  This method does not call the
 * remove_callback method passed in at construction since it is a
 * deliberate call not requiring notification.
 * @param alias 12-bit alias associated with Node ID
 */
void AliasCache::remove(NodeAlias alias)
{
    auto it = aliasMap.find(alias);

    if (it != aliasMap.end())
    {
        Metadata *metadata = it->deref(this);
        aliasMap.erase(it);
        idMap.erase(idMap.find(metadata->get_node_id()));

        if (!metadata->newer_.empty())
        {
            metadata->newer_.deref(this)->older_ = metadata->older_;
        }
        if (!metadata->older_.empty())
        {
            metadata->older_.deref(this)->newer_ = metadata->newer_;
        }
        if (metadata == newest.deref(this))
        {
            newest = metadata->older_;
        }
        if (metadata == oldest.deref(this))
        {
            oldest = metadata->newer_;
        }

        metadata->older_ = freeList;
        freeList.idx_ = metadata - pool;
    }

#if defined(TEST_CONSISTENCY)
    consistency_result = check_consistency();
    HASSERT(0 == consistency_result);
#endif
}

bool AliasCache::retrieve(unsigned entry, NodeID* node, NodeAlias* alias)
{
    HASSERT(entry < size());
    Metadata* md = pool + entry;
    if (!md->alias_) return false;
    if (node) *node = md->get_node_id();
    if (alias) *alias = md->alias_;
    return true;
}

/** Lookup a node's alias based on its Node ID.
 * @param id Node ID to look for
 * @return alias that matches the Node ID, else 0 if not found
 */
NodeAlias AliasCache::lookup(NodeID id)
{
    HASSERT(id != 0);

    auto it = idMap.find(id);

    if (it != idMap.end())
    {
        Metadata *metadata = it->deref(this);

        /* update timestamp */
        touch(metadata);
        return metadata->alias_;
    }

    /* no match found */
    return 0;
}

/** Lookup a node's ID based on its alias.
 * @param alias alias to look for
 * @return Node ID that matches the alias, else 0 if not found
 */
NodeID AliasCache::lookup(NodeAlias alias)
{
    HASSERT(alias != 0);

    auto it = aliasMap.find(alias);

    if (it != aliasMap.end())
    {
        Metadata *metadata = it->deref(this);

        /* update timestamp */
        touch(metadata);
        return metadata->get_node_id();
    }
    
    /* no match found */
    return 0;
}

/** Call the given callback function once for each alias tracked.  The order
 * will be in last "touched" order.
 * @param callback method to call
 * @param context context pointer to pass to callback
 */
void AliasCache::for_each(void (*callback)(void*, NodeID, NodeAlias), void *context)
{
    HASSERT(callback != NULL);

    for (PoolIdx idx = newest; !idx.empty(); idx = idx.deref(this)->older_)
    {
        Metadata *metadata = idx.deref(this);
        (*callback)(context, metadata->get_node_id(), metadata->alias_);
    }
}

/** Generate a 12-bit pseudo-random alias for a givin alias cache.
 * @return pseudo-random 12-bit alias, an alias of zero is invalid
 */
NodeAlias AliasCache::generate()
{
    NodeAlias alias;

    do
    {
        /* calculate the alias given the current seed */
        alias = (seed ^ (seed >> 12) ^ (seed >> 24) ^ (seed >> 36)) & 0xfff;

        /* calculate the next seed */
        seed = ((((1 << 9) + 1) * (seed) + CONSTANT)) & 0xffffffffffff;
    } while (alias == 0 || lookup(alias) != 0);

    /* new random alias */
    return alias;
}

/** Update the time stamp for a given entry.
 * @param  metadata metadata associated with the entry
 */
void AliasCache::touch(Metadata* metadata)
{
    if (metadata != newest.deref(this))
    {
        if (metadata == oldest.deref(this))
        {
            oldest = metadata->newer_;
            oldest.deref(this)->older_.idx_ = NONE_ENTRY;
        }
        else
        {
            /* we have someone older */
            metadata->older_.deref(this)->newer_ = metadata->newer_;
        }
        metadata->newer_.deref(this)->older_ = metadata->older_;
        metadata->newer_.idx_ = NONE_ENTRY;
        metadata->older_ = newest;
        newest.deref(this)->newer_.idx_ = metadata - pool;
        newest.idx_ = metadata - pool;
    }
#if defined(TEST_CONSISTENCY)
    consistency_result = check_consistency();
    HASSERT(0 == consistency_result);
#endif
}

}
