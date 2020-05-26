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

#include "os/OS.hxx"

namespace openlcb
{

#define CONSTANT 0x1B0CA37ABA9 /**< constant for random number generation */

const NodeID AliasCache::RESERVED_ALIAS_NODE_ID = 1;

void AliasCache::clear()
{
    idMap.clear();
    aliasMap.clear();
    oldest = nullptr;
    newest = nullptr;
    freeList = nullptr;
    /* initialize the freeList */
    for (size_t i = 0; i < entries; ++i)
    {
        pool[i].prev = NULL;
        pool[i].next = freeList;
        freeList = pool + i;
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

    AliasMap::Iterator it = aliasMap.find(alias);
    if (it != aliasMap.end())
    {
        /* we already have a mapping for this alias, so lets remove it */
        insert = (*it).second;
        remove(alias);
        
        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(insert->id, insert->alias, context);
        }
    }

    if (freeList)
    {
        /* found an empty slot */
        insert = freeList;
        freeList = insert->next;        
    }
    else
    {
        HASSERT(oldest != NULL && newest != NULL);

        /* kick out the oldest mapping and re-link the oldest endpoint */
        insert = oldest;
        if (oldest->newer)
        {
            oldest->newer->older = NULL;
        }
        if (insert == newest)
        {
            newest = NULL;
        }
        oldest = oldest->newer;

        aliasMap.erase(insert->alias);
        idMap.erase(insert->id);

        if (removeCallback)
        {
            /* tell the interface layer that we removed this mapping */
            (*removeCallback)(insert->id, insert->alias, context);
        }
    }
        
    insert->timestamp = OSTime::get_monotonic();
    insert->id = id;
    insert->alias = alias;

    aliasMap[alias] = insert;
    idMap[id] = insert;

    /* update the time based list */
    insert->newer = NULL;
    if (newest == NULL)
    {
        /* if newest == NULL, then oldest must also be NULL */
        HASSERT(oldest == NULL);

        insert->older = NULL;
        oldest = insert;
    }
    else
    {
        insert->older = newest;
        newest->newer = insert;
    }

    newest = insert;
    
    return;
}

/** Remove an alias from an alias cache.  This method does not call the
 * remove_callback method passed in at construction since it is a
 * deliberate call not requiring notification.
 * @param alias 12-bit alias associated with Node ID
 */
void AliasCache::remove(NodeAlias alias)
{
    AliasMap::Iterator it = aliasMap.find(alias);

    if (it != aliasMap.end())
    {
        Metadata *metadata = (*it).second;
        aliasMap.erase(it);
        idMap.erase(metadata->id);
        
        if (metadata->newer)
        {
            metadata->newer->older = metadata->older;
        }
        if (metadata->older)
        {
            metadata->older->newer = metadata->newer;
        }
        if (metadata == newest)
        {
            newest = metadata->older;
        }
        if (metadata == oldest)
        {
            oldest = metadata->newer;
        }
    
        metadata->next = freeList;
        freeList = metadata;
    }
    
}

bool AliasCache::retrieve(unsigned entry, NodeID* node, NodeAlias* alias)
{
    HASSERT(entry < size());
    Metadata* md = pool + entry;
    if (!md->alias) return false;
    if (node) *node = md->id;
    if (alias) *alias = md->alias;
    return true;
}

/** Lookup a node's alias based on its Node ID.
 * @param id Node ID to look for
 * @return alias that matches the Node ID, else 0 if not found
 */
NodeAlias AliasCache::lookup(NodeID id)
{
    HASSERT(id != 0);

    IdMap::Iterator it = idMap.find(id);

    if (it != idMap.end())
    {
        Metadata *metadata = (*it).second;
        
        /* update timestamp */
        touch(metadata);
        return metadata->alias;
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

    AliasMap::Iterator it = aliasMap.find(alias);

    if (it != aliasMap.end())
    {
        Metadata *metadata = (*it).second;
        
        /* update timestamp */
        touch(metadata);
        return metadata->id;
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

    for (Metadata *metadata = newest; metadata != NULL; metadata = metadata->older)
    {
        (*callback)(context, metadata->id, metadata->alias);
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
    metadata->timestamp = OSTime::get_monotonic();

    if (metadata != newest)
    {
        if (metadata == oldest)
        {
            oldest = metadata->newer;
            oldest->older = NULL;
        }
        else
        {
            /* we have someone older */
            metadata->older->newer = metadata->newer;
        }
        metadata->newer->older = metadata->older;
        metadata->newer = NULL;
        metadata->older = newest;
        newest->newer = metadata;
        newest = metadata;
    }
}

}
