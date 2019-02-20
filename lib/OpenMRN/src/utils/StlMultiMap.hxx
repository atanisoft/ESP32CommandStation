/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file StlMultiMap.hxx
 * This file provides a feature limited abstraction of std::multimap
 *
 * @author Stuart W. Baker
 * @date 10 March 2014
 */

#ifndef _UTILS_STLMULTIMAP_HXX_
#define _UTILS_STLMULTIMAP_HXX_

#include <map>

#include "utils/Allocator.hxx"
#include "utils/macros.h"

/** Though at the surface, this may seem like an unnecessary abstraction of
 * std::multimap, it has the purpose of limiting the implementation to features
 * found only in common to @ref LinearMap implementation.  In this way,
 * one can use @ref MultiMap and based on compile time settings choose to use
 * any one of @ref StlMultiMap or @ref LinearMap from the same source
 * usage of @ref MultiMap.
 */
template <typename Key, typename Value> class StlMultiMap
{
public:
    /** Default Constructor which with no mapping entry limit.
     */
    StlMultiMap()
        : mappingAllocator(NULL),
          mapping(new Mapping())
    {
    }

    /** Constructor that limits the number of mappings to a static pool.
     * @param entries number of nodes to statically create and track
     */
    StlMultiMap(size_t entries) 
        : mappingAllocator(new MappingAllocator(std::less<Key>(), Allocator<std::pair<const Key, Value>>(entries))),
          mapping(NULL)
    {
    }

    /** Destructor.
     */
    ~StlMultiMap()
    {
        if (mappingAllocator)
        {
            delete mappingAllocator;
        }
        if (mapping)
        {
            delete mapping;
        }
    }

    /** This translation is done for consistency with @ref SysMap and @ref LinearMap */
    typedef std::pair<Key, Value> Pair;

    /** Short hand for the iterator type of a given instance */
    typedef typename std::multimap<Key, Value>::iterator Iterator;

    /** Remove an element from the tree.
     * @param key key for the element to remove
     * @return number of elements removed
     */
    size_t erase(Key key)
    {
        return mapping ? mapping->erase(key) : mappingAllocator->erase(key);
    }
    
    /** Remove a node from the tree.
     * @param it iterator index for the element to remove
     */
    void erase(Iterator it)
    {
        mapping ? mapping->erase(it) : mappingAllocator->erase(it);
    }

    /** Insert (create) a new key value pair.
     * @param pos hint as to the position to which to place the new mapping
     * @param val Key, Value Pair to insert into the mapping
     * @return Iterator pointing to new or exising Key, Value Pair
     */
    Iterator insert(const Iterator pos, const Pair& val)
    {
        return mapping ? mapping->insert(pos, val) : mappingAllocator->insert(pos, val);
    }

    /** Insert (create) a new key value pair.
     * @param val Key, Value Pair to insert into the mapping
     * @return Iterator pointing to new or exising Key, Value Pair
     */
    Iterator insert(const Pair& val)
    {
        return mapping ? mapping->insert(val) : mappingAllocator->insert(val);
    }

    /** Number of elements currently in the map.
     * @return number of elements in the map
     */
    size_t size()
    {
        return mapping ? mapping->size() : mappingAllocator->size();
    }

    /** Maximum theoretical number of elements in the map.
     * @return maximum theoretical number of elements in the map
     */
    size_t max_size()
    {
        return mapping ? mapping->max_size() : mappingAllocator->max_size();
    }

    /** Find an element matching the given key.
     * @param key key to search for
     * @return iterator index pointing to key, else iterator end() if not found
     */
    Iterator find( const Key &key )
    {
        return mapping ? mapping->find(key) : mappingAllocator->find(key);
    }
    
    /** Get an iterator pointing the  the first element in the map
     * with the provided key, else iterator end() if not found.
     * @param key key for the elements to lower bound
     * @return iterator pointing to first element with the provided key
     */
    Iterator lower_bound(const Key& key)
    {
        return mapping ? mapping->lower_bound(key) : mappingAllocator->lower_bound(key);
    }

    /** Get an iterator pointing the  the first element in the map
     * after the provided key.
     * @param key key for the elements to upper bound
     * @return iterator pointing to first element after the provided key
     */
    Iterator upper_bound(const Key& key)
    {
        return mapping ? mapping->upper_bound(key) : mappingAllocator->upper_bound(key);
    }

    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    Iterator end()
    {
        return mapping ? mapping->end() : mappingAllocator->end();
    }
    
    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    Iterator begin()
    {
        return mapping ? mapping->begin() : mappingAllocator->begin();
    }

    /** Get the count of elements with a givin Key.
     * @param key key for the elements to count
     * @return number of elements with a givin key
     */
    size_t count(const Key& key) const
    {
        return mapping ? mapping->count(key) : mappingAllocator->count(key);
    }

private:
    /** short hand for the custom allocator std::map type */
    typedef std::multimap<Key, Value, std::less<Key>, Allocator<std::pair<const Key, Value>>> MappingAllocator;
    
    /** short hand for the default allocator std::map type */
    typedef std::multimap<Key, Value> Mapping;
    
    /** pointer to an std::map instance with a custom allocator */
    MappingAllocator *mappingAllocator;
    
    /** pointer to an std::map instance with a default allocator */
    Mapping *mapping;

    DISALLOW_COPY_AND_ASSIGN(StlMultiMap);
};

#endif /* _UTILS_STLMULTIMAP_HXX_ */
