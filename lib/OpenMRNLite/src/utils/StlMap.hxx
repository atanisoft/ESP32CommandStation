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
 * \file StlMap.hxx
 * This file provides a feature limited abstraction of std::map
 *
 * @author Stuart W. Baker
 * @date 2 December 2013
 */

#ifndef _UTILS_STLMAP_HXX_
#define _UTILS_STLMAP_HXX_

#include <map>

#include "utils/Allocator.hxx"
#include "utils/macros.h"

/** Though at the surface, this may seem like an unnecessary abstraction of
 * std::map, it has the purpose of limiting the implementation to features
 * found only in common to @ref SysMap and @ref LinearMap implementations.  In
 * this way, one can use @ref Map and based on compile time settings choose to
 * use any one of @ref StlMap, @ref SysMap, or @ref LinearMap from the same
 * source usage of @ref Map.
 */
template <typename Key, typename Value> class StlMap
{
public:
    /** Default Constructor which with no mapping entry limit.
     */
    StlMap()
        : mappingAllocator(NULL),
          mapping(new Mapping())
    {
    }

    /** Constructor that limits the number of mappings to a static pool.
     * @param entries number of nodes to statically create and track
     */
    StlMap(size_t entries) 
        : mappingAllocator(entries > 0 ? new MappingAllocator(std::less<Key>(), Allocator<std::pair<const Key, Value>>(entries)) : NULL),
          mapping(entries > 0 ? NULL : new Mapping())
    {
    }

    /** Destructor.
     */
    ~StlMap()
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
    typedef pair<Key, Value> Pair;

    /** Short hand for the iterator type of a given instance */
    typedef typename std::map<Key, Value>::iterator Iterator;

    /** Removes all elements in the map. */
    void clear() {
        if (mapping) {
            mapping->clear();
        } else {
            mappingAllocator->clear();
        }
    }

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
    
    /** Find the index associated with the key and create it if does not exist.
     * @param key key to lookup
     * @return value of the key by reference
     */
    Value& operator[](const Key &key)
    {
        return mapping ? (*mapping)[key] : (*mappingAllocator)[key];
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

private:
    /** short hand for the custom allocator std::map type */
    typedef std::map<Key, Value, std::less<Key>, Allocator<std::pair<const Key, Value>>> MappingAllocator;
    
    /** short hand for the default allocator std::map type */
    typedef std::map<Key, Value> Mapping;
    
    /** pointer to an std::map instance with a custom allocator */
    MappingAllocator *mappingAllocator;
    
    /** pointer to an std::map instance with a default allocator */
    Mapping *mapping;

    DISALLOW_COPY_AND_ASSIGN(StlMap);
};

#endif /* _UTILS_STLMAP_HXX_ */
