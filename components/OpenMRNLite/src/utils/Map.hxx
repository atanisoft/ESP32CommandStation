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
 * \file Map.hxx
 * This file provides a C++ abstraction of the several Key/Value pair mapping
 * implementations.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#ifndef _UTILS_MAP_HXX_
#define _UTILS_MAP_HXX_

#if defined (__LINEAR_MAP__)
#include "utils/LinearMap.hxx"
/** Define LinearMap as the base class for @ref Map */
#define BASE_CLASS LinearMap

#else

#include "utils/StlMap.hxx"
/** Define StlMap as the base class for @ref Map */
#define BASE_CLASS StlMap

#endif

/** Though the standard template library includes std::map, commonly
 * implemented as a Red Black tree, this alternative provides a transparent
 * mechanism for the user to at compile time decide between an implementation
 * that uses @ref StlMap based on std::map, SysMap based on BSD sys/tree.h, or
 * @ref LinearMap based on a small linear search.  If __LINEAR_MAP__ is defined,
 * @ref LinearMap will be used, else if __USE_LIBSTDCPP__ is defined, StlMap
 * will be used, else if neither is defined, SysMap will be used.  Given that
 * this is a header only implementation, a clever developer could define or
 * undefine __USE_LIBSTDCPP__ and/or __LENEAR_MAP__ ahead of a
 * #include "utils/Map.hxx" to override any compiler flag settings.
 */
template <typename Key, typename Value> class Map : public BASE_CLASS <Key, Value>
{
public:
    /** Default Constructor which with no mapping entry limit.
     */
    Map()
        : BASE_CLASS<Key, Value>()
    {
    }
    
    /** Constructor that limits the number of mappings to a static pool.
     * @param entries number of nodes to statically create and track
     */
    Map(size_t entries)
        : BASE_CLASS<Key, Value>(entries)
    {
    }
    
    /** Destructor.
     */
    ~Map()
    {
    }

private:
    DISALLOW_COPY_AND_ASSIGN(Map);
};

#undef BASE_CLASS

#endif /* _UTILS_MAP_HXX_ */
