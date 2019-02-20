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
 * \file MultiMap.hxx
 * This file provides a C++ abstraction of the several Key/Value pair
 * multi-mapping implementations.
 *
 * @author Stuart W. Baker
 * @date 15 March 2014
 */

#ifndef _UTILS_MULTIMAP_HXX_
#define _UTILS_MULTIMAP_HXX_

#if defined (__LINEAR_MULTIMAP__)
#include "utils/LinearMultiMap.hxx"
/** Define LinearMap as the base class for @ref MultiMap */
#define BASE_CLASS LinearMultiMap

#elif defined (__USE_LIBSTDCPP__)
#include "utils/StlMultiMap.hxx"
/** Define stlMultiMap as the base class for @ref MultiMap */
#define BASE_CLASS StlMultiMap

#else
#include "utils/StlMultiMap.hxx"
/** Define StlMultiMap as the base class for @ref MultiMap */
#define BASE_CLASS StlMultiMap

#endif

/** MultiMap abstraction that will allow access to one of StlMultiMap or
 * LinearMultiMap.
 */
template <typename Key, typename Value> class MultiMap : public BASE_CLASS <Key, Value>
{
public:
    /** Default Constructor which with no mapping entry limit.
     */
    MultiMap()
        : BASE_CLASS<Key, Value>()
    {
    }
    
    /** Constructor that limits the number of mappings to a static pool.
     * @param entries number of nodes to statically create and track
     */
    MultiMap(size_t entries)
        : BASE_CLASS<Key, Value>(entries)
    {
    }
    
    /** Destructor.
     */
    ~MultiMap()
    {
    }

private:
    DISALLOW_COPY_AND_ASSIGN(MultiMap);
};

#undef BASE_CLASS

#endif /* _UTILS_MULTIMAP_HXX_ */
