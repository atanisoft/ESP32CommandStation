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
 * \file Allocator.hxx
 * This file provides an implementation of a C++ Allocator with onetime up
 * front allocation of memory.
 *
 * @author Stuart W. Baker
 * @date 10 March 2014
 */

#ifndef _UTILS_ALLOCATOR_HXX_
#define _UTILS_ALLOCATOR_HXX_

#include "utils/macros.h"

/** This is a custom allocator that limits the number of mappings.  It also
 * performs a single dynamic allocation capable of holding the total number
 * of mappings on the first allocation request.  This is so that successive
 * allocation requests are faster, more deterministic, and don't waste
 * memory potentially used for the heap management headers.
 */
template <typename T> class Allocator
{
public:
    /** List of unused elements.
     */
    union FreeList
    {
        T data; /**< unused item */
        FreeList *next; /**< next item in the list */
    };

    /** Start of free list */
    FreeList *freeList;
    
    /** flag that tell us if we have initilized our selves or not */
    bool init;
    
    /** number of elements in the fixed size pool */
    size_t entries;
    
    typedef T value_type; /**< value_type required by stl */
    typedef value_type* pointer; /**< pointer required by stl */
    typedef const value_type* const_pointer; /**< const_pointer required by stl */
    typedef value_type& reference; /**< reference required by stl */
    typedef const value_type& const_reference; /**< const_reference required by stl */
    typedef std::size_t size_type; /**< size_type required by stl */
    typedef std::ptrdiff_t difference_type; /**< difference_type required by stl */
    
    /** typedef for allocator specialization */
    template <typename U> struct rebind
    {
        /** typedef for allocator specialization */
        typedef Allocator<U> other;
    };

    /** Constructor.
     * @param e number of entries in the fixed size pool
     */
    explicit Allocator(size_t e)
        : freeList(NULL),
          init(false),
          entries(e)
    {
    }

    /** Copy constructor.
     * @param a instance to copy
     */
    explicit Allocator(Allocator const& a)
        : freeList(a.freeList),
          init(a.init),
          entries(a.entries)
    {
    }
    
    /** Destructor.
     */
    ~Allocator()
    {
    }
    
    /** template copy constructor.
     * @param o insance to copy
     */
    template <typename U> Allocator(Allocator<U> const& o)
        : freeList(NULL),
          init(o.init),
          entries(o.entries)
    {
    }

    /** Address of item.
     * @param r item to take the address of
     * @return address of item
     */
    T *address(T &r)
    {
        return &r;
    }

    /** Const address of item.
     * @param r item to take the address of
     * @return address of item
     */
    const T *address(const T &r)
    {
        return &r;
    }

    /** Allocate item(s) out of the pool.
     * @param cnt number of items to allocate
     * @return newly allocated item(s)
     */
    T *allocate(size_t cnt, const void* = 0)
    {
        if (init == false)
        {
            HASSERT(entries != 0);
            init = true;
            FreeList *newList = (FreeList*)malloc(sizeof(FreeList) * max_size());
            for (size_t i = 0; i < max_size(); ++i)
            {
                newList[i].next = freeList;
                freeList = newList + i;
            }
        }
        HASSERT(freeList != NULL);
        HASSERT(cnt == 1);
        
        T *newT = &(freeList->data);
        freeList = freeList->next;
        return newT;
    }
    
    /** Free itme.
     * @param p item to free
     * @param n number of items to free
     */
    void deallocate(T *p, size_t n)
    {
        HASSERT(n == 1);
        FreeList *pFreeList = (FreeList*)p;
        pFreeList->next = freeList;
        freeList = pFreeList;
    }

    /** Maximum number of items that can be allocated.
     */
    size_t max_size() const
    {
        return entries;
    }

    
    /** Placement constructor.
     * @param p location of placement
     * @param t parameter to constructor
     */
    void construct(T *p, const T& t)
    {
        new(p) T(t);
    }
    
    /** Destruct.
     * @param p item to destruct
     */
    void destroy(T *p)
    {
        p->~T();
    }

    /** Overloaded operator ==.
     * @return always true.
     */
    bool operator==(Allocator const&)
    {
        return true;
    }
    
    /** Overloaded operator !=
     * @param a item to compare
     * @return always false
     */
    bool operator!=(Allocator const& a)
    {
        return !operator==(a);
    }
private:
    /** Default Constructor.
     */
    Allocator()
        : init(false),
          entries(0)
    {
    }
};

#endif /* _UTILS_ALLOCATOR_HXX_ */
