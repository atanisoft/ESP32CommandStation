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
 * \file LinearMap.hxx
 * This file provides a C++ abstraction of BSD sys/tree.h.
 *
 * @author Stuart W. Baker
 * @date 2 December 2013
 */

#ifndef _UTILS_LINEARMAP_HXX_
#define _UTILS_LINEARMAP_HXX_

#include <stdio.h>  // for ssize_t
#include <sys/tree.hxx>

#include "utils/macros.h"

/** This is an abstraction of BSD sys/tree.h that is meant to mimic the
 * semantics of std::map without having to link in libstdc++.
 */
template <typename Key, typename Value> class LinearMap
{
public:
    /** Default Constructor which with no mapping entry limit.
     */
    LinearMap()
        : entries(0),
          used(0),
          list(NULL)
    {
        /* dynamic allocation not supported */
        HASSERT(0);
    }

    /** Constructor that limits the number of mappings to a static pool.
     * @param entries number of nodes to statically create and track
     */
    LinearMap(size_t entries) 
        : entries(entries),
          used(0),
          list(new Element[entries])
    {
    }

    /** Destructor.
     */
    ~LinearMap()
    {
        delete [] list;
    }

    /** list entry. */
    struct Pair
    {
        Key first; /**< mimic first element in an std::pair */
        Value second; /**< mimic second element in an std::pair */
    };

private:
    /** Entry element in mapping */
    struct Element
    {
        union
        {
            Pair p; /**< pair of element */
            struct
            {
                Key key; /**< key by which to sort the node */
                Value value; /**< value of the node */
            };
        };
    }; 

public:

    /** This mimics an std::Iterator.
     */
    class Iterator
    {
    public:
        /** Default constructor. The iterator must not be used until a valid
         * iterator is assigned to it. */
        Iterator() : index(0), m(nullptr)
        {
        }

        /** Copy constructor.
         */
        Iterator(const Iterator &it)
            : index(it.index),
              m(it.m)
        {
        }

        /** Constructor.
         * @param context context passed in at instantiation
         * @param index index to initialize this iteration with
         */
        Iterator(LinearMap* context, size_t index)
            : index(index),
              m(context)
        {
        }

        /** Destructor */
        ~Iterator()
        {
        }

        /** Overloaded reference operator.
         */
        Pair &operator*() const
        {
            return m->list[index].p;
        }

        /** Overloaded pointer operator.
         */
        Pair *operator->() const
        {
            return &m->list[index].p;
        }

        /** Overloaded pre-increement operator. @return new this.*/
        Iterator &operator ++ ()
        {
            if (index < m->used)
            {
                ++index;
            }
            return *this;
        }

        /** Overloaded not equals operator. @param it other oterator to
         * compare. @return comparison result. */
        bool operator != (const Iterator& it)
        {
            return m != it.m || index != it.index;
        }

        /** Overloaded equals operator. @param it other iterator to
         * compare. @return comparison result. */
        bool operator == (const Iterator& it)
        {
            return m == it.m && index == it.index;
        }

    private:
        /** index this iteration is currently indexed to */
        size_t index;
        
        /** Context this iteration lives in */
        LinearMap *m;
        
        /** Allow access to index member */
        friend class LinearMap;
    };

    /** Removes all elements */
    void clear()
    {
        used = 0;
    }

    /** Remove a node from the tree.
     * @param key key for the element to remove
     * @return number of elements removed
     */
    size_t erase(Key key)
    {
        for (size_t i = 0; i < used; ++i)
        {
            if (list[i].key == key)
            {
                /* scrunch up the list if we are able */
                if (i != --used)
                {
                    list[i].key = list[used].key;
                    list[i].value = list[used].value;
                }
                return 1;
            }
        }
        return 0;
    }
    
    /** Remove a node from the tree.
     * @param it Iterator index for the element to remove
     */
    void erase(Iterator it)
    {
        ssize_t i = it.index;
        HASSERT(i < (ssize_t)used && i >= 0);

        /* scrunch up the list if we are able */
        if (i != (ssize_t)--used)
        {
            list[i].key = list[used].key;
            list[i].value = list[used].value;
        }
    }
    
    /** Find the index associated with the key and create it does not exist.
     * @param key key to lookup
     * @return value of the key by reference
     */
    Value &operator[](const Key &key)
    {
        for (size_t i = 0; i < used; ++i)
        {
            if (list[i].key == key)
            {
                return list[i].value;
            }
        }
        
        HASSERT(used < entries);
        
        list[used].key = key;
        list[used].value = 0;
        
        return list[used++].value;
    }

    /** Number of elements currently in the map.
     * @return number of elements in the map
     */
    size_t size()
    {
        return used;
    }

    /** Maximum theoretical number of elements in the map.
     * @return maximum theoretical number of elements in the map
     */
    size_t max_size()
    {
        return entries;
    }

    /** Find an element matching the given key.
     * @param key key to search for
     * @return Iterator index pointing to key, else Iterator end() if not found
     */
    Iterator find(const Key &key)
    {
        for (size_t i = 0; i < used; ++i)
        {
            if (list[i].key == key)
            {
                return Iterator(this, i);
            }
        }
        return end();
    }
    
    /** Get an Iterator index pointing one past the last element in mapping.
     * @return Iterator index pointing to one past the last element in mapping
     */
    Iterator end()
    {
        return Iterator(this, size());
    }

    /** Get an Iterator index pointing to the first element in the mapping.
     * @return Iterator index pointing to the first element in the mapping or
     * to end() if the mapping is empty
     */
    Iterator begin()
    {
        return Iterator(this, 0);
    }

private:
    /** total number of entries for this instance */
    size_t entries;

    /** total number of entries in use for this instance */
    size_t used;

    /** list of entries */
    Element *list;
    
    DISALLOW_COPY_AND_ASSIGN(LinearMap);
};

#endif /* _UTILS_LINEARMAP_HXX_ */
