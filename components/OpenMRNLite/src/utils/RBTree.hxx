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
 * \file RBTree.hxx
 * This file provides a C++ abstraction of the BSD <sys/tree.h>.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#ifndef _UTILS_RBTREE_HXX_
#define _UTILS_RBTREE_HXX_

extern "C"
{
#include <sys/tree.hxx>
};

#include "utils/macros.h"

/** Though the standard template library includes std::map, commonly
 * implemented as a Red Black tree, this alternative provides a transparent
 * mechanism for the user to manage all of the memory used by the tree.  This
 * allows for a instantiation that does not require the dynamic allocation
 * and freeing of small chunks of memory.  Note, there is no mutual exclusion
 * locking mechanism built into this class.  Mutual exclusion must be handled
 * by the user as needed.
 */ 
template <typename Key, typename Value> class RBTree
{
public:
    /** Default Constructor
     */
    RBTree()
        : freeList(NULL)
    {
        RB_INIT(&head);
    }

    /** Default Constructor.
     * @param nodes number of nodes to statically create and track
     */
    RBTree(size_t nodes)
        : freeList(NULL)
    {
        RB_INIT(&head);
        Node *first = new Node[nodes];
        first->entry.rbe_left = (Node*)this;
        
        for (size_t i = 1; i < nodes; i++)
        {
            first[i].entry.rbe_left = freeList;
            freeList = first + i;
        }
    }

    /** The metadata for a tree node. */
    struct Node
    {
    public:
        /// The pointer structure.
        RB_ENTRY(Node) entry;
        Key key; /**< key by which to sort the node */
        Value value; /**< value of the node */

        /** Default constructor.  Does not initialize key or value.
         */
        Node()
        {
        }
        
        /** Constructor.
         * @param key initial value of key
         * @param value initial value of value
         */
        Node(Key key, Value value)
            : key(key),
              value(value)
        {
        }
    };

    /** Insert a node into the tree from pre-allocated Node pool.
     * @param key key to insert
     * @param value value to insert
     * @return reference to new node inserted, existing matching node, or NULL
     *         if there are no more pre-allocated nodes left
     */
    Node *insert(Key key, Value value)
    {
        Node *node = find(key);
        if (node)
        {
            node->value = value;
        }
        else
        {
            node = alloc();
            if (node)
            {
                node->key = key;
                node->value = value;
                insert(node);
            }
        }
        return node;
    }

    /** Insert a node into the tree.
     * @param node to insert
     */
    void insert(Node *node)
    {
        RB_INSERT(tree, &head, node);
    }

    /** Remove a node from the tree.
     * @param node node to remove
     */    
    void remove(Node *node)
    {
        RB_REMOVE(tree, &head, node);
        free(node);
    }
    
    /** Remove a node from the tree.
     * @param key key for the node to remove
     * @return pointer to node that was removed, NULL if not found
     */
    Node *remove(Key key)
    {
        Node *node = find(key);
        if (node)
        {
            remove(node);
            free(node);
        }
        return node;
    }
    
    /** Find a node based on its lookup key.
     * @param key key of node to lookup.
     * @return pointer to node found, NULL if not found
     */
    Node *find(Key key)
    {
        Node lookup;
        
        lookup.key = key;
        
        return RB_FIND(tree, &head, &lookup);
    };
    
    /** Get the first node in the tree.
     * @return first node in the tree, NULL if tree is empty
     */
    Node *first()
    {
        return RB_MIN(tree, &head);
    }
    
    /** Get the last node in the tree.
     * @return last node in the tree, NULL if tree is empty
     */
    Node *last()
    {
        return RB_MAX(tree, &head);
    }

    /** Get the next node in the tree.
     * @param node node to get the next node from
     * @return next node in the tree, NULL if at the end of the tree
     */    
    Node *next(Node *node)
    {
        return RB_NEXT(tree, &head, node);
    }
    
    /** Get the next node in the tree.
     * @param key key of the node to get the next node from
     * @return next node in the tree, NULL if at the end of the tree
     */    
    Node *next(Key key)
    {
        Node *node = find(key);
        return node ? RB_NEXT(tree, &head, node) : node;
    }
    
    /** Get the previous node in the tree.
     * @param node node to get the previous node from
     * @return previous node in the tree, NULL if at the beginning of the tree
     */    
    Node *previous(Node *node)
    {
        return RB_PREV(tree, &head, node);
    }
    
    /** Default destructor */
    ~RBTree()
    {
    }
    
private:
    /** Allocate a node from the free list.
     * @return newly allocated node, else NULL if no free nodes left
     */
    Node *alloc()
    {
        if (freeList && freeList != (Node*)this)
        {
            Node *node = freeList;
            freeList = freeList->entry.rbe_left;
            return node;
        }
        return NULL;
    }
    
    /** free a node to the free list if it exists.
     * @param node node to free
     */
    void free(Node *node)
    {
        if (freeList || freeList == (Node*)this)
        {
            node->entry.rbe_left = freeList;
            freeList = node;
        }
    }

    /** Compare two nodes.
     * @param a first of two nodes to compare
     * @param b second of two nodes to compare
     * @return difference between node keys (a->key - b->key)
     */
    Key compare(Node *a, Node *b)
    {
        return a->key - b->key;
    }

    /** list of free nodes */
    Node *freeList;

    /** The datagram tree type. */
    RB_HEAD(tree, Node);

    /** The datagram tree methods. */
    RB_GENERATE(tree, Node, entry, compare);
    
    /** tree instance */
    struct tree head;
    
    DISALLOW_COPY_AND_ASSIGN(RBTree);
};

#endif /* _UTILS_RBTREE_HXX_ */
