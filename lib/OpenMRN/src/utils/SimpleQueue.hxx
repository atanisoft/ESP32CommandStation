/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file SimpleQueue.hxx
 * A simple fast single-linked queue class with non-virtual methods.
 *
 * @author Balazs Racz
 * @date 6 Apr 2015
 */

#ifndef _UTILS_SIMPLEQUEUE_HXX_
#define _UTILS_SIMPLEQUEUE_HXX_

#include "utils/QMember.hxx"

/// A simple fast single-linked stack class with non-virtual methods.
///
/// This structure allows putting QMember descendants (including buffers,
/// stateflows, exeutables etc) into a queue without incurring the overhead of
/// the virtual methods and allocation semantics of BufferQueue.
///
/// SimpleQueue does not support asynchronous access, allocation/deallocation
/// and notification, but supports strongly typed access (via @ref TypedQueue)
/// and queueing types other than Buffers. Almost all functions on SimpleQueue
/// compile into just a few machine instructions.
class SimpleQueue {
public:
    SimpleQueue() : head_(nullptr) {}

    /// @return true ofthe queue has no elements in there.
    bool empty() {
        return !head_;
    }

    /// Adds an entry to the front of the queue. @param member entry to add.
    void push_front(QMember* member) {
        HASSERT(!member->next);
        member->next = head_;
        head_ = member;
    }

    /// Removes the entry at the front of the queue. @return the entry at the
    /// front of the queue.
    QMember* pop_front() {
        HASSERT(head_);
        QMember* f = head_;
        head_ = f->next;
        f->next = nullptr;
        return f;
    }

    /// peeks. @return the entry at the front of the queue.
    QMember* front() const {
        return head_;
    }

    /// Static iterator class pointing to the "end" of the queue.
    class end_iterator {};

    /// STL-compatible iterator for SimpleQueue.
    class iterator {
    public:
        /// Constructor. @param link pointer to link.
        iterator(QMember** link): link_(link) {
            HASSERT(link_);
        }

        bool operator==(const iterator& o) const {
            return *link_ == *o.link_;
        }

        bool operator==(const end_iterator& o) const {
            return *link_ == nullptr;
        }

        bool operator!=(const iterator& o) const {
            return *link_ != *o.link_;
        }

        bool operator!=(const end_iterator& o) const {
            return *link_ != nullptr;
        }

        iterator& operator++() {
            HASSERT(*link_);
            link_ = &(*link_)->next;
            return *this;
        }

        QMember* operator->() {
            return *link_;
        }

        QMember& operator*() {
            return **link_;
        }

    private:
        friend class SimpleQueue;

        /// **link == *this for the iterator semantics. Points to the pointer
        /// **to the entry **this.
        QMember** link_;
    };

    /// STL-compatible iterator for TypedQueue.
    template<class T> class typed_iterator : public iterator {
    public:
        /// Typed itartor consturctor. @param link is the queue link entry
        /// pointing to *this.
        typed_iterator(QMember** link): iterator(link) {}
        T* operator->() {
            return static_cast<T*>(*link_);
        }
        T& operator*() {
            return static_cast<T&>(**link_);
        }
    };

    /// @return iterator pointing to first element.
    iterator begin() {
        return iterator(&head_);
    }

    /** @return a sentinel to compare against for determining when an iteration
     * is done. This sentinel cannot be used to insert entries at the end of
     * the queue. */
    end_iterator end() {
        return end_iterator{};
    }

    /** Inserts the element entry before the position.  The iterator will point
     * to the new member.
     * @param position which entry to insert before
     * @param entry what to insert before *position.
     */
    void insert(const iterator& position, QMember* entry) {
        HASSERT(!entry->next);
        entry->next = *position.link_;
        *position.link_ = entry;
    }

    /** Removes the entry pointed to by the iterator. The iterator will
     * afterwards point to the next member after the removed one (or end() if
     * this was the last member). Invalidates any other iterator pointing just
     * behind the marked position (but not iterators pointing elsewhere).
     * @param position iterator to entry to remove */
    void erase(const iterator& position) {
        QMember* m = *position.link_;
        HASSERT(m);
        *position.link_ = m->next;
        m->next = nullptr;
    }

protected:
    /** Used as a guard for comparing against for the end of the queue. */
    static QMember* const PTR_END;

    /// Top pointer for the stack.
    QMember* head_;
};


/// A simple, fast, type-safe single-linked queue class with non-virtual
/// methods.
///
/// see @ref SimpleQueue for details.
template<class T>
class TypedQueue : public SimpleQueue {
public:
    /// Inserts an entry to the front of the queue.
    void push_front(T* entry) {
        SimpleQueue::push_front(entry);
    }

    /// Removes the entry at the front of the queue. @return the entry at the
    /// front of the queue.
    T* pop_front() {
        return static_cast<T*>(SimpleQueue::pop_front());
    }

    /// peeks. @return the entry at the front of the queue.
    T* front() const {
        return static_cast<T*>(SimpleQueue::front());
    }

    /// Typed iterator type.
    typedef typed_iterator<T> iterator;

    /// @return iterator pointing to first element.
    iterator begin() {
        return iterator(&head_);
    }
};

#endif // _UTILS_SIMPLEQUEUE_HXX_
