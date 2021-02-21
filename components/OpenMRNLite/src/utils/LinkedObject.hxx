/** \copyright
 * Copyright (c) 2017, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file LinkedObject.hxx
 *
 * Helper class that allows all instances of a specific class to be maintained
 * in a linked list.
 *
 * @author Balazs Racz
 * @date 9 Sep 2017
 */

#ifndef _UTILS_LINKEDOBJECT_HXX_
#define _UTILS_LINKEDOBJECT_HXX_

#include <atomic>

#include "utils/Atomic.hxx"
#include "utils/Uninitialized.hxx"

/// This object encapsulates an Atomic, which never gets destroyed. It is
/// intended to be used only as static object, because we depend on zero
/// initialization at construction time.
class CreateOnlyAtomic
{
public:
    CreateOnlyAtomic()
    {
        // Ensures the object gets created at static initialization time while
        // the program is still single threaded.
        get();
    }

    Atomic *get()
    {
        if (isInitialized_.exchange(1) == 0)
        {
            atomic_.emplace();
        }
        return &atomic_.value();
    }

private:
    std::atomic_uint_least8_t isInitialized_;
    uninitialized<Atomic> atomic_;
};

/// This static object is factored into a separate namespace because current
/// GDB crashes when trying to print an object with a static Atomic member
/// variable.
template <class T> class LinkedObjectHeadMutex
{
public:
    static CreateOnlyAtomic headMu_;
};

/// Using this class as a base class will cause the given class to have all its
/// instances linked up in a list. The cost is 4 bytes per object, and some CPU
/// cost in the destructor (to walk the list).
template <class T> class LinkedObject
{
public:
    /// @return the subclass pointer next on the list (or nullptr if we're at
    /// the end of the list).
    T *link_next()
    {
        return static_cast<T *>(link_);
    }

    /// @return the subclass pointer of the beginning of the list.
    static T *link_head()
    {
        return static_cast<T *>(head_);
    }

    /// Locks the list for modification (at any entry!).
    static Atomic* head_mu() {
        return LinkedObjectHeadMutex<T>::headMu_.get();
    }

protected:
    /// @return the current subclass pointer.
    T *link_this()
    {
        return static_cast<T *>(this);
    }

    /// Constructor. Puts *this on the linked list.
    LinkedObject()
    {
        AtomicHolder h(head_mu());
        link_ = head_;
        head_ = link_this();
    }

    /// Constructor. Removes *this from the linked list.
    ~LinkedObject()
    {
        AtomicHolder h(head_mu());
        T **p = &head_;
        while (*p && *p != this)
        {
            p = &((*p)->LinkedObject<T>::link_);
        }
        if (*p == this)
        {
            *p = this->link_;
        }
        else
        {
            HASSERT(0);
        }
    }

    /// Linked list pointer.
    T *link_;
    /// Beginning of the list.
    static T *head_;
};

// static
template <class T> T *LinkedObject<T>::head_{nullptr};
template <class T> CreateOnlyAtomic LinkedObjectHeadMutex<T>::headMu_;

#endif // _UTILS_LINKEDOBJECT_HXX_
