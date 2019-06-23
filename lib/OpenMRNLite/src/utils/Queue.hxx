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
 * \file Queue.hxx
 * This file provides an implementation queues.
 *
 * @author Stuart W. Baker
 * @date 3 August 2013
 */

#ifndef _UTILS_QUEUE_HXX_
#define _UTILS_QUEUE_HXX_

#include <cstdint>
#include <cstdlib>
#include <cstdarg>

#include "executor/Executable.hxx"
#include "executor/Notifiable.hxx"
#include "os/OS.hxx"
#include "utils/Atomic.hxx"
#include "utils/MultiMap.hxx"
#include "utils/QMember.hxx"
#include "utils/macros.h"

namespace openlcb
{
class AsyncIfTest;
}


/** Result of pulling an item from the queue based on priority.
 */
struct Result
{
    /** Defualt Constructor.
     */
    Result()
        : item(NULL)
        , index(0)
        {
        }

    /** Explicit initializer constructor.
     * @param item item presented in result
     * @param index index presented in result
     */
    Result(QMember *item, unsigned index)
        : item(item)
        , index(index)
        {
        }

    /** Default Destructor.
     */
    ~Result()
        {
        }

    QMember *item;  /**< item pulled from queue */
    unsigned index; /**< index of item pulled from queue */
};

class QResultHolder
{
public:
};

/** This class implements a linked list "queue" of buffers.  It may be
 * instantiated to use the mainBufferPool for its memory pool, or optionally
 * another BufferPool instance may be specified for its memory pool.
 */
class Q : private Atomic
{
public:
    /** Default Constructor.
     */
    Q()
        : head(NULL)
        , tail(NULL)
        , count(0)
    {
    }

    /** Default destructor.
     */
    ~Q()
    {
    }

    Atomic *lock()
    {
        return this;
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index unused parameter
     */
    void insert(QMember *item, unsigned index = 0)
    {
        HASSERT(item->next == nullptr);
        HASSERT(item != tail);
        AtomicHolder h(this);
        if (head == NULL)
        {
            head = tail = item;
        }
        else
        {
            tail->next = item;
            tail = item;
        }
        item->next = NULL;
        ++count;
    }

    /** Add an item to the back of the queue. Needs external locking.
     * @param item to add to queue
     * @param index unused parameter
     */
    void insert_locked(QMember *item, unsigned index = 0)
    {
        HASSERT(item->next == nullptr);
        HASSERT(item != tail);
        if (head == NULL)
        {
            head = tail = item;
        }
        else
        {
            tail->next = item;
            tail = item;
        }
        item->next = NULL;
        ++count;
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        return next().item;
    }

    /** Get an item from the front of the queue.
     * @return @ref Result structure with item retrieved from queue, NULL if
     *         no item available
     */
    Result next()
    {
        AtomicHolder h(this);
        return next_locked();
    }

    /** Get an item from the front of the queue.
     * @return @ref Result structure with item retrieved from queue, NULL if
     *         no item available
     */
    Result next_locked()
    {
        if (head == NULL)
        {
            return Result();
        }
        --count;
        QMember *qm = head;
        if (head == tail)
        {
            tail = NULL;
        }
        head = (qm->next);
        qm->next = NULL;

        return Result(qm, 0);
    }
    
    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return pending();
    };

    /** Get the number of pending items in the queue.
     * @return number of pending items in the queue
     */
    size_t pending()
    {
        return count;
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return empty();
    }

    /** Test if the queue is empty.
     * @return true if empty, else false
     */
    bool empty()
    {
        return (head == NULL);
    }

private:
    /** head item in queue */
    QMember *head;

    /** tail item in queue */
    QMember *tail;

    /** number of items in queue */
    size_t count;

    DISALLOW_COPY_AND_ASSIGN(Q);
};

/** Asynchronous specialization of @ref Q.
 */
class QAsync
{
public:
    /** Default Constructor.
     */
    QAsync()
        : waiting(true)
    {
    }

    /** Default destructor.
     */
    ~QAsync()
    {
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index unused parameter
     */
    void insert(QMember *item, unsigned index = 0)
    {
        Executable *executable = NULL;
        {
            AtomicHolder h(impl_.lock());
            if (waiting)
            {
                if (impl_.empty())
                {
                    waiting = false;
                    impl_.insert(item);
                }
                else
                {
                    executable = static_cast<Executable *>(impl_.next().item);
                }
            }
            else
            {
                impl_.insert(item);
            }
        }
        if (executable)
        {
            executable->alloc_result(item);
        }
    }

    /** Get an item from the front of the queue.
     * @param flow Executable that will wait on the item
     * @return item retrieved from queue, NULL if no item available
     */
    void next_async(Executable *flow)
    {
        QMember *qm = NULL;
        {
            AtomicHolder h(impl_.lock());
            if (waiting)
            {
                impl_.insert(flow);
            }
            else
            {
                qm = impl_.next(0);
                if (qm == NULL)
                {
                    impl_.insert(flow);
                    waiting = true;
                }
            }
        }
        if (qm)
        {
            flow->alloc_result(qm);
        }
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        return next().item;
    }

    /** Get an item from the front of the queue.
     * @return @ref Result structure with item retrieved from queue, NULL if
     *         no item available
     */
    Result next()
    {
        AtomicHolder h(impl_.lock());
        return waiting ? Result() : impl_.next();
    }

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return pending();
    };

    /** Get the number of pending items in the queue.
     * @return number of pending items in the queue
     */
    size_t pending()
    {
        AtomicHolder h(impl_.lock());
        return waiting ? 0 : impl_.pending();
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return empty();
    }

    /** Test if the queue is empty.
     * @return true if empty, else false
     */
    bool empty()
    {
        AtomicHolder h(impl_.lock());
        return waiting ? true : impl_.empty();
    }

private:
    /** true if someone is waiting for an insertion */
    bool waiting;

    /** Implementation helper. */
    Q impl_;

    DISALLOW_COPY_AND_ASSIGN(QAsync);
};

/// Strongly typed queue class with asynchronous access. 
template <class T> class TypedQAsync : public QAsync
{
public:
    /** @return the next item from the queue. If the queue is currently empty,
     * then blocks the current thread until an item is available. MUST NOT BE
     * CALLED ON INTERFACE EXECUTORS. */
    T *next_blocking()
    {
        return BlockingWait(this).result();
    }

    /// Inserts an entry at the end of the queue.
    void typed_insert(T* entry) {
        insert(entry);
    }

    /// @todo(balazs.racz): add a typed next() command here.

private:
    /// Helper class for waiting (blocking the current thread) until a message
    /// in the queue shows up.
    class BlockingWait : public Executable
    {
    public:
        /// Constructor. @param parent is the queue to take an entry from.
        BlockingWait(TypedQAsync<T> *parent)
        {
            parent->next_async(this);
            n_.wait_for_notification();
        }

        /// @return (typed) result of the queue wait operation.
        T *result()
        {
            return result_;
        }

    private:
        void alloc_result(QMember *item) OVERRIDE
        {
            result_ = static_cast<T *>(item);
            n_.notify();
        }

        void run() OVERRIDE
        {
            DIE("Unexpected call to Run() in BlockingWait");
        }

    private:
        /// helps blocking the calling thread until the allocation is complete.
        SyncNotifiable n_;
        /// Response of the allocation.
        T *result_;
    };
};

/** A list of queues.  Index 0 is the highest priority queue with increasingly
 * higher indexes having increasingly lower priority.
 */
template <unsigned ITEMS> class QList
{
public:
    /** Default Constructor.
     */
    QList()
        : list()
    {
    }

    /** Destructor.
     */
    ~QList()
    {
    }

    typedef ::Result Result;

    Atomic *lock()
    {
        return list[0].lock();
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index in the list to operate on
     */
    void insert(QMember *item, unsigned index)
    {
        AtomicHolder h(lock());
        insert_locked(item, index);
    }

    /** Add an item to the back of the queue. Needs external locking.
     * @param item to add to queue
     * @param index in the list to operate on
     */
    void insert_locked(QMember *item, unsigned index)
    {
        if (index >= ITEMS)
        {
            index = ITEMS - 1;
        }
        list[index].insert_locked(item);
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        AtomicHolder h(lock());
        return list[index].next_locked().item;
    }

    /** Get an item from the front of the queue queue in priority order.
     * @return item retrieved from queue + index, NULL if no item available
     */
    Result next()
    {
        AtomicHolder h(lock());
        return next_locked();
    }

    Result next_locked()
    {
        for (unsigned i = 0; i < ITEMS; ++i)
        {
            QMember *result = list[i].next_locked().item;
            if (result)
            {
                return Result(result, i);
            }
        }
        return Result();
    }

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return list[index].pending();
    }

    /** Get the total number of pending items in all queues in the list.
     * @return number of total pending items in all queues in the list
     */
    size_t pending()
    {
        // @todo we probably don't want to keep locking and unlocking all of
        // the queues here. There should probably be only one lock for this
        // entire iteration.
        size_t result = 0;
        for (unsigned i = 0; i < ITEMS; ++i)
        {
            result += list[i].pending();
        }
        return result;
    }

    /// @return how many entries are enqueued right now (across all lists).
    size_t size()
    {
        return pending();
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return list[index].empty();
    }

    /** Test if all the queues are empty.
     * @return true if empty (all lists), else false
     */
    bool empty()
    {
        AtomicHolder h(lock());
        for (unsigned i = 0; i < ITEMS; ++i)
        {
            if (!list[i].empty())
            {
                return false;
            }
        }
        return true;
    }

private:
    /** the list of queues */
    Q list[ITEMS];

    DISALLOW_COPY_AND_ASSIGN(QList);
};

/** A list of queues.
 */
template<unsigned items> using QListProtected = QList<items>;


#if 0
/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.
 */
template <class T> class QueueWait : public Q <T>, public OSSem
{
public:
    /** Default Constructor. */
    QueueWait()
        : Q<T>(),
          OSSem(0)
    {
    }

    /** Default destructor.
     */
    ~QueueWait()
    {
    }

    /** Add an item to the back of the queue.
     * @param item item to add to queue
     */
    void insert(T *item)
    {
        Q<T>::insert(item);
        post();
    }

    /** Get an item from the front of the queue.
     * @return item retrieved from queue
     */
    T *next()
    {
        T *result = Q<T>::next();
        if (result != NULL)
        {
            /* decrement semaphore */
            OSSem::wait();
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    T *wait()
    {
        OSSem::wait();
        T *result = Q<T>::next();
        if(result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR woken up asynchronously
     */
    T *timedwait(long long timeout)
    {
        if (OSSem::timedwait(timeout) != 0)
        {
            errno = ETIMEDOUT;
            return NULL;
        }

        T *result = Q<T>::next();
        if (result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wakeup anyone waiting on the wait queue.
     */
    void wakeup()
    {
        post();
    }

private:

    DISALLOW_COPY_AND_ASSIGN(QueueWait);
};

/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.
 */
template <class T> class QueueProtectedWait : public QProtected <T>, public OSSem
{
public:
    /** Default Constructor. */
    QueueProtectedWait()
        : QProtected<T>(),
          OSSem(0)
    {
    }

    /** Default destructor.
     */
    ~QueueProtectedWait()
    {
    }

    /** Add an item to the back of the queue.
     * @param item item to add to queue
     */
    void insert(T *item)
    {
        QProtected<T>::insert(item);
        post();
    }

#ifdef __FreeRTOS__
    /** Add an item to the back of the queue, callable from interrupt context.
     * @param item item to add to queue
     */
    void insert_from_isr(T *item)
    {
        QProtected<T>::insert_locked(item);
        post_from_isr();
    }
#endif

    /** Get an item from the front of the queue.
     * @return item retrieved from queue
     */
    T *next()
    {
        T *result = QProtected<T>::next();
        if (result != NULL)
        {
            /* decrement semaphore */
            OSSem::wait();
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    T *wait()
    {
        OSSem::wait();
        T *result = QProtected<T>::next();
        if(result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR - woken up asynchronously
     */
    T *timedwait(long long timeout)
    {
        if (OSSem::timedwait(timeout) != 0)
        {
            errno = ETIMEDOUT;
            return NULL;
        }

        T *result = QProtected<T>::next();
        if (result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wakeup someone waiting on the wait queue.
     */
    void wakeup()
    {
        post();
    }

private:
    DISALLOW_COPY_AND_ASSIGN(QueueProtectedWait);
};

#endif

/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.  The priority of pulling items out of
 * of the list is fixed to look at index 0 first and the highest index last.
 */
template <unsigned items>
class QListProtectedWait : public QListProtected<items>, public OSSem
{
public:
    /** Default Constructor.
     */
    QListProtectedWait()
        : QListProtected<items>()
        , OSSem(0)
    {
    }

    /** Default destructor.
     */
    ~QListProtectedWait()
    {
    }

    /** Add an item to the back of the queue.
     * @param item item to add to queue
     * @param index in the list to operate on
     */
    void insert(QMember *item, unsigned index)
    {
        QListProtected<items>::insert(item, index);
        post();
    }

#ifdef __FreeRTOS__
    /** Add an item to the back of the queue.
     * @param item item to add to queue
     * @param index in the list to operate on
     */
    void insert_from_isr(QMember *item, unsigned index)
    {
        int woken = 0;
        QListProtected<items>::insert_locked(item, index);
        this->post_from_isr(&woken);
    }
#endif

    /** Translate the Result type */
    typedef typename QListProtected<items>::Result Result;

    /** Get an item from the front of the queue.
     * @return item retrieved from one of the queues
     */
    Result next()
    {
        Result result = QListProtected<items>::next();
        if (result.item != NULL)
        {
            /* decrement semaphore */
            OSSem::wait();
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    Result wait()
    {
        OSSem::wait();
        Result result = QListProtected<items>::next();
        if (result.item == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

#if !(defined(ESP_NONOS) || defined(ARDUINO))
    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR - woken up asynchronously
     */
    Result timedwait(long long timeout)
    {
        if (OSSem::timedwait(timeout) != 0)
        {
            errno = ETIMEDOUT;
            return {NULL, 0};
        }

        Result result = QListProtected<items>::next();
        if (result.item == NULL)
        {
            errno = EINTR;
        }
        return result;
    }
#endif

    /** Wakeup anyone waiting on the wait queue.
     */
    void wakeup()
    {
        post();
    }

private:
    DISALLOW_COPY_AND_ASSIGN(QListProtectedWait);
};

#endif /* _UTILS_QUEUE_HXX_ */
