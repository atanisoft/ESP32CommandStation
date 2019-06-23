/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file OS.hxx
 * This file represents a C++ language abstraction of common operating
 * system calls.
 *
 * @author Stuart W. Baker
 * @date 28 May 2012
 */

#ifndef _OS_OS_HXX_
#define _OS_OS_HXX_

#include <endian.h>

#include "utils/macros.h"
#include "os/os.h"

/** This class provides a threading API.
 */
class OSThread
{
public:
    /** Create a thread.
     * @param name name of thread, NULL for an auto generated name
     * @param priority priority of created thread, 0 means default,
     *        lower numbers means lower priority, higher numbers mean higher
     *        priority
     * @param stack_size size in bytes of the created thread's stack
     * @param start_routine entry point of the thread
     * @param arg entry parameter to the thread
     */
    OSThread(const char *name, int priority, size_t stack_size,
             void *(*start_routine)(void*), void *arg)
    {
        os_thread_create(&handle, name, priority, stack_size, start_routine, arg);
    }

    /** Creates a thread via inheritance. The user must overload the entry()
     * function and call the start() method whenever convenient. */
    OSThread()
        : handle(0)
    {
    }

   /** Starts the thread.  This call can be used when OSThread is inherited and
    * there is a virtual entry point.
    * @param name name of thread, NULL for an auto generated name
    * @param priority priority of created thread, 0 means default,
    *        lower numbers means lower priority, higher numbers mean higher
    *        priority
    * @param stack_size size in bytes of the created thread's stack
    */
    void start(const char *name, int priority, size_t stack_size)
    {
        os_thread_create(&handle, name, priority, stack_size, start, this);
    }

    /** Default destructor. */
    virtual ~OSThread()
    {
    }

    /** @return true if a thread was already created. */
    bool is_created()
    {
        return handle != 0;
    }

    /** Inherits the current thread. */
    void inherit()
    {
        HASSERT(!is_created());
        ScopedSetThreadHandle h(this);
        entry();
    }

    /** Return the current thread priority.  Depricated, use get_priority().
     * @param thread handle to thread of interest
     * @return current thread priority
     */
    static int getpriority(OSThread *thread)
    {
        return get_priority(thread);
    }

    /** Return the current thread priority.
     * @param thread handle to thread of interest
     * @return current thread priority
     */
    static int get_priority(OSThread *thread)
    {
        return os_thread_get_priority(thread->handle);
    }

    /** Get the minimum thread priority.
     * @return minimum trhead priority
     */
    static int get_priority_min()
    {
        return os_thread_get_priority_min();
    }

    /** Get the maximum thread priority.
     * @return maximum trhead priority
     */
    static int get_priority_max()
    {
        return os_thread_get_priority_max();
    }

    /// @return the thread handle for os_xxx operations.
    os_thread_t get_handle()
    {
        return handle;
    }

    /// Sets the thread handle to the current calling thread's. Needed for
    /// multiplexing environments like Arduino.
    void lock_to_thread()
    {
        HASSERT(handle == 0);
        handle = os_thread_self();
    }

    /// Resets the thread handle to none.
    void unlock_from_thread()
    {
        handle = 0;
    }

    /// Helper class for using lock_to_thread.
    struct ScopedSetThreadHandle
    {
        ScopedSetThreadHandle(OSThread *parent)
            : parent_(parent)
        {
            parent_->lock_to_thread();
        }

        ~ScopedSetThreadHandle()
        {
            parent_->unlock_from_thread();
        }

    private:
        OSThread *parent_;
    };

protected:
    /** User entry point for the created thread.
     * @return exit status
     */
    virtual void *entry()
    {
        HASSERT(0 && "forgot to overload OSThread entry point. This thread "
                     "would do nothing.");
        return NULL;
    }
    
private:
    /** Starting point for a new thread.
     * @param arg pointer to an OSThread instance
     * @return exit status
     */
    static void *start(void *arg)
    {
        OSThread *thread = (OSThread*)arg;
        return thread->entry();
    }

    DISALLOW_COPY_AND_ASSIGN(OSThread);

    /** Private thread handle. */
    os_thread_t handle;
};

/** This class provides support for one time initialization.
 */
class OSThreadOnce
{
public:
    /** One time intialization constructor.
     * @param routine method to call once
     */
    OSThreadOnce(void (*routine)(void))
        : handle(OS_THREAD_ONCE_INIT),
          routine(routine)
    {
    }
    
    /** call one time intialization routine
     * @return 0 upon success
     */
    int once(void)
    {
        return os_thread_once(&handle, routine);
    }

    /** Default Destructor */
    ~OSThreadOnce()
    {
    }

private:
    DISALLOW_COPY_AND_ASSIGN(OSThreadOnce);

    /** Private once handle. */
    os_thread_once_t handle;
    
    /** One time initialization routine */
    void (*routine)(void);
};

/** This class provides a counting semaphore API.
 */
class OSSem
{
public:
    /** Initialize a Semaphore.
     * @param value initial count
     */
    OSSem(unsigned int value = 0)
    {
        os_sem_init(&handle, value);
    }

    ~OSSem()
    {
        os_sem_destroy(&handle);
    }

    /** Post (increment) a semaphore.
     */
    void post()
    {
        os_sem_post(&handle);
    }


#if defined (__FreeRTOS__)
    /** Post (increment) a semaphore from ISR context.
     * @param woken is the task woken up
     */
    void post_from_isr(int *woken)
    {
        os_sem_post_from_isr(&handle, woken);
    }
#endif


    /** Wait on (decrement) a semaphore.
     */
    void wait()
    {
        os_sem_wait(&handle);
    }

#if !(defined(ESP_NONOS) || defined(ARDUINO))
    /** Wait on (decrement) a semaphore with timeout condition.
     * @param timeout timeout in nanoseconds, else OPENMRN_OS_WAIT_FOREVER to wait forever
     * @return 0 upon success, else -1 with errno set to indicate error
     */
    int timedwait(long long timeout)
    {
        return os_sem_timedwait(&handle, timeout);
    }
#endif

private:
    DISALLOW_COPY_AND_ASSIGN(OSSem);

    /** Private semaphore handle. */
    os_sem_t handle;
};

/** This class provides a simple message queue.
 */
class OSMQ
{
public:
    
    /** Constructor.
     * @param length number of items the queue can hold
     * @param item_size size of each item in bytes in the queue
     */
    OSMQ(size_t length, size_t item_size)
        : handle(os_mq_create(length, item_size))
    {
        HASSERT(handle != NULL);
    }
    
    /** Destructor.
     */
    ~OSMQ()
    {
        /** @todo (Stuart Baker) need a way to destroy a message queue */
    }
    
    /** Blocking send of a message to a queue.
     * @param data message to copy into queue
     */
    void send(const void *data)
    {
        os_mq_send(handle, data);
    }

    /** Send a message to a queue with a timeout.
     * @param data message to copy into queue
     * @param timeout time in nanoseconds to wait for queue to be able to accept message
     * @return OS_MQ_NONE on success, OS_MQ_TIMEDOUT on timeout
     */
    int timedsend(const void *data, long long timeout)
    {
        return os_mq_timedsend(handle, data, timeout);
    }
    
    /** Blocking receive a message from a queue.
     * @param data location to copy message from the queue
     */
    void receive(void *data)
    {
        os_mq_receive(handle, data);
    }
    
    /** Receive a message from a queue.
     * @param data location to copy message from the queue
     * @param timeout time in nanoseconds to wait for queue to have a message available
     * @return OS_MQ_NONE on success, OS_MQ_TIMEDOUT on timeout
     */
    int timedreceive(void *data, long long timeout)
    {
        return os_mq_timedreceive(handle, data, timeout);
    }

    /** Send of a message to a queue from ISR context.
     * @param data message to copy into queue
     * @param woken is the task woken up
     * @return OS_MQ_NONE on success, else OS_MQ_FULL
     */
    int send_from_isr(const void *data, int *woken)
    {
        return os_mq_send_from_isr(handle, data, woken);
    }

    /** Receive a message from a queue from ISR context.
     * @param data location to copy message from the queue
     * @param woken is the task woken up
     * @return OS_MQ_NONE on success, else OS_MQ_FULL
     */
    int receive_from_isr(void *data, int *woken)
    {
        return os_mq_receive_from_isr(handle, data, woken);
    }

    /** Return the number of messages pending in the queue.
     * @return number of messages in the queue
     */
    int num_pending()
    {
        return os_mq_num_pending(handle);
    }

    /** Return the number of spaces available.
     * @return number of spaces available
     */
    int num_spaces()
    {
        return os_mq_num_spaces(handle);
    }

    /** Return the number of messages pending in the queue from ISR context.
     * @return number of messages in the queue
     */
    int pending_from_isr()
    {
        return os_mq_num_pending_from_isr(handle);
    }

    /** Check if a queue is full from ISR context.
     * @return non-zero if the queue is full.
     */
    int is_full_from_isr()
    {
        return os_mq_is_full_from_isr(handle);
    }
    
private:
    /** Default Constructor.
     */
    OSMQ();

    /** Private message queue handle */
    os_mq_t handle;
    
    DISALLOW_COPY_AND_ASSIGN(OSMQ);
};

/** This class provides a mutex API.
 */
class OSMutex
{
public:
    /** Initialize a mutex.
     * @param recursive false creates a normal mutex, true creates a recursive mutex
     */
    OSMutex(bool recursive = false)
    {
        if (recursive)
        {
            os_recursive_mutex_init(&handle);
        }
        else
        {
            os_mutex_init(&handle);
        }
    }

    /** Lock a mutex.
     */
    void lock()
    {
        os_mutex_lock(&handle);
    }

    /** Unlock a mutex.
     */
    void unlock()
    {
        os_mutex_unlock(&handle);
    }

    /** Destructor */
    ~OSMutex()
    {
        os_mutex_destroy(&handle);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(OSMutex);

    friend class OSMutexLock;
    /** Private mutex handle. */
    os_mutex_t handle;
};

/**
 * Class to allow convenient locking and unlocking of mutexes in a C context.
 * The mutex will be automatically unlocked when the context is left, even if
 * there are multiple return, break or continue statements.
 *
 * Usage:
 *
 * void foo()
 * {
 *   // ...non-critical-section code here...
 *   {
 *     OSMutexLock locker(&mutex_);
 *     // ...critical section here...
 *     if (error) return;
 *     // ...more critical code here...
 *   }
 *   // ...at this point the mutex is unlocked...
 *   // ...more non-critical-section code here...
 * }
 * 
 */
class OSMutexLock
{
public:
    /// Constructor. @param mutex is the mutex to lock.
    OSMutexLock(OSMutex* mutex)
        : mutex_(&mutex->handle)
    {
        os_mutex_lock(mutex_);
    }

    /// Constructor. @param mutex is the mutex to lock.
    OSMutexLock(os_mutex_t* mutex)
        : mutex_(mutex)
    {
        os_mutex_lock(mutex_);
    }

    ~OSMutexLock()
    {
        os_mutex_unlock(mutex_);
    }
private:
    DISALLOW_COPY_AND_ASSIGN(OSMutexLock);

    /// Mutex we are having locked.
    os_mutex_t* mutex_;
};

/** This catches programming errors where you declare a mutex locker object
 *  without a name like this:
 *
 *  void foo()
 *  {
 *    OSMutexLock(&mutex_);
 *    // ...critical section here...
 *  }

 *  The above is valid C++, which creates a temporary OSMutexLock object, locks
 *  it, then immediately destructs the object, unlocking the mutex in the same
 *  line it was created. Thus the critical sections goes unprotected
 *  unintentionally.
 *
 *  The correct code is
 *
 *  void foo()
 *  {
 *    OSMutexLock locker(&mutex_);
 *    // ...critical section here...
 *  }
 *
 */
#define OSMutexLock(l) int error_omitted_mutex_lock_variable[-1]

/** Allows for OS abstracted access to time.
 */

extern "C"
{
extern long long rtcOffset;
}

class OSTime
{
public:
    /** Get the monotonic time since the system started.
     * @return time in nanoseconds since system start
     */
    static long long get_monotonic()
    {
        return os_get_time_monotonic();
    }

    /** Get the current RTC time.
     * @return time in nanoseconds of the RTC time.
     */
    static long long get_realtime()
    {
        /// @todo need to fill in the Real Time Clock infrastructure
        return get_monotonic() + rtcOffset;
    }

    /** Set the current RTC time.
     */
    static void set_realtime(long long time)
    {
        /// @todo need to fill in the Real Time Clock infrastructure
        rtcOffset = time - get_monotonic();
    }

private:
    DISALLOW_COPY_AND_ASSIGN(OSTime);

    /* Private default constructor prevents instantiating this class. */
    OSTime();

    /** Default destructor. */
    ~OSTime();
};

#if defined (__FreeRTOS__)
/** Event bit mask type */
typedef EventBits_t OSEventType;
/** Abstraction to a group of event bits that can support a masked pend.
 */
class OSEvent
{
public:
    /** types of wait test
     */
    enum Test
    {
        WAIT_ANY = 0, /**< wait for any of the bits in the mask to be set */
        WAIT_ALL, /**< wait for all of the bits in the mask to be set */
    };

    /** Constructor.
     */
    OSEvent()
        : event(xEventGroupCreate())
    {
        HASSERT(event);
    }

    /** Destructor.
     */
    ~OSEvent()
    {
        vEventGroupDelete(event);
    }

    /** Get the number of event bits in an event group.
     * @return number of event bits in an event group
     */
    static int number_of_bits()
    {
#if configUSE_16_BIT_TICKS
        return 8;
#else
        return 24;
#endif
    }

    /** Wait on (decrement) an event condition.
     * @param mask bitwise mask of interesting bits
     * @param value NULL if unused, else returns with the value of the event
                    buts at the time the condition held true.  On a timout,
                    value remains untouched.
     * @param clear true if upon return the bits called out in mask are to be
     *              cleared.  If a timeout occurs, no bits will be cleared.
     * @param test type of test on the mask bits
     * @return 0 upon success, else -1 with errno set to indicate error
     */
    int wait(OSEventType mask, OSEventType *value, bool clear, Test test)
    {
        return timedwait(mask, value, clear, test, OPENMRN_OS_WAIT_FOREVER);
    }

    /** Wait on (decrement) an event with timeout condition.
     * @param mask bitwise mask of interesting bits
     * @param value NULL if unused, else returns with the value of the event
                    buts at the time the condition held true.  On a timout,
                    value remains unchanged.
     * @param clear true if upon return the bits caled out in mask are to be
     *              cleared.  If a timeout occurs, no bits will be cleared.
     * @param test type of test on the mask bits
     * @param timeout timeout in nanoseconds, else OPENMRN_OS_WAIT_FOREVER to wait forever
     * @return 0 upon success, else -1 with errno set to indicate error
     */
    int timedwait(OSEventType mask, OSEventType *value, bool clear, Test test, long long timeout)
    {
        BaseType_t e_clear = clear ? pdTRUE : pdFALSE;
        BaseType_t e_test = test ? pdTRUE : pdFALSE;
        TickType_t e_timeout = timeout == OPENMRN_OS_WAIT_FOREVER ? portMAX_DELAY : timeout >> NSEC_TO_TICK_SHIFT;

        OSEventType bits = xEventGroupWaitBits(event, mask, e_clear, e_test, e_timeout);

        if (((bits & mask) == mask && test == WAIT_ALL) ||
            ((bits & mask) != 0    && test == WAIT_ANY))
        {
            // success
            if (value)
            {
                *value = bits;
            }
            return 0;
        }
        else
        {
            // timeout
            errno = ETIMEDOUT;
            return -1;
        }
    }

    /** Set event bits.
     * @param mask bitwise mask of interesting bits
     */
    void set(OSEventType mask)
    {
        xEventGroupSetBits(event, mask);
    }

    /** Set event bits from ISR context.
     * @param mask bitwise mask of interesting bits
     * @param woken is the task woken up
     */
    void set_from_isr(OSEventType mask, int *woken)
    {
        portBASE_TYPE local_woken;
        xEventGroupSetBitsFromISR(event, mask, &local_woken);
        *woken |= local_woken;
    }

    /** clear event bits.
     * @param mask bitwise mask of interesting bits
     */
    void clear(OSEventType mask)
    {
        xEventGroupClearBits(event, mask);
    }

    /** clear event bits from ISR context.
     * @param mask bitwise mask of interesting bits
     */
    void clear_from_isr(OSEventType mask)
    {
        xEventGroupClearBitsFromISR(event, mask);
    }

    /** Get the current value of the event bits
     * @return current value of the event bits
     */
    OSEventType peek()
    {
        return xEventGroupGetBits(event);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(OSEvent);

    /** handle to event object */
    EventGroupHandle_t event;
};
#elif defined(ARDUINO)

typedef uint32_t OSEventType;

extern "C" {
extern unsigned critical_nesting;
extern uint32_t SystemCoreClock;
}
#define cm3_cpu_clock_hz SystemCoreClock

#if !defined(ESP32)

#define portENTER_CRITICAL()                                                   \
    do                                                                         \
    {                                                                          \
        noInterrupts();                                                        \
        ++critical_nesting;                                                    \
    } while (0)
#define portEXIT_CRITICAL()                                                    \
    do                                                                         \
    {                                                                          \
        if (critical_nesting <= 1)                                             \
        {                                                                      \
            critical_nesting = 0;                                              \
            interrupts();                                                      \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            --critical_nesting;                                                \
        }                                                                      \
    } while (0)

#define configKERNEL_INTERRUPT_PRIORITY (0xa0)

#endif // ESP32

#endif  // freertos

#endif /* _OS_OS_HXX_ */
