/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file Notifiable.hxx
 * Basic building block of asynchronous operations: a zero-argument callback.
 *
 * @author Balazs Racz
 * @date 2 September 2013
 */

#ifndef _EXECUTOR_NOTIFIABLE_HXX_
#define _EXECUTOR_NOTIFIABLE_HXX_

#include <functional>

#include "os/OS.hxx"
#include "utils/Atomic.hxx"
#include "utils/Destructable.hxx"

/// An object that can schedule itself on an executor to run.
class Notifiable : public Destructable
{
public:
    /// Generic callback.
    virtual void notify() = 0;
#ifdef __FreeRTOS__
    virtual void notify_from_isr()
    {
        DIE("Unexpected call to notify_from_isr.");
    }
#endif
};

/// A Notifiable for synchronously waiting for a notification.
/// TODO(balazs.racz): We should make a syncnotifiable not need a semaphore
/// of itself, but rather use a thread-local semaphore.
class SyncNotifiable : public Notifiable
{
public:
    /// Constructor.
    SyncNotifiable() : sem_(0)
    {
    }

    /// Implementation of notification receive.
    void notify() override
    {
        sem_.post();
    }

#ifdef __FreeRTOS__
    /// Implementation of notification receive from a FreeRTOS interrupt
    /// context.
    void notify_from_isr() OVERRIDE
    {
        int woken = 0;
        sem_.post_from_isr(&woken);
    }
#endif

    /// Blocks the current thread until the notification is delivered.
    void wait_for_notification()
    {
        sem_.wait();
    }

private:
    /// Semaphore helping the implementation.
    OSSem sem_;

    DISALLOW_COPY_AND_ASSIGN(SyncNotifiable);
};

/// A Notifiable that doesn't do anything when notified.
class EmptyNotifiable : public Notifiable
{
public:
    /// Drops notification to the floor.
    void notify() override
    {
    }

    /// @return a static instance of EmptyNotifiable.
    static Notifiable* DefaultInstance();
};

/// A Notifiable that will crash whenever called.
class CrashNotifiable : public Notifiable
{
public:
    /// Crashes.
    void notify() override;

    /// @returns a static instance of CrashNotifiable.
    static Notifiable* DefaultInstance();
};

/** A class for reliably detecting whether a Notification has happened yet or
 * not.
 *
 * ProxyNotifiable can give a Notifiable callback, proxy to a parent Notifiable
 * any calls that may come in, and tell through {@link HasBeenNotified} whether
 * such a callback has happened yet or not.
 *
 */
class ProxyNotifiable : private Notifiable
{
public:
    ProxyNotifiable() : parent_(nullptr)
    {
    }
    /// Creates a new callback. When this callback is called, the parent
    /// notifiable is called and HasBeenNotified() will return true after that
    /// point. This function must not be called again until the returned
    /// callback is invoked.
    ///
    /// @param parent where to proxy notifications to.
    /// @return a Notifiable to be used as a done callback for some
    /// asynchronous processing.
    Notifiable* NewCallback(Notifiable* parent)
    {
        HASSERT(!parent_);
        parent_ = parent;
        return this;
    }
    /// @return true if the Notifiable returned by NewCallback has already been
    /// called.
    bool HasBeenNotified()
    {
        return !parent_;
    };

private:
    /// Implementation of the private Notifiable interface.
    void notify() override
    {
        Notifiable* p = parent_;
        HASSERT(p);
        parent_ = nullptr;
        p->notify();
    }

    /// Where to proxy notifications. If nullptr, then this was already
    /// notified and shall not be notified again.
    Notifiable* parent_;
};

/// A BarrierNotifiable allows to create a number of child Notifiable and wait
/// for all of them to finish. When the last one is finished, the parent done
/// callback is called.
class BarrierNotifiable : public Notifiable, private Atomic
{
public:
    /** Constructs a barrier notifiable that is done. Users should call reset()
     * later. */
    BarrierNotifiable() : count_(0), done_(nullptr)
    {
    }
    /** Constructs a barrier notifiable that is live. @param done will be
     * called when the barrier reaches zero (all children are notified and the
     * parent is notified). */
    BarrierNotifiable(Notifiable* done) : count_(1), done_(done)
    {
    }

    /// Resets the barrier. Returns &*this. Asserts that is_done().
    BarrierNotifiable* reset(Notifiable* done);

    ~BarrierNotifiable();

    /// Call this for each child task.
    BarrierNotifiable* new_child();
    /// When there are no more child tasks to add, call maybe_done(). Then once
    /// all previously added child tasks are done, the parent callback will be
    /// called. If you haven't added any children, this will call the parent
    /// callback inline.
    void maybe_done()
    {
        notify();
    }
    /// Implementation of the barrier semantics.
    void notify() override;

    /// @return true if the barrier condition is true, i.e., the owner has
    /// called maybe_done() and all children have called Done.
    bool is_done()
    {
        return !count_;
    }

    /// Checks if there is exactly one outstanding notification left in the
    /// barrier. It is required that the caller be actually holding on to a
    /// child. Effectively this tests whether there were either no children
    /// created, or all were notified inline.
    ///
    /// The use-case is that the owner of the barrier wants to figure out
    /// whether all children completed inline or not. If the children were
    /// completed inline, we may want to call the next phase immediately
    /// instead of yielding to the executor. For that the barrier must
    /// swallow the notification.
    ///
    /// @returns true, if there were exactly one outstanding notify pending,
    /// and in such case sets the barrier to done. Otherwise returns false and
    /// has no side-effect.
    bool abort_if_almost_done()
    {
        AtomicHolder h(this);
        HASSERT(!is_done());
        if (count_ == 1)
        {
            count_ = 0;
            return true;
        }
        else
        {
            return false;
        }
    }

private:
    /// How many outstanding notifications we are still waiting for. When 0,
    /// the barrier is not live; when reaches zero, done_ will be called.
    unsigned count_;
    /// Notifiable to call when the barrier reaches zero.
    Notifiable* done_;
};

/// Allocates a new barrier notifiable on the heap. The caller must free the
/// returned pointer when the done notifiable is called.
///
/// @param done is the notifiable to call when the barrier is completed.
/// @return a new heap-allocated barrier notifiable.
inline BarrierNotifiable* NewBarrierNotifiable(Notifiable* done)
{
    return new BarrierNotifiable(done);
}

/** This class sends a notification in its destructor. Use as RAII class:
 *
 * bool DoFoo(Notifiable* done)
 * {
 *    AutoNotify n(done);
 *    // ... doo stuuufff ...
 *    if (something_wrong) return false;
 *    // do more stuff
 *    return true;
 * }
 *
 * The notification will be called on all return statements. */
class AutoNotify
{
public:
    /// Constructor.
    ///
    /// @param n Notifiable to notify when *this goes out of scope. May be null
    /// in which case nothing will be notified.
    ///
    AutoNotify(Notifiable *n)
        : n_(n)
    {
    }

    /// Destructor. Notifies the stored notifiable.
    ~AutoNotify()
    {
        if (n_)
        {
            n_->notify();
        }
    }

    /** Transfers the ownership of the notification; it will NOT be called in
     * the destructor. The caller is now responsible for calling it.
     * @return the notification pointer stored in the constructor. */
    Notifiable* Transfer()
    {
        Notifiable* r = n_;
        n_ = nullptr;
        return r;
    }

private:
    /// Stored notifiable to notify upon destruction.
    Notifiable* n_;
};

/// If this error is presented, then the syntax of the AutoNotify was
/// incorrectly used:
///
/// WRONG:  AutoNotify(done);
/// RIGHT:  AutoNotify an(done);
///
/// Without the variable name the notification gets called immediately (since
/// the temporary C++ object of type AutoNotifiable gets destructed immediately
/// when the c++ statement fininshes), thus causing subtle concurrency bugs.
#define AutoNotify(l) int error_omitted_autonotify_holder_variable[-1]

/** A notifiable class that calls a particular function object once when it is
 * invoked, then deletes itself. */
class TempNotifiable : public Notifiable
{
public:
    /// Constructor. @param body is the function object that will be called
    /// when *this is notified, just before *this is deleted.
    TempNotifiable(std::function<void()> body)
        : body_(std::move(body))
    {
    }

    /// Calls the notification method.
    void notify() OVERRIDE
    {
        body_();
        delete this;
    }

private:
    /// Function object (callback) to call.
    std::function<void()> body_;
};

#endif // _EXECUTOR_NOTIFIABLE_HXX_
