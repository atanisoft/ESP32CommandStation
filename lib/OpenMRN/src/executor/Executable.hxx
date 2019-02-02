/** \copyright
 * Copyright (c) 2013-2014, Stuart W Baker and Balazs Racz
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
 * \file Executable.hxx
 *
 * Base class for work that can be scheduled on an Executor.
 *
 * @author Stuart W Baker and Balazs Racz
 * @date 22 May 2014
 */

#ifndef _EXECUTOR_EXECUTABLE_HXX_
#define _EXECUTOR_EXECUTABLE_HXX_

#include "executor/Notifiable.hxx"
#include "utils/QMember.hxx"

/// An object that can be scheduled on an executor to run.
class Executable : public Notifiable, public QMember
{
public:
    virtual ~Executable();

    void test_deletion();
    
    /** Entry point. This funciton will be called when *this gets scheduled on
     * the CPU. */
    virtual void run() = 0;

    /// Crashes the program -- everyone who is expecting notify calls must
    /// override this function. It is not virtual because not all
    /// implementation expect Notify calls.
    void notify() override {
        HASSERT(0 && "unexpected call to notify in Executable");
    }

    /** Return the result of an alloc_async() from a memory @ref Pool
     * @param item result of the the allocation
     */
    virtual void alloc_result(QMember *item)
    {
        HASSERT(0 && "unexpected call to alloc_result");
    }
};

/** A notifiable class that calls a particular function object once when it is
 * invoked, then deletes itself. */
class CallbackExecutable : public Executable
{
public:
    /// Constructor. @param body is the function object that will be called
    /// when *this is executed, just before *this is deleted.
    CallbackExecutable(std::function<void()> &&body)
        : body_(std::move(body))
    {
    }

    /// Calls the notification method.
    void run() override
    {
        body_();
        delete this;
    }

private:
    /// Function object (callback) to call.
    std::function<void()> body_;
};

#endif // _EXECUTOR_EXECUTABLE_HXX_
