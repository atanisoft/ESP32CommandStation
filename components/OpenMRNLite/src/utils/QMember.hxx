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
 * \file QMember.hxx
 * Base class for objects that can be enqueued.
 *
 * @author Stuart W. Baker
 * @date 10 March 2014
 */

#ifndef _UTILS_QMEMBER_HXX_
#define _UTILS_QMEMBER_HXX_

#include "utils/macros.h"

/** Essentially a "next" pointer container.
 */
class QMember
{
public:
    /** Initiailize a QMember, in place of a public placement construction.
     */
    void init()
    {
        next = NULL;
    }

protected:
    /** Constructor.
     */
    QMember() : next(NULL)
    {
    }

    /** Destructor.
     */
    ~QMember()
    {
    }

    /** pointer to the next member in the queue */
    QMember *next;

    /** This class is a helper of Q */
    friend class Q;
    /** This class is a helper of SimpleQueue */
    friend class SimpleQueue;
    /** ActiveTimers needs to iterate through the queue. */
    friend class ActiveTimers;
    /** ActiveTimers needs to iterate through the queue. */
    friend class ExecutorBase;
    friend class TimerTest;
};

#endif /* _UTILS_QMEMBER_HXX_ */
