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
 * \file Notifiable.cxx
 * Implementation of basic Notifiables.
 *
 * @author Balazs Racz
 * @date 2 September 2013
 */

#include "executor/Notifiable.hxx"

#include "utils/macros.h"

/// Default instance of an empty notifiable.
static EmptyNotifiable default_empty_notifiable;

/// @return Default instance of an empty notifiable.
Notifiable* EmptyNotifiable::DefaultInstance() {
  return &default_empty_notifiable;
}

/// Default instance of a crashing notifiable.
static CrashNotifiable default_crash_notifiable;

/// @return Default instance of a crashing notifiable.
Notifiable* CrashNotifiable::DefaultInstance() {
  return &default_crash_notifiable;
}

void CrashNotifiable::notify() { DIE("Called CrashNotifiable."); }

BarrierNotifiable* BarrierNotifiable::new_child() {
  AtomicHolder h(this);
  count_++;
  return this;
}

void BarrierNotifiable::notify() {
  unsigned new_value;
  {
    AtomicHolder h(this);
    HASSERT(count_ && "barrier notifyable received too many notifys");
    new_value = --count_;
  }
  if (!new_value) {
    HASSERT(done_);
    done_->notify();
  }
}

BarrierNotifiable::~BarrierNotifiable() { HASSERT(!count_); }

BarrierNotifiable* BarrierNotifiable::reset(Notifiable* done) {
  AtomicHolder h(this);
  HASSERT(!count_);
  count_ = 1;
  done_ = done;
  return this;
}
