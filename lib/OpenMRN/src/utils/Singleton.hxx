/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file Singleton.hxx
 *
 * Utility class to ensure a particular class has only a single object, and
 * access it via a static reference.
 *
 * @author Balazs Racz
 * @date 19 July 2013
 */

#ifndef _UTILS_SINGLETON_HXX_
#define _UTILS_SINGLETON_HXX_

#include "utils/macros.h"

/** Singleton class.

   By inheriting from this class you declare that the descendants will have
   only a single instance in the running binary. This will allow accessing the
   running instance without passing around a pointer to it.

   Usage:

   class Foo;

   class Foo: public Singleton<Foo> {
   ...as usual...
   };

   DEFINE_SINGLETON_INSTANCE(Foo);
   
   void appl_main() {
     Foo my_instance(constructor_args);
   }

   somewhere else:

   Foo::instance()->DoSomething();
 */
template<class T> class Singleton {
public:
  Singleton() {
    HASSERT(instance_ == nullptr);
    instance_ = static_cast<T*>(this);
  }

  ~Singleton() {
    instance_ = nullptr;
  }

    /// @return the singleton instance of this object.
  static T* instance() {
    HASSERT(instance_ != nullptr);
    return instance_;
  }

    /// @return true if there is a class of this singleton instantiated.
  static bool exists() {
    return (instance_ != nullptr);
  }

private:
    /// The singleton instance pointer.
  static T* instance_;
};

template<class T> T* Singleton<T>::instance_ = nullptr;

/// Helper macro the the customer may (but is not required to) use for ensuring
/// that the singleton instance is found by the linker. Must appear in a single
/// .cxx file.
/// @param T the class name that is a Singleton.
#define DEFINE_SINGLETON_INSTANCE(T) template<> T* Singleton<T>::instance_ = nullptr

#endif // _UTILS_SINGLETON_HXX_

