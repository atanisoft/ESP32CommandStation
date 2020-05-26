/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file Uninitialized.hxx
 * Helper class for creating delayed initialized objects in static memory.
 *
 * @author Balazs Racz
 * @date 24 June 2017
 */

#ifndef _UTILS_UNINITIALIZED_HXX_
#define _UTILS_UNINITIALIZED_HXX_

#include <type_traits>

/// Template class that allows allocating storage for an object but not calling
/// its constructor.
///
/// The object shall be constructed before first use by calling emplace(). It
/// is the responsibility of the caller to also call the destructor sometime by
/// invoking reset(), otherwise the object will be leaked.
///
/// Using this class is inherently unsafe, so avoid if possible. After c++17
/// this class will be replaceable by std::optional.
template <class T>
class uninitialized
{
public:
    /// @return pointer to the embedded object
    const T *operator->() const
    {
        return tptr();
    }
    /// @return pointer to the embedded object.
    T *operator->()
    {
        return tptrm();
    }
    /// @return the embedded object
    T &operator*()
    {
        return *tptrm();
    }
    /// @return the embedded object
    const T &operator*() const
    {
        return *tptr();
    }
    /// @return the embedded object
    T &value()
    {
        return *tptrm();
    }
    /// @return the embedded object
    const T &value() const
    {
        return *tptr();
    };

    /// Gets the embedded object pointer in a way that is friendly to
    /// linker-initialization.
    /// NOTE: when switching to std::optional<>, calls to this function need to
    /// be replaced with calls to .operator->().
    /// @return mutable pointer to the embedded object
    constexpr T* get_mutable() const
    {
        return tptrm();
    }

    /// Gets the embedded object pointer in a way that is friendly to
    /// linker-initialization.
    /// NOTE: when switching to std::optional<>, calls to this function need to
    /// be replaced with calls to .operator->().
    /// @return const pointer to the embedded object
    constexpr const T* get() const
    {
        return tptr();
    }
    
    /// Constructs the embedded object.
    template <class... Args> T &emplace(Args &&... args)
    {
        new (this) T(std::forward<Args>(args)...);
        return *tptrm();
    }

    /// Destructs the embedded object.
    void reset()
    {
        tptr()->~T();
    }

    /// Public API to convert the pointer in a linker-initialized way.
    static constexpr T *cast_data(uninitialized<T> *parent)
    {
        return static_cast<T *>((void*)&parent->data);
    }

private:
    typename std::aligned_storage<sizeof(T), alignof(T)>::type data;

    /// @return the embedded object (mutable pointer)
    constexpr T *tptrm() const
    {
        return static_cast<T *>((void*)&data);
    }
    /// @return the embedded object
    constexpr const T *tptr() const
    {
        return static_cast<const T *>((void*)&data);
    }
};

#endif // _UTILS_UNINITIALIZED_HXX_
