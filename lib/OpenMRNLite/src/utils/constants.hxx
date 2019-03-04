/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file constants.hxx
 * Utility to specify linking-time constants and overrides for them.
 *
 * @author Balazs Racz
 * @date 30 Apr 2014
 */

#ifndef _UTILS_CONSTANTS_HXX_
#define _UTILS_CONSTANTS_HXX_

#include <stddef.h>

#ifdef __cplusplus
#define EXTERNC extern "C" {
#define EXTERNCEND }
#else
/// Allows both C and C++ compilation
#define EXTERNC
/// Allows both C and C++ compilation
#define EXTERNCEND
#endif

#if defined (__EMSCRIPTEN__) || defined (__MACH__) || defined(__linux__)
#define NEED_SIMPLE_CONST
#endif

#ifdef NEED_SIMPLE_CONST

/// Declares a constant value. Put this into a header and include that header
/// to the code which has to access that constant.
///
/// @param name name of the constant. For a name NNN Creates a function called
/// config_NNN() that returns the configured value.
#define DECLARE_CONST(name)                                                    \
    EXTERNC extern const int _sym_##name;                                      \
    EXTERNCEND                                                                 \
    static inline int config_##name(void)                                      \
    {                                                                          \
        return _sym_##name;                                                    \
    }                                                                          \
    /** internal guard */                                                      \
    typedef unsigned char                                                      \
        _do_not_add_declare_and_default_const_to_the_same_file_for_##name;

/// Defines the default value of a constant. Use this is a single .cxx file and
/// make sure NOT to include the header that has the respective DECLARE_CONST
/// macros. Best not to incude anything at all.
///
/// @param name name of the constant.
/// @param value is what the default value should be.
#define DEFAULT_CONST(name, value)                                             \
    EXTERNC extern const int __attribute__((__weak__)) _sym_##name = value;    \
    EXTERNCEND                                                                 \
    /** internal guard */                                                      \
    typedef signed char                                                        \
        _do_not_add_declare_and_default_const_to_the_same_file_for_##name;

/// Overrides the value of a constant. Use this is a single .cxx file (usually
/// main.cxx).
///
/// @param name name of the constant.
/// @param value is what the actual value should be.
#define OVERRIDE_CONST(name, value)                                            \
    EXTERNC extern const int _sym_##name;                                      \
    const int _sym_##name = value;                                             \
    EXTERNCEND

#else  // native C

#define DECLARE_CONST(name)                                                    \
    EXTERNC extern void _sym_##name(void);                                     \
    EXTERNCEND typedef unsigned char                                           \
    _do_not_add_declare_and_default_const_to_the_same_file_for_##name;         \
    static inline ptrdiff_t config_##name(void)                                \
    {                                                                          \
        return (ptrdiff_t)(&_sym_##name);                                      \
    }

#define DEFAULT_CONST(name, value)                                             \
    typedef signed char                                                        \
    _do_not_add_declare_and_default_const_to_the_same_file_for_##name;         \
    asm(".global _sym_" #name " \n");                                          \
    asm(".weak _sym_" #name " \n");                                            \
    asm(".set _sym_" #name ", " #value " \n");

#define OVERRIDE_CONST(name, value)                                            \
    asm(".global _sym_" #name " \n");                                          \
    asm(".set _sym_" #name ", " #value " \n");

#endif // native C

/// We cannot compare constants to zero, so we use 1 and 2 as constant values
/// for booleans.
#define CONSTANT_TRUE 1
/// We cannot compare constants to zero, so we use 1 and 2 as constant values
/// for booleans.
#define CONSTANT_FALSE 2

/// Sets the default value of a boolean constant to true.
/// @param name is the name of the constant to set.
#define DEFAULT_CONST_TRUE(name) DEFAULT_CONST(name, 1)
/// Sets the default value of a boolean constant to false.
/// @param name is the name of the constant to set.
#define DEFAULT_CONST_FALSE(name) DEFAULT_CONST(name, 2)

/// Overrides the value of a boolean constant to true.
/// @param name is the name of the constant to set.
#define OVERRIDE_CONST_TRUE(name) OVERRIDE_CONST(name, 1)
/// Overrides the value of a boolean constant to false.
/// @param name is the name of the constant to set.
#define OVERRIDE_CONST_FALSE(name) OVERRIDE_CONST(name, 2)

#endif // _UTILS_CONSTANTS_HXX_
