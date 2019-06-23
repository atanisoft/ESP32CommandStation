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
 * \file logging.h
 * Facility to do debug printf's on a configurable loglevel.
 *
 * @author Balazs Racz
 * @date 3 August 2013
 */

#ifndef _UTILS_LOGGING_H_
#define _UTILS_LOGGING_H_

#ifndef __cplusplus
#ifndef __STDC_VERSION__
/// Hack to make vsnprintf show up.
#define __STDC_VERSION__ 199901L
#endif
#endif

#include <stdio.h>
#include <inttypes.h>
#include "os/os.h"

/// Loglevel that kills the current process.
static const int FATAL = 0;
/// Loglevel that is always printed, reporting an error.
static const int LEVEL_ERROR = 1;
/// Loglevel that is always printed, reporting a warning or a retryable error.
static const int WARNING = 2;
/// Loglevel that is printed by default, reporting some status information.
static const int INFO = 3;
/// Loglevel that is usually not printed, reporting debugging information.
static const int VERBOSE = 4;

#if defined(__linux__) || defined(__MACH__) || defined(GCC_ARMCM3) || defined(GCC_ARMCM0)
#define LOCKED_LOGGING
#endif

#ifdef LOCKED_LOGGING
extern os_mutex_t g_log_mutex;
#define LOCK_LOG os_mutex_lock(&g_log_mutex)
#define UNLOCK_LOG os_mutex_unlock(&g_log_mutex)
#else
/// Called before starting to use the logging buffer.
#define LOCK_LOG
/// Called after the log buffer is flushed.
#define UNLOCK_LOG
#endif

#ifdef __cplusplus
#define GLOBAL_LOG_OUTPUT ::log_output
#else
/// Used for writing log buffersto the log output. Hack to make this compile
/// under C and C++ both.
#define GLOBAL_LOG_OUTPUT log_output
#endif

#ifdef __FreeRTOS__
#define LOG_MAYBE_DIE(level) (level == FATAL)
#else
/// Splits the death behavior of FreeRTOS (where we just blink) from everything
/// else (where we actually print a death message).
#define LOG_MAYBE_DIE(level) 0
#endif

/// Conditionally write a message to the logging output.
/// @param level is the log level; if the configured loglevel is smaller, then
/// the log is not printed, not rendered, and the rendering code is never even
/// compiled. This makes it cheap to have LOG(VERBOSE, ...) messages left in
/// the code everywhere.
/// @param message is a printf format argument and possibly more arguments that
/// are referenced from the printf format.
#define LOG(level, message...)                                                 \
    do                                                                         \
    {                                                                          \
        if (LOG_MAYBE_DIE(level))                                              \
        {                                                                      \
            DIE("log fatal");                                                  \
        }                                                                      \
        else if (level == FATAL)                                               \
        {                                                                      \
            fprintf(stderr, message);                                          \
            fprintf(stderr, "\n");                                             \
            abort();                                                           \
        }                                                                      \
        else if (LOGLEVEL >= level)                                            \
        {                                                                      \
            LOCK_LOG;                                                          \
            int sret = snprintf(logbuffer, sizeof(logbuffer), message);        \
            if (sret > (int)sizeof(logbuffer))                                 \
                sret = sizeof(logbuffer);                                      \
            GLOBAL_LOG_OUTPUT(logbuffer, sret);                                \
            UNLOCK_LOG;                                                        \
        }                                                                      \
    } while (0)

/// Shorthand for LOG(LEVEL_ERROR, message...). See @ref LOG.
#define LOG_ERROR(message...) LOG(LEVEL_ERROR, message)

#if defined(__linux__) || defined(__MACH__)
extern char logbuffer[4096];
#else
/// Temporary buffer to sprintf() the log lines into.
extern char logbuffer[256];
#endif

#ifndef LOGLEVEL
#ifdef __FreeRTOS__
#define LOGLEVEL FATAL
#else
/// Default loglevel.
#define LOGLEVEL INFO
#endif // not FreeRTOS
#endif // ifndef LOGLEVEL

#ifdef __cplusplus
extern "C" {
#endif
/// Prints a line of log to the log output destination.
///
/// @param buf is the logging buffer. Guaranteed to be available until this
/// function returns.
/// @param size is now many bytes are there in the logging buffer. There is
/// never a terminating \n in the log buffer. There is a terminating zero at
/// buf[size].
void log_output(char *buf, int size);
/// Prints an error message about errno to std error and terminates the current
/// program.
///
/// @param where C-style string describing where the error has happened. Will
/// be printed too.
///
void print_errno_and_exit(const char *where);
#ifdef __cplusplus
}
#endif

/// Calls the function x, and if the return value is negative, prints errno as
/// error message to stderr and exits the current program.
/// @param where is a string, describing the operation that's being executed.
/// @param x is an expression, usually a system function call with arguments
/// that needsto be run and the return value checked.
///
/// example usage:
///   ERRNOCHECK("file close", close(fd_)); 
#define ERRNOCHECK(where, x...)                                                \
    do                                                                         \
    {                                                                          \
        if ((x) < 0)                                                           \
        {                                                                      \
            print_errno_and_exit(where);                                       \
        }                                                                      \
    } while (0)

#endif // _UTILS_LOGGING_H_
