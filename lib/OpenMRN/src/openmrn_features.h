/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file openmrn_features.h
 *
 * This file defines compilation-time configuration options for OpenMRN, which
 * are exclusively in the form of C-compatible macros. These control
 * conditional compilation on different operating systems.
 *
 * @author Balazs Racz
 * @date 24 February 2019
 */

#ifndef _INCLUDE_OPENMRN_FEATURES_
#define _INCLUDE_OPENMRN_FEATURES_


#ifdef __FreeRTOS__
/// Compiles the FreeRTOS event group based ::select() implementation.
#define OPENMRN_FEATURE_DEVICE_SELECT 1
/// Adds implementations for ::read ::write etc, with fd table.
#define OPENMRN_FEATURE_DEVTAB 1
/// Adds struct reent pointer to the FreeRTOS Task Priv structure and swaps it
/// in when the tasks are swapped in.
#define OPENMRN_FEATURE_REENT 1
#endif

/// @todo this should probably be a whitelist: __linux__ || __MACH__.
#if !defined(__FreeRTOS__) && !defined(__WINNT__) && !defined(ESP32) && !defined(ARDUINO) && !defined(ESP_NONOS)
/// Uses ::pselect in the Executor for sleep and pkill for waking up.
#define OPENMRN_HAVE_PSELECT 1
#endif

#if defined(__WINNT__) || defined(ESP32) || defined(ESP_NONOS)
/// Uses ::select in the executor to sleep (unsure how wakeup is handled)
#define OPENMRN_HAVE_SELECT 1
#endif

#if (defined(ARDUINO) && !defined(ESP32)) || defined(ESP_NONOS) ||             \
    defined(__EMSCRIPTEN__)
/// A loop() function is calling the executor in the single-threaded OS context.
#define OPENMRN_FEATURE_SINGLE_THREADED 1
#endif

#if defined(__FreeRTOS__) || defined(ESP32)
/// Use os_mutex_... implementation based on FreeRTOS mutex and semaphores.
#define OPENMRN_FEATURE_MUTEX_FREERTOS 1
#elif OPENMRN_FEATURE_SINGLE_THREADED
/// Add a fake implementation for os_mutex_lock that crashes if there is a
/// conflict.
#define OPENMRN_FEATURE_MUTEX_FAKE 1
#else
/// Use pthread_mutex for os_mutex implementation.
#define OPENMRN_FEATURE_MUTEX_PTHREAD 1
#endif

#if OPENMRN_FEATURE_MUTEX_FREERTOS || OPENMRN_FEATURE_MUTEX_PTHREAD || defined(__EMSCRIPTEN__)
/// Compile os_sem_timedwait functions.
#define OPENMRN_FEATURE_SEM_TIMEDWAIT 1
#endif



#endif // _INCLUDE_OPENMRN_FEATURES_
