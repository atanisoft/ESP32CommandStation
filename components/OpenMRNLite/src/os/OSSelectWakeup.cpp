/** \copyright
* Copyright (c) 2015, Balazs Racz
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
* \file OSSelectWakeup.cxx
* Helper class for portable wakeup of a thread blocked in a select call.
*
* @author Balazs Racz
* @date 10 Apr 2015
*/

#include "os/OSSelectWakeup.hxx"
#include "utils/logging.h"
#if defined(__MACH__)
#define _DARWIN_C_SOURCE // pselect
#endif

void empty_signal_handler(int)
{
}

int OSSelectWakeup::select(int nfds, fd_set *readfds,
                           fd_set *writefds, fd_set *exceptfds,
                           long long deadline_nsec)
{
    {
        AtomicHolder l(this);
        inSelect_ = true;
        if (pendingWakeup_)
        {
            deadline_nsec = 0;
        }
        else
        {
#if OPENMRN_FEATURE_DEVICE_SELECT
            Device::select_clear();
#endif
        }
    }
#if OPENMRN_FEATURE_DEVICE_SELECT
    int ret =
        Device::select(nfds, readfds, writefds, exceptfds, deadline_nsec);
    if (!ret && pendingWakeup_)
    {
        ret = -1;
        errno = EINTR;
    }
#elif OPENMRN_HAVE_PSELECT
    struct timespec timeout;
    timeout.tv_sec = deadline_nsec / 1000000000;
    timeout.tv_nsec = deadline_nsec % 1000000000;
    int ret =
        ::pselect(nfds, readfds, writefds, exceptfds, &timeout, &origMask_);
#elif OPENMRN_HAVE_SELECT
#ifdef ESP32
    fd_set newexcept;
    if (!exceptfds)
    {
        FD_ZERO(&newexcept);
        exceptfds = &newexcept;
    }
    FD_SET(vfsFd_, exceptfds);
    if (vfsFd_ >= nfds)
    {
        nfds = vfsFd_ + 1;
    }
#endif //ESP32
    struct timeval timeout;
    // divide deadline_nsec by 1000 to prevent overflow on esp32
    timeout.tv_sec = (deadline_nsec / 1000) / 1000000ULL;
    timeout.tv_usec = (deadline_nsec / 1000) % 1000000ULL;
    int ret =
        ::select(nfds, readfds, writefds, exceptfds, &timeout);
#elif !defined(OPENMRN_FEATURE_SINGLE_THREADED)
    #error no select implementation in multi threaded OS.
#else
    // Single threaded OS: nothing to wake up.
    int ret = 0;
#endif
    {
        AtomicHolder l(this);
        pendingWakeup_ = false;
        inSelect_ = false;
    }
    return ret;
}

#ifdef ESP32
#include "freertos_includes.h"

#include <esp_system.h>
#include <esp_vfs.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

/// Protects the initialization of vfs_id.
static pthread_once_t vfs_init_once = PTHREAD_ONCE_INIT;

/// This per-thread key will store the OSSelectWakeup object that has been
/// locked to any given calling thread.
static pthread_key_t select_wakeup_key;

/// This is the VFS FD that will be returned for ::open().
///
/// NOTE: The ESP VFS layer will ensure uniqueness and pass this FD back into
/// all VFS APIs we implement.
static constexpr int WAKEUP_VFS_FD = 0;

/// This function is called by the ESP32's select implementation. It is passed
/// in as a function pointer to the VFS API.
/// @param nfds see standard select API
/// @param readfds see standard select API
/// @param writefds see standard select API
/// @param exceptfds see standard select API
/// @param signal_sem is the semaphore object to trigger when the select has
/// completed.
/// @param end_select_args are the arguments to pass to end_select upon wakeup.
static esp_err_t esp_start_select(int nfds, fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, esp_vfs_select_sem_t signal_sem, void **end_select_args)
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
    LOG(VERBOSE, "esp start select %p (thr %p parent %p)", signal_sem.sem
      , os_thread_self(), parent);
    if (nfds >= 1 &&
        (FD_ISSET(WAKEUP_VFS_FD, readfds) ||
         FD_ISSET(WAKEUP_VFS_FD, writefds) ||
         FD_ISSET(WAKEUP_VFS_FD, exceptfds)))
    {
        parent->esp_start_select(signal_sem, readfds, writefds, exceptfds);
    }
    return ESP_OK;
}

/// This function is called by the ESP32's select implementation.
/// @param signal_sem is the semaphore container provided by the VFS layer that
/// can be used to wake up the select() call early.
void OSSelectWakeup::esp_start_select(esp_vfs_select_sem_t signal_sem,
    fd_set *readfds, fd_set *writefds, fd_set *exceptfds)
{
    AtomicHolder h(this);
    espSem_ = signal_sem;
    readFds_ = readfds;
    origReadFds_ = *readfds;
    writeFds_ = writefds;
    origWriteFds_ = *writefds;
    exceptFds_ = exceptfds;
    origExceptFds_ = *exceptfds;
    semValid_ = true;
}

static esp_err_t esp_end_select(void *arg)
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
    LOG(VERBOSE, "esp end select (thr %p parent %p)", os_thread_self()
      , parent);
    parent->esp_end_select();
    return ESP_OK;
}

void OSSelectWakeup::esp_end_select()
{
    AtomicHolder h(this);
    semValid_ = false;
}

/// This function is called by the ESP32 VFS layer when a file is opened under
/// the registered VFS root.
/// @param path see standard open API.
/// @param flags see standard open API.
/// @param mode see standard open API.
/// @return the FD for the opened file.
static int esp_wakeup_open(const char * path, int flags, int mode)
{
    // This virtual FS has only one fd, 0.
    return WAKEUP_VFS_FD;
}

/// This function will trigger the ESP32 to wake up from any pending select()
/// call.
void OSSelectWakeup::esp_wakeup()
{
    AtomicHolder h(this);
    if (semValid_)
    {
        LOG(VERBOSE, "wakeup es %p %u", espSem_.sem, *(unsigned*)espSem_.sem);
        if (FD_ISSET(vfsFd_, &origReadFds_))
        {
            FD_SET(vfsFd_, readFds_);
        }
        if (FD_ISSET(vfsFd_, &origWriteFds_))
        {
            FD_SET(vfsFd_, writeFds_);
        }
        if (FD_ISSET(vfsFd_, &origExceptFds_))
        {
            FD_SET(vfsFd_, exceptFds_);
        }
        esp_vfs_select_triggered(espSem_);
    }
}

/// This function will trigger the ESP32 to wake up from any pending select()
/// call from within an ISR context.
void OSSelectWakeup::esp_wakeup_from_isr()
{
    AtomicHolder h(this);
    if (semValid_)
    {
        BaseType_t woken = pdFALSE;
        if (FD_ISSET(vfsFd_, &origReadFds_))
        {
            FD_SET(vfsFd_, readFds_);
        }
        if (FD_ISSET(vfsFd_, &origWriteFds_))
        {
            FD_SET(vfsFd_, writeFds_);
        }
        if (FD_ISSET(vfsFd_, &origExceptFds_))
        {
            FD_SET(vfsFd_, exceptFds_);
        }
        esp_vfs_select_triggered_isr(espSem_, &woken);
        if (woken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
}

/// Registers the VFS driver that is used for waking up the ESP32 from a call
/// to select() within the Executor.
static void esp_vfs_init()
{
    esp_vfs_t vfs;
    bzero(&vfs, sizeof(esp_vfs_t));
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    vfs.start_select = esp_start_select;
    vfs.end_select = esp_end_select;
    vfs.open = esp_wakeup_open;
    ESP_ERROR_CHECK(esp_vfs_register("/dev/wakeup", &vfs, nullptr));
    HASSERT(0 == pthread_key_create(&select_wakeup_key, nullptr));
}

/// Allocates an FD from the VFS layer to this thread for waking it up from
/// select().
///
/// Note: this will register the VFS driver if this is the first time this
/// function has been called.
void OSSelectWakeup::esp_allocate_vfs_fd()
{
    HASSERT(0 == pthread_once(&vfs_init_once, &esp_vfs_init));
    vfsFd_ = ::open("/dev/wakeup/0", 0, 0);
    HASSERT(vfsFd_ >= 0);
    HASSERT(0 == pthread_setspecific(select_wakeup_key, this));
    LOG(VERBOSE, "VFSALLOC wakeup fd %d (thr %p test %p)", vfsFd_,
        os_thread_self(), pthread_getspecific(select_wakeup_key));
}

/// Releases an FD previously allocated by esp_allocate_vfs_fd.
/// Note: this is currently no-op since there is only one FD allocated for all
/// instances of OSSelectWakeup.
void OSSelectWakeup::esp_deallocate_vfs_fd()
{
    if (vfsFd_ >= 0)
    {
        ::close(vfsFd_);
    }
    vfsFd_ = -1;
}

#endif // ESP32