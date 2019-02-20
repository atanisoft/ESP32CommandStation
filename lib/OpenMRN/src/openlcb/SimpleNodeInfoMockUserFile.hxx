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
 * \file SimpleNodeInfoMockUserFile.hxx
 *
 * Mock file implementation for the SNIP user-modifiable data. Use this in
 * tests and when there is no storage available.
 *
 * @author Balazs Racz
 * @date 22 Mar 2015
 */


#ifndef _OPENLCB_SIMPLENODEINFOMOCKUSERFILE_HXX_
#define _OPENLCB_SIMPLENODEINFOMOCKUSERFILE_HXX_

#include "SimpleNodeInfo.hxx"
#ifdef __FreeRTOS__
#include "freertos_drivers/common/RamDisk.hxx"
#endif

#ifndef __WINNT__
#include "os/TempFile.hxx"

namespace openlcb {

/** Helper class for mock implementations. Creates a mock file with the SNIP
 * user-modifiable data inside that can be used as SNIP_DYNAMIC_FILENAME.
 *
 * Usage:
 * 
 * static MockSNIPUserFile g_snip_file(g_dir, "Default user name", "Default user description");
 * const char *const SNIP_DYNAMIC_FILENAME = MockSNIPUserFile::snip_user_file_path;
 */
class MockSNIPUserFile
{
public:
#ifdef __FreeRTOS__
    static constexpr const char* snip_user_file_path = "/etc/snip_user_data";
#else
    static char snip_user_file_path[128];
#endif
    MockSNIPUserFile(const char *user_name,
                     const char *user_description);

    ~MockSNIPUserFile();

private:
#ifdef __FreeRTOS__
    SimpleNodeDynamicValues snipData_;
    RamDisk userFile_;
#else
    TempFile userFile_;
#endif
};

}  // namespace openlcb

#endif // !winnt
#endif // _OPENLCB_SIMPLENODEINFOMOCKUSERFILE_HXX_
