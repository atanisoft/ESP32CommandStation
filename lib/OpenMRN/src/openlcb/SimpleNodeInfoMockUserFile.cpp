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

#ifndef  _POSIX_C_SOURCE
#define  _POSIX_C_SOURCE  200112L
#endif

#include "SimpleNodeInfoMockUserFile.hxx"

#ifdef __FreeRTOS__
openlcb::MockSNIPUserFile::MockSNIPUserFile(const char *user_name,
                                            const char *user_description)
    : snipData_{2}
    , userFile_(MockSNIPUserFile::snip_user_file_path, &snipData_, false)
{
    strncpy(snipData_.user_name, user_name, sizeof(snipData_.user_name));
    strncpy(snipData_.user_description, user_description,
            sizeof(snipData_.user_description));
}

openlcb::MockSNIPUserFile::~MockSNIPUserFile()
{
}

#elif !defined(__WINNT__)
#include "os/TempFile.hxx"

openlcb::MockSNIPUserFile::MockSNIPUserFile(const char *user_name,
                                            const char *user_description)
  : userFile_(*TempDir::instance(), "snip_user_file")
{
    init_snip_user_file(userFile_.fd(), user_name, user_description);
    HASSERT(userFile_.name().size() < sizeof(snip_user_file_path));
    strncpy(snip_user_file_path, userFile_.name().c_str(),
            sizeof(snip_user_file_path));
}

char openlcb::MockSNIPUserFile::snip_user_file_path[128] = "/dev/zero";

openlcb::MockSNIPUserFile::~MockSNIPUserFile()
{
}

#endif

