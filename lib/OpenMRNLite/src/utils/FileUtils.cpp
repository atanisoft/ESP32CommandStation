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
 * \file FileUtils.cxx
 *
 * Utilities for dealing with files on host OSes.
 *
 * @author Balazs Racz
 * @date 4 Dec 2015
 */

#include <string>

#include "utils/macros.h"

#ifdef __EMSCRIPTEN__

#include <emscripten.h>
#include <emscripten/val.h>

string read_file_to_string(const string &filename)
{
    using emscripten::val;
    EM_ASM(var fs = require('fs'); Module.fs = fs;);
    val fs = val::module_property("fs");
    string contents = fs.call<val>("readFileSync", string(filename),
                             string("binary")).as<string>();
    return contents;
}

#else

#include <stdio.h>
#include <string.h>
#include <errno.h>

/// Opens a file, reads the entire contents, stores it in a c++ std::string and
/// returns this string. Helper function in some client applications. Exits the
/// current application if there is an error.
///
/// @param filename name of file to open.
///
/// @return the file contents.
///
string read_file_to_string(const string &filename)
{
    FILE *f = fopen(filename.c_str(), "rb");
    if (!f)
    {
        fprintf(stderr, "Could not open file %s: %s\n", filename.c_str(),
            strerror(errno));
        exit(1);
    }
    char buf[1024];
    size_t nr;
    string ret;
    while ((nr = fread(buf, 1, sizeof(buf), f)) > 0)
    {
        ret.append(buf, nr);
    }
    fclose(f);
    return ret;
}

/// Opens (or creates) a file, truncates it and overwrites the contents with
/// what is given in a string. Terminates the application if an error is
/// encountered.
///
/// @param filename name of file to open.
/// @param data what to write into the file.
void write_string_to_file(const string &filename, const string &data)
{
    FILE *f = fopen(filename.c_str(), "wb");
    if (!f)
    {
        fprintf(stderr, "Could not open file %s: %s\n", filename.c_str(),
            strerror(errno));
        exit(1);
    }
    size_t nr;
    size_t offset = 0;
    string ret;
    while ((nr = fwrite(data.data() + offset, 1, data.size() - offset, f)) > 0)
    {
        offset += nr;
        if (offset >= data.size()) break;
    }
    if (nr < 0) {
        fprintf(stderr, "error writing: %s\n", strerror(errno));
    }
    fclose(f);
}

#endif
