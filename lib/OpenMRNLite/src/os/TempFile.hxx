/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file TempFile.hxx
 *
 * Helper classes for creating temporary files for testing and mock
 * implementations.
 *
 * @author Balazs Racz
 * @date 22 Mar 2015
 */

#ifndef _OS_TEMPFILE_HXX_
#define _OS_TEMPFILE_HXX_

#include <string>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "utils/logging.h"

/** This class creates a temporary directory for the test, and removes it when
 * the test is done.  The caller is responsible for creating and removing the
 * files in this directory. 
 *
 * The temporary directory will be under the current working directory (usually
 * wherever the test is running). */
class TempDir {
public:
#ifndef __FreeRTOS__
  TempDir();
#endif

  ~TempDir() {
      if (::rmdir(dirName_.c_str()) != 0)
      {
          LOG(WARNING, "Error deleting temporary directory %s: %s",
              dirName_.c_str(), strerror(errno));
    }
  }

    /// @return a global singleton instance of TempDir.
  static TempDir* instance() {
    static TempDir me;
    return &me;
  }

    /// @return name of the temporary directory.
  const string& name() const {
    return dirName_;
  }

private:
    /// name of the temporary directory.
  string dirName_;
};

/** This class creates a temporary file for the test, and removes it when the
 * test is done. */
class TempFile {
public:
    /// Constructor.
    /// @param dir isthe temp directory to create the file within.
    /// @param basename will be the prefix of the name. A unique suffix will be
    /// appended for each file.
  TempFile(const TempDir& dir, const string& basename);

  ~TempFile() {
    ::close(fd_);
    ::unlink(fileName_.c_str());
  }

    /// @return the full path name to this temporary file.
  const string& name() const {
    return fileName_;
  }

    /// @return the file descriptor.
  int fd()
  {
      return fd_;
  }

  /// writes a single byte to the temporary file. @param byte isthe data to
  /// write.
  void write(const uint8_t byte) {
    string s;
    s.push_back(byte);
    write(s);
  }

  /// writes the given data to the temporary file from offset 0. @param s is
  /// the data to write.
  void rewrite(const string& s) {
      ::lseek(fd_, 0, SEEK_SET);
      write(s);
  }
    
  /// writes the given data to the temporary file. @param s is the data to
  /// write.
  void write(const string& s) {
    size_t ofs = 0;
    while (ofs < s.size()) {
      int ret = ::write(fd_, s.data() + ofs, s.size() - ofs);
      HASSERT(ret >= 0);
      ofs += ret;
    }
#ifndef __WINNT__    
    fsync(fd_);
#endif    
  }

private:
    /// The full path name.
  string fileName_;
    /// The file descriptor.
  int fd_;
};

#endif
