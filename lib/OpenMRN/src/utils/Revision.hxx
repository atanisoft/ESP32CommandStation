/** @copyright
 * Copyright (c) 2018, Stuart W. Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * @file Revision.hxx
 *
 * Utility for obtaining build revision info.
 *
 * @author Stuart Baker
 * @date 28 July 2018
 */

#ifndef _UTILS_REVISION_H_
#define _UTILS_REVISION_H_

#include <string>

/// Helper object for obtaining buid revision info.
class Revision
{
public:
    /// Get a revision out of the revision string array.
    /// @param index index within the array to get the string for
    /// @return revision string, else empty string if at or beyond array bounds
    static std::string get(unsigned index)
    {
        if (index < count())
        {
            return REVISION[index];
        }
        return "";
    }

    /// Get the total number of valid revision strings
    /// @return total number of valid revision strings
    static size_t count();

private:
    static const char *REVISION[];
};

#endif // _UTILS_REVISION_H_
