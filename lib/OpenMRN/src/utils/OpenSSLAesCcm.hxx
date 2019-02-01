/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file OpenSSLAesCcm.hxx
 *
 * Helper function to perform encryption/authentication/decryption using
 * AES-CCM algorithm with OpenSSL crypt library.
 *
 * In order to use this function, you need to install libssl-dev package and
 * add to the makefile of your application target to link against it:
 *
 * SYSLIBRARIESEXTRA+=-lcrypto
 *
 * @author Balazs Racz
 * @date 4 Mar 2017
 */

#ifndef _UTILS_OPENSSLAESCCM_HXX_
#define _UTILS_OPENSSLAESCCM_HXX_

#include <string>
#include <openssl/evp.h>

#include "utils/logging.h"

#ifndef ASSERT_EQ
#define ASSERT_EQ(expected, actual)                                            \
    do                                                                         \
    {                                                                          \
        int __ret = (actual);                                                  \
        int __e = (expected);                                                  \
        if (__e != __ret)                                                      \
        {                                                                      \
            LOG(FATAL, "At %s(%d): Failed assertion " #actual                  \
                       ": expected %d actual %d\n",                            \
                __FILE__, __LINE__, __e, __ret);                               \
            HASSERT(0);                                                        \
        }                                                                      \
    } while (false)
#endif

/// Performs authenticated encryption using AES-CCM.
/// @param aes_key is the 256-bit key.
/// @param iv is the unique initialization vector, must be 11 bytes long.
/// @param auth_data is plaintext additional authenticated data.
/// @param plain will be encrypted.
/// @param cipher is the output of the encryption.
/// @param tag is the output of the signature. Will use 16 bytes.
void CCMEncrypt(const std::string &aes_key, const std::string &iv,
    const std::string &auth_data, const std::string &plain, std::string *cipher,
    std::string *tag)
{
    ASSERT_EQ(32u, aes_key.size());
    ASSERT_EQ(11u, iv.size());

    EVP_CIPHER_CTX *ctx;
    ctx = EVP_CIPHER_CTX_new();

    ASSERT_EQ(1,
        EVP_EncryptInit_ex(ctx, EVP_aes_256_ccm(), nullptr, nullptr, nullptr));

    constexpr int taglen = 16;
    constexpr int Lvalue = 4;
    constexpr int ivlen = 15 - Lvalue;
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_SET_TAG, taglen, nullptr);
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_SET_IVLEN, ivlen, NULL);
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_SET_L, Lvalue, NULL);
    // ASSERT_EQ(ivlen, EVP_CIPHER_CTX_iv_length(ctx));
    ASSERT_EQ(32, EVP_CIPHER_CTX_key_length(ctx));
    ASSERT_EQ(
        1, EVP_EncryptInit_ex(ctx, nullptr, nullptr,
               (const uint8_t *)aes_key.data(), (const uint8_t *)iv.data()));

    int outlen = -1;
    ASSERT_EQ(
        1, EVP_EncryptUpdate(ctx, nullptr, &outlen, nullptr, plain.size()));

    ASSERT_EQ(1, EVP_EncryptUpdate(ctx, nullptr, &outlen,
                     (const uint8_t *)auth_data.data(), auth_data.size()));
    cipher->resize(plain.size() + 16);
    ASSERT_EQ(1, EVP_EncryptUpdate(ctx, (uint8_t *)&((*cipher)[0]), &outlen,
                     (const uint8_t *)plain.data(), plain.size()));
    HASSERT(outlen <= (int)cipher->size());
    LOG(INFO, "encupdate: in=%d, ret=%d", (int)plain.size(), outlen);
    cipher->resize(outlen);
    // CCM always outputs the same number of bytes than the input.
    ASSERT_EQ((int)plain.size(), outlen);
    int ret = -1;
    ASSERT_EQ(1, EVP_EncryptFinal_ex(ctx, (uint8_t *)&((*cipher)[0]), &ret));
    ASSERT_EQ(0, ret);

    tag->resize(16);
    ASSERT_EQ(
        1, EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_GET_TAG, 16, &((*tag)[0])));

    EVP_CIPHER_CTX_free(ctx);
}

#endif // _UTILS_OPENSSLAESCCM_HXX_
