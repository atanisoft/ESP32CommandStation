/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file Crc.hxx
 *
 * Helper functions for computing checksums.
 *
 * @author Balazs Racz
 * @date 16 Dec 2014
 */

#include <stdint.h>
#include <stddef.h>

/** Computes the 16-bit CRC value over data using the CRC16-ANSI (aka
 * CRC16-IBM) settings. This involves zero init value, zero terminating value,
 * reversed polynomial 0xA001, reversing input bits and reversing output
 * bits. The example CRC value of "123456789" is 0xbb3d.
 * @param data what to compute the checksum over
 * @param length_bytes how long data is
 * @return the CRC-16-IBM value of the checksummed data.
 */
uint16_t crc_16_ibm(const void* data, size_t length_bytes);

/** Computes the triple-CRC value over a chunk of data. checksum is an array of
 * 3 halfwords. The first halfword will get the CRC of the data array, the
 * second halfword the CRC of all odd bytes (starting with the first byte), the
 * third halfword will get the CRC of all even bytes. 
 *
 * This routine is helpful, because a similar routine is part of the TIVA
 * microcontroller ROM, thus needs no implementation on an actual part. It is
 * important to note that the TIVA version takes the length in 4-byte words and
 * allows only using data length divisible by 4.
 * @param data what to compute the checksum over
 * @param length_bytes how long data is
 * @param checksum is the output buffer where to store the 48-bit checksum.
 */
void crc3_crc16_ibm(const void* data, size_t length_bytes, uint16_t* checksum);
