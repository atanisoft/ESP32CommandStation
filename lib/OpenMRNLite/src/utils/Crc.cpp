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
 * \file Crc.cxx
 *
 * Helper functions for computing checksums.
 *
 * @author Balazs Racz
 * @date 16 Dec 2014
 */

#include <stdint.h>

#include "utils/Crc.hxx"
#include "utils/macros.h"

/// Initialization value for the CRC-16-IBM calculator.
static const uint16_t crc_16_ibm_init_value = 0x0000; // TODO: check
/// Polynomial for the CRC-16-IBM calculator.
static const uint16_t crc_16_ibm_poly = 0xA001; // TODO: check

/// Reverses the bits of a byte.
///
/// @param data input byte
///
/// @return the input bits reversed (bit 0 exchanged with bit 7 et al.)
///
uint8_t reverse(uint8_t data) {
    uint8_t out = 0;
    for (int i = 0; i < 8; i++) {
        out = (out << 1) | (data & 1);
        data >>= 1;
    }
    return out;
}

/*

static const uint16_t crc_16_ibm_poly = 0x8005; // WORKING

inline void crc_16_ibm_add(uint16_t& state, uint8_t data) {
    //data = reverse(data);
    // This does LSB-first.
    for (int i = 0; i < 8; i++) {
        bool bit = data & 1;
        data >>= 1;
        if (state & 0x8000) {
            bit = !bit;
        }
        if (bit) {
            state = (state << 1) ^ crc_16_ibm_poly;
        } else {
            state = (state << 1);
        }
    }
}

inline uint16_t crc_16_ibm_finish(uint16_t state) {
    return (reverse(state & 0xff) << 8) | reverse(state >> 8);
    //crc_16_ibm_add(state, 0);
    //crc_16_ibm_add(state, 0);
    //return state;
    }*/

/// Appends a byte to a CRC16 state machine.
///
/// @param state the state machine of the CRC computer.
/// @param data next byte to add.
///
inline void crc_16_ibm_add(uint16_t& state, uint8_t data) {
    state ^= data;
    for (int i = 0; i < 8; i++) {
        if (state & 1) {
            state = (state >> 1) ^ crc_16_ibm_poly;
        } else {
            state = (state >> 1);
        }
    }
}

/// Finalizes the state machine of a CRC16-IBM calculator.
///
/// @param state internal state of the machine.
///
/// @return the CRC-16-IBM value of the byte sequence added to the state
/// machine during its lifetime.
///
inline uint16_t crc_16_ibm_finish(uint16_t state) {
    //return (reverse(state & 0xff) << 8) | reverse(state >> 8);
    //crc_16_ibm_add(state, 0);
    //crc_16_ibm_add(state, 0);
    return state;
}


uint16_t crc_16_ibm(const void* data, size_t length) {
    const uint8_t *payload = static_cast<const uint8_t*>(data);
    uint16_t state = crc_16_ibm_init_value;
    for (size_t i = 0; i < length; ++i) {
        crc_16_ibm_add(state, payload[i]);
    }
    return crc_16_ibm_finish(state);
}

void crc3_crc16_ibm(const void* data, size_t length_bytes, uint16_t* checksum) {
  uint16_t state1 = crc_16_ibm_init_value;
  uint16_t state2 = crc_16_ibm_init_value;
  uint16_t state3 = crc_16_ibm_init_value;

#ifdef ESP_NONOS
  // Aligned reads only.
  const uint32_t* payload = static_cast<const uint32_t*>(data);
  HASSERT((((uint32_t)payload) & 3) == 0);
  uint32_t cword = 0;
  for (size_t i = 1; i <= length_bytes; ++i) {
      if ((i & 3) == 1) {
          cword = payload[(i - 1) >> 2];
      } else {
          cword >>= 8;
      }
      uint8_t cbyte = cword & 0xff;
      crc_16_ibm_add(state1, cbyte);
      if (i & 1) {
          // odd byte
          crc_16_ibm_add(state2, cbyte);
      } else {
          // even byte
          crc_16_ibm_add(state3, cbyte);
      }
  }
#else
  const uint8_t *payload = static_cast<const uint8_t*>(data);
  for (size_t i = 1; i <= length_bytes; ++i) {
    crc_16_ibm_add(state1, payload[i-1]);
    if (i & 1) {
      // odd byte
      crc_16_ibm_add(state2, payload[i-1]);
    } else {
      // even byte
      crc_16_ibm_add(state3, payload[i-1]);
    }
  }
#endif

  checksum[0] = crc_16_ibm_finish(state1);
  checksum[1] = crc_16_ibm_finish(state2);
  checksum[2] = crc_16_ibm_finish(state3);
}
