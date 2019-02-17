/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file can_frame.h
 * This file defines an abstration of CAN for various environments.
 *
 * @author Stuart W. Baker
 * @date 27 September 2012
 */

#ifndef _nmranet_can_h_
#define _nmranet_can_h_

#if defined (__linux__)
    #include <sys/socket.h>
    #include <linux/can.h>
    #include <linux/can/raw.h>
    #include <linux/can/error.h>

    #define SET_CAN_FRAME_EFF(_frame) (_frame).can_id |= CAN_EFF_FLAG
    #define SET_CAN_FRAME_RTR(_frame) (_frame).can_id |= CAN_RTR_FLAG
    #define SET_CAN_FRAME_ERR(_frame) (_frame).can_id |= CAN_ERR_FLAG
    #define CLR_CAN_FRAME_EFF(_frame) (_frame).can_id &= ~CAN_EFF_FLAG
    #define CLR_CAN_FRAME_RTR(_frame) (_frame).can_id &= ~CAN_RTR_FLAG
    #define CLR_CAN_FRAME_ERR(_frame) (_frame).can_id &= ~CAN_ERR_FLAG
    #define IS_CAN_FRAME_EFF(_frame) ((_frame).can_id & CAN_EFF_FLAG)
    #define IS_CAN_FRAME_RTR(_frame) ((_frame).can_id & CAN_RTR_FLAG)
    #define IS_CAN_FRAME_ERR(_frame) ((_frame).can_id & CAN_ERR_FLAG)

    #define GET_CAN_FRAME_ID_EFF(_frame) ((_frame).can_id & CAN_EFF_MASK)
    #define GET_CAN_FRAME_ID(_frame)     ((_frame).can_id & CAN_SFF_MASK)
    #define SET_CAN_FRAME_ID_EFF(_frame, _value) \
    {                                            \
        (_frame).can_id &= ~CAN_EFF_MASK;        \
        (_frame).can_id += ((_value) & CAN_EFF_MASK);   \
    }
    #define SET_CAN_FRAME_ID(_frame, _value) \
    {                                        \
        (_frame).can_id &= ~CAN_SFF_MASK;    \
        (_frame).can_id += ((_value) & CAN_SFF_MASK);   \
    }

#elif defined (__nuttx__) || defined (__FreeRTOS__) || defined (__MACH__) || defined (__WIN32__) || defined(__EMSCRIPTEN__) || defined(ESP_NONOS) || defined(ARDUINO)
    #include <stdint.h>

    struct can_frame
    {
        uint32_t can_id;      /**< 11- or 29-bit ID (3-bits unsed) */

        uint8_t  can_dlc : 4; /**< 4-bit DLC */
        uint8_t  can_rtr : 1; /**< RTR indication */
        uint8_t  can_eff : 1; /**< Extended ID indication */
        uint8_t  can_err : 1; /**< @todo not supported by nuttx */
        uint8_t  can_res : 1; /**< Unused */

        uint8_t  pad;  /**< padding */
        uint8_t  res0; /**< reserved */
        uint8_t  res1; /**< reserved */

        union
        {
            /** CAN message data (64-bit) */
            uint64_t data64 __attribute__((aligned(8)));
            /** CAN message data (0-8 byte) */
            uint8_t  data[8] __attribute__((aligned(8)));
        };
    };

    #define SET_CAN_FRAME_EFF(_frame) (_frame).can_eff = 1
    #define SET_CAN_FRAME_RTR(_frame) (_frame).can_rtr = 1
    #define SET_CAN_FRAME_ERR(_frame) (_frame).can_err = 1
    #define CLR_CAN_FRAME_EFF(_frame) (_frame).can_eff = 0
    #define CLR_CAN_FRAME_RTR(_frame) (_frame).can_rtr = 0
    #define CLR_CAN_FRAME_ERR(_frame) (_frame).can_err = 0
    #define IS_CAN_FRAME_EFF(_frame) ((_frame).can_eff)
    #define IS_CAN_FRAME_RTR(_frame) ((_frame).can_rtr)
    #define IS_CAN_FRAME_ERR(_frame) ((0))

    #define GET_CAN_FRAME_ID_EFF(_frame)  (_frame).can_id
    #define GET_CAN_FRAME_ID(_frame)      (_frame).can_id
    #define SET_CAN_FRAME_ID_EFF(_frame, _value)  (_frame).can_id = ((_value) & 0x1FFFFFFFU)
    #define SET_CAN_FRAME_ID(_frame, _value)     (_frame).can_id = ((_value) & 0x7ffU)

#else
#error No CAN frame representation for your OS
#endif

#endif /* _nmranet_can_h_ */
