/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * @file Stm32Can.hxx
 * This file implements a can device driver layer specific to STM32F0xx devices.
 *
 * @author Stuart W. Baker
 * @date 3 May 2015
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32F0XXCAN_HXX_
#define _FREERTOS_DRIVERS_ST_STM32F0XXCAN_HXX_

#include <cstdint>

#ifdef ARDUINO
#include <Arduino.h>
#include "freertos_drivers/arduino/Can.hxx"
#else
#include "freertos_drivers/common/Can.hxx"
#endif

#if defined(STM32F072xB) || defined(STM32F091xC) 
#include "stm32f0xx_hal_can.h"
#elif defined(STM32F103xB)
#include "stm32f1xx_hal_can.h"
#elif defined(STM32F303xC) || defined(STM32F303xE)
#include "stm32f3xx_hal_can.h"
#elif defined(STM32F767xx)
#include "stm32f7xx_hal_can.h"
#else
#error Dont know what STM32 chip you have.
#endif

/** Specialization of CAN driver for LPC17xx and LPC40xx CAN.
 */
class Stm32Can : public Can
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     */
    Stm32Can(const char *name);

    /** Destructor.
     */
    ~Stm32Can()
    {
    }

    /** Handle an interrupt. */
    void rx_interrupt_handler();
    /** Handle an interrupt. */
    void tx_interrupt_handler();

    /** Instance pointers help us get context from the interrupt handler(s) */
    static Stm32Can *instances[1];

private:
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */
    void tx_msg() override; /**< function to try and transmit a message */

    /** one interrupt vector is shared between two CAN controllers, so we need
     *  to keep track of the number of controllers in use.
     */
    static unsigned int intCount;

    /** Default constructor.
     */
    Stm32Can();

    DISALLOW_COPY_AND_ASSIGN(Stm32Can);
};

#ifdef ARDUINO

extern void arduino_can_pinmap(PinName tx_pin, PinName rx_pin);

#endif

#endif /* _FREERTOS_DRIVERS_ST_STM32F0XXCAN_HXX_ */
