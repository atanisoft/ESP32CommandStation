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
 * @file Stm32Can.cxx
 * This file implements a can device driver layer specific to STM32F0xx devices.
 *
 * @author Stuart W. Baker
 * @date 3 May 2015
 */

#if (!defined(ARDUINO)) || defined(ARDUINO_ARCH_STM32)

#include "Stm32Can.hxx"

#include <stdint.h>

#include "stm32f_hal_conf.hxx"

#if defined (STM32F072xB) || defined (STM32F091xC)

#include "stm32f0xx_hal_cortex.h"
#define CAN_IRQN CEC_CAN_IRQn

#define CAN_CLOCK cpu_clock_hz

#elif defined (STM32F103xB)

#include "stm32f1xx_hal_cortex.h"
#define SPLIT_INT
#define CAN_TX_IRQN USB_HP_CAN1_TX_IRQn
#define CAN_IRQN CAN_TX_IRQN
#define CAN_SECOND_IRQN USB_LP_CAN1_RX0_IRQn
#define CAN CAN1
#define CAN_CLOCK (cm3_cpu_clock_hz >> 1)

#elif defined (STM32F303xC) || defined (STM32F303xE)

#include "stm32f3xx_hal_cortex.h"
#define SPLIT_INT
#define CAN_TX_IRQN USB_HP_CAN_TX_IRQn
#define CAN_IRQN CAN_TX_IRQN
#define CAN_SECOND_IRQN USB_LP_CAN_RX0_IRQn
#define CAN_CLOCK (cm3_cpu_clock_hz >> 1)

#elif defined (STM32F767xx)

#include "stm32f7xx_hal_cortex.h"
#define SPLIT_INT
#define CAN CAN1
#define CAN_TX_IRQN CAN1_TX_IRQn
#define CAN_IRQN CAN_TX_IRQN
#define CAN_SECOND_IRQN CAN1_RX0_IRQn
#define CAN_CLOCK (cm3_cpu_clock_hz >> 2) // 54 MHz, sysclk/4

#else
#error Dont know what STM32 chip you have.
#endif

Stm32Can *Stm32Can::instances[1] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 */
Stm32Can::Stm32Can(const char *name)
    : Can(name)
{
    /* only one instance allowed */
    HASSERT(instances[0] == NULL);

    instances[0] = this;

    /* should already be disabled, but just in case */
    HAL_NVIC_DisableIRQ(CAN_IRQN);

#if defined (STM32F030x6) || defined (STM32F031x6) || defined (STM32F038xx) \
 || defined (STM32F030x8) || defined (STM32F030xC) || defined (STM32F042x6) \
 || defined (STM32F048xx) || defined (STM32F051x8) || defined (STM32F058xx) \
 || defined (STM32F070x6) || defined (STM32F070xB) || defined (STM32F071xB) \
 || defined (STM32F072xB) || defined (STM32F078xx) \
 || defined (STM32F091xC) || defined (STM32F098xx)
#else
    /* The priority of CAN interrupt is as high as possible while maintaining
     * FreeRTOS compatibility.
     */
    SetInterruptPriority(CAN_IRQN, configKERNEL_INTERRUPT_PRIORITY);

#ifdef SPLIT_INT
    HAL_NVIC_DisableIRQ(CAN_SECOND_IRQN);
    SetInterruptPriority(CAN_SECOND_IRQN, configKERNEL_INTERRUPT_PRIORITY);
#endif
#endif
}

/** Enable use of the device.
 */
void Stm32Can::enable()
{
    /* disable sleep, enter init mode */
    CAN->MCR = CAN_MCR_INRQ;

    /* Time triggered tranmission off
     * Bus off state is left automatically
     * Auto-Wakeup mode disabled
     * automatic re-transmission enabled
     * receive FIFO not locked on overrun
     * TX FIFO mode on
     */
    CAN->MCR |= (CAN_MCR_ABOM | CAN_MCR_TXFP);

    /* Setup timing.
     * 125,000 Kbps = 8 usec/bit
     */
    CAN->BTR = (CAN_BS1_5TQ | CAN_BS2_2TQ | CAN_SJW_1TQ |
                ((CAN_CLOCK / 1000000) - 1));

    /* enter normal mode */
    CAN->MCR &= ~CAN_MCR_INRQ;

    /* Enter filter initialization mode.  Filter 0 will be used as a single
     * 32-bit filter, ID Mask Mode, we accept everything, no mask.
     */
    CAN->FMR |= CAN_FMR_FINIT;
    CAN->FM1R = 0;
    CAN->FS1R = 0x000000001;
    CAN->FFA1R = 0;
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;

    /* Activeate filter and exit initialization mode. */
    CAN->FA1R = 0x000000001;
    CAN->FMR &= ~CAN_FMR_FINIT;

    /* enable interrupts */
    CAN->IER = (/*CAN_IER_ERRIE |*/ CAN_IER_BOFIE | CAN_IER_FMPIE0);
    HAL_NVIC_EnableIRQ(CAN_IRQN);
#ifdef SPLIT_INT
    HAL_NVIC_EnableIRQ(CAN_SECOND_IRQN);
#endif
}

/** Disable use of the device.
 */
void Stm32Can::disable()
{
    HAL_NVIC_DisableIRQ(CAN_IRQN);
#ifdef SPLIT_INT
    HAL_NVIC_DisableIRQ(CAN_SECOND_IRQN);
#endif
    CAN->IER = 0;

    /* disable sleep, enter init mode */
    CAN->MCR = CAN_MCR_INRQ;
}

/* Try and transmit a message.
 */
void Stm32Can::tx_msg()
{
    /* see if we can send anything out */
    struct can_frame *can_frame;

    size_t msg_count = txBuf->data_read_pointer(&can_frame);
    unsigned i;

    for (i = 0; i < msg_count; ++i, ++can_frame)
    {
        volatile CAN_TxMailBox_TypeDef *mailbox;
        if (CAN->TSR & CAN_TSR_TME0)
        {
            mailbox = CAN->sTxMailBox + 0;
        }
        else if (CAN->TSR & CAN_TSR_TME1)
        {
            mailbox = CAN->sTxMailBox + 1;
        }
        else if (CAN->TSR & CAN_TSR_TME2)
        {
            mailbox = CAN->sTxMailBox + 2;
        }
        else
        {
            /* no empty mailboxes left to fill */
            break;
        }

        /* setup frame */
        if (can_frame->can_eff)
        {
            mailbox->TIR = (can_frame->can_id << 3) | CAN_TI0R_IDE;
        }
        else
        {
            mailbox->TIR = can_frame->can_id << 21;
        }
        if (can_frame->can_rtr)
        {
            mailbox->TIR |= CAN_TI0R_RTR;
        }
        else
        {
            mailbox->TDTR = can_frame->can_dlc;
            mailbox->TDLR = (can_frame->data[0] <<  0) |
                            (can_frame->data[1] <<  8) |
                            (can_frame->data[2] << 16) |
                            (can_frame->data[3] << 24);
            mailbox->TDHR = (can_frame->data[4] <<  0) |
                            (can_frame->data[5] <<  8) |
                            (can_frame->data[6] << 16) |
                            (can_frame->data[7] << 24);
        }

        /* request transmission */
        mailbox->TIR |= CAN_TI0R_TXRQ;
    }

    if (i)
    {
        txBuf->consume(i);
        txBuf->signal_condition();
    }

    /* enable transmit interrupt */
    CAN->IER |= CAN_IER_TMEIE;
}

/** Handle an interrupt.
 */
void Stm32Can::rx_interrupt_handler()
{
    unsigned msg_receive_count = 0;

    if (CAN->ESR & CAN_ESR_BOFF)
    {
        /* bus off error condition */
        CAN->TSR |= CAN_TSR_ABRQ2;
        CAN->TSR |= CAN_TSR_ABRQ1;
        CAN->TSR |= CAN_TSR_ABRQ0;
        CAN->IER &= ~CAN_IER_TMEIE;
        txBuf->flush();
        txBuf->signal_condition_from_isr();
    }
#if 0
    if (CAN->MSR & CAN_MSR_ERRI)
    {
        /* error condition */
        CAN->TSR |= CAN_TSR_ABRQ2;
        CAN->TSR |= CAN_TSR_ABRQ1;
        CAN->TSR |= CAN_TSR_ABRQ0;
        CAN->IER &= ~CAN_IER_TMEIE;
        CAN->MSR &= ~CAN_MSR_ERRI;
        txBuf->flush();
        txBuf->signal_condition_from_isr();
        ++softErrorCount;
    }
#endif
    while (CAN->RF0R & CAN_RF0R_FMP0)
    {
        /* rx data received */
        struct can_frame *can_frame;
        size_t msg_count = rxBuf->data_write_pointer(&can_frame);
        if (msg_count)
        {
            if (CAN->sFIFOMailBox[0].RIR & CAN_RI0R_IDE)
            {
                /* extended frame */
                can_frame->can_id = CAN->sFIFOMailBox[0].RIR >> 3;
                can_frame->can_eff = 1;
            }
            else
            {
                /* standard frame */
                can_frame->can_id = CAN->sFIFOMailBox[0].RIR >> 21;
                can_frame->can_eff = 0;
            }
            if (CAN->sFIFOMailBox[0].RIR & CAN_RI0R_RTR)
            {
                /* remote frame */
                can_frame->can_rtr = 1;
                can_frame->can_dlc = 0;
            }
            else
            {
                /* data frame */
                can_frame->can_rtr = 0;
                can_frame->can_dlc = CAN->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC;
                can_frame->data[0] = (CAN->sFIFOMailBox[0].RDLR >>  0) & 0xFF;
                can_frame->data[1] = (CAN->sFIFOMailBox[0].RDLR >>  8) & 0xFF;
                can_frame->data[2] = (CAN->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
                can_frame->data[3] = (CAN->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
                can_frame->data[4] = (CAN->sFIFOMailBox[0].RDHR >>  0) & 0xFF;
                can_frame->data[5] = (CAN->sFIFOMailBox[0].RDHR >>  8) & 0xFF;
                can_frame->data[6] = (CAN->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
                can_frame->data[7] = (CAN->sFIFOMailBox[0].RDHR >> 24) & 0xFF;
            }
        }
        else
        {
            ++overrunCount;
        }
        /* release FIFO */
        CAN->RF0R |= CAN_RF0R_RFOM0;
        ++msg_receive_count;
    }

    if (msg_receive_count)
    {
        /* advance the "zero copy" buffer by the number of messages received */
        rxBuf->advance(msg_receive_count);
        rxBuf->signal_condition_from_isr();
    }
}

void Stm32Can::tx_interrupt_handler()
{
    if (CAN->TSR & (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2))
    {
        /* transmit request completed, should be able to send another */
        struct can_frame *can_frame;

        size_t msg_count = txBuf->data_read_pointer(&can_frame);
        if (msg_count)
        {
            /* try and send some more CAN frames */
            unsigned i;

            for (i = 0; i < msg_count; ++i, ++can_frame)
            {
                volatile CAN_TxMailBox_TypeDef *mailbox;
                if (CAN->TSR & CAN_TSR_TME0)
                {
                    mailbox = CAN->sTxMailBox + 0;
                }
                else if (CAN->TSR & CAN_TSR_TME1)
                {
                    mailbox = CAN->sTxMailBox + 1;
                }
                else if (CAN->TSR & CAN_TSR_TME2)
                {
                    mailbox = CAN->sTxMailBox + 2;
                }
                else
                {
                    /* no empty mailboxes left to fill */
                    break;
                }

                /* setup frame */
                if (can_frame->can_eff)
                {
                    mailbox->TIR = (can_frame->can_id << 3) | CAN_TI0R_IDE;
                }
                else
                {
                    mailbox->TIR = can_frame->can_id << 21;
                }
                if (can_frame->can_rtr)
                {
                    mailbox->TIR |= CAN_TI0R_RTR;
                }
                else
                {
                    mailbox->TDTR = can_frame->can_dlc;
                    mailbox->TDLR = (can_frame->data[0] <<  0) |
                                    (can_frame->data[1] <<  8) |
                                    (can_frame->data[2] << 16) |
                                    (can_frame->data[3] << 24);
                    mailbox->TDHR = (can_frame->data[4] <<  0) |
                                    (can_frame->data[5] <<  8) |
                                    (can_frame->data[6] << 16) |
                                    (can_frame->data[7] << 24);
                }

                /* request transmission */
                mailbox->TIR |= CAN_TI0R_TXRQ;
            }
            txBuf->consume(i);
        }
        else
        {
            /* no more data left to transmit */
            CAN->IER &= ~CAN_IER_TMEIE;
        }
        txBuf->signal_condition_from_isr();
    }
}


extern "C" {
/** This is the interrupt handler for the can device.
 */

#if defined (STM32F072xB) || defined (STM32F091xC)
void cec_can_interrupt_handler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
    Stm32Can::instances[0]->tx_interrupt_handler();
}
#elif defined (STM32F103xB) || defined (STM32F303xC) || defined (STM32F303xE)

void usb_hp_can1_tx_interrupt_handler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}

void usb_lp_can1_rx0_interrupt_handler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}

#elif defined(STM32F767xx)

void can1_tx_interrupt_handler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}

void can1_rx0_interrupt_handler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}

#else
#error Dont know what STM32 chip you have.
#endif

} // extern "C"

#endif // !ARDUINO || STM32

#if defined(ARDUINO_ARCH_STM32)

#include "stm32_def.h"
#include "PinAF_STM32F1.h"
#include <PeripheralPins.h>

void arduino_can_pinmap(PinName tx_pin, PinName rx_pin) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    void* can_tx = pinmap_peripheral(tx_pin, PinMap_CAN_TD);
    void* can_rx = pinmap_peripheral(rx_pin, PinMap_CAN_RD);
    void* can = pinmap_merge_peripheral(can_tx, can_rx);
    if (can == NP) {
        DIE("Could not find CAN peripheral");
    }
        
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Pin       = STM_GPIO_PIN(tx_pin);
    auto fn = pinmap_function(tx_pin, PinMap_CAN_TD);
    GPIO_InitStruct.Mode      = STM_PIN_MODE(fn);
    GPIO_InitStruct.Pull      = STM_PIN_PUPD(fn);
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
#ifdef STM32F1xx
    pin_SetF1AFPin(STM_PIN_AFNUM(fn));
#else
    GPIO_InitStruct.Alternate = STM_PIN_AFNUM(fn);
#endif /* STM32F1xx */
    HAL_GPIO_Init(set_GPIO_Port_Clock(STM_PORT(tx_pin)), &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = STM_GPIO_PIN(rx_pin);
    fn = pinmap_function(rx_pin, PinMap_CAN_RD);
    GPIO_InitStruct.Mode      = STM_PIN_MODE(fn);
    GPIO_InitStruct.Pull      = STM_PIN_PUPD(fn);
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
#ifdef STM32F1xx
    pin_SetF1AFPin(STM_PIN_AFNUM(fn));
#else
    GPIO_InitStruct.Alternate = STM_PIN_AFNUM(fn);
#endif /* STM32F1xx */
    HAL_GPIO_Init(set_GPIO_Port_Clock(STM_PORT(rx_pin)), &GPIO_InitStruct);
}

extern "C" {
void USB_HP_CAN_TX_IRQHandler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}

void USB_LP_CAN_RX0_IRQHandler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}
void CEC_CAN_IRQHandler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
    Stm32Can::instances[0]->tx_interrupt_handler();
}
void CAN1_TX_IRQHandler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}
void CAN1_RX0_IRQHandler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}
} // extern "C"

#endif

