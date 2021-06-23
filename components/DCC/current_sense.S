/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2021 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#include "sdkconfig.h"

#include <soc/rtc_cntl_reg.h>
#include <soc/soc_ulp.h>

    /* Configure the number of ADC samples to average on each measurement.
       Must be a power of 2. */
    .set adc_sample_count_log, 4
    .set adc_sample_count, (1 << adc_sample_count_log)

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY)
    .set ops_adc_channel, CONFIG_OPS_TRACK_CURRENT_SENSE_ADC
#endif

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY)
    .set prog_adc_channel, CONFIG_PROG_TRACK_CURRENT_SENSE_ADC
#endif

    /* Start of variable declaration section, all zero initialized. */
    .bss

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY)
    /* OPS Short threshold. */
    .global ops_short_threshold
ops_short_threshold:
    .long 0

    /* OPS last reading */
    .global ops_last_reading
ops_last_reading:
    .long 0
#endif

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY)
    /* PROG ACK threshold. */
    .global prog_ack_threshold
prog_ack_threshold:
    .long 0

    /* PROG Short threshold. */
    .global prog_short_threshold
prog_short_threshold:
    .long 0

    /* PROG last reading */
    .global prog_last_reading
prog_last_reading:
    .long 0
#endif

    /* Start of ULP code */
    .text
    .global entry
entry:

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY)
    /* r0 will be used as accumulator for OPS */
    move r0, 0
#endif

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY)
    /* r2 will be used as accumulator for PROG */
    move r2, 0
#endif

    /* initialize the loop counter */
    stage_rst

measure:
#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY)
    /* measure and add value to accumulator */
    adc r1, 0, ops_adc_channel + 1
    add r0, r0, r1
#endif

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY)
    /* measure and add value to accumulator */
    adc r3, 0, prog_adc_channel + 1
    add r2, r2, r3
#endif

    /* increment loop counter and check exit condition */
    stage_inc 1
    jumps measure, adc_sample_count, lt

    /* reset stage counter */
    stage_rst

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY)
    /* divide accumulator value using bit shifting */
    rsh     r0, r0, adc_sample_count_log
    /* store value for later usage */
    move    r3, ops_last_reading
	st      r0, r3, 0
#endif

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY)
    /* divide accumulator value using bit shifting */
    rsh     r2, r2, adc_sample_count_log
    /* store value for later usage */
    move    r3, prog_last_reading
	st      r2, r3, 0
#endif

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_ONLY)
    move    r3, ops_last_reading
    ld      r0, r3, 0

    /* Wakeup SoC if ops_last_reading > ops_short_threshold */
    move    r3, ops_short_threshold
    ld      r3, r3, 0
    sub     r3, r3, r0
    jump    wake_up, ov
#endif

#if defined(CONFIG_DCC_TRACK_OUTPUTS_OPS_AND_PROG) || \
    defined(CONFIG_DCC_TRACK_OUTPUTS_PROG_ONLY)
    move    r3, prog_last_reading
    ld      r0, r3, 0

    /* Wakeup SoC if prog_last_reading > prog_ack_threshold.
    NOTE: prog_short_threshold is not checked as it should be higher than the
          prog_ack_threshold value.*/
    move    r3, prog_ack_threshold
    ld      r3, r3, 0
    sub     r3, r3, r0
    jump    wake_up, ov
#endif

    .global exit
exit:
    halt

    .global wake_up
wake_up:
    wake
    halt