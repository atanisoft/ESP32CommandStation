/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file RailcomDriver.hxx
 *
 * Abstract interface for communicating railcom-related information between
 * multiple device drivers.
 *
 * @author Balazs Racz
 * @date 6 Jan 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_RAILCOMDRIVER_HXX_
#define _FREERTOS_DRIVERS_COMMON_RAILCOMDRIVER_HXX_

/// Abstract base class for railcom drivers. This interface is used to
/// communicate when the railcom cutout happens. The railcom cutout is produced
/// or detected in the DCC generator or DCC parser driver, but the railcom
/// drivers need to adjust the UART configuration in accordance with it.
class RailcomDriver {
public:
  /** Call to the driver for sampling the current sensors. This call is
   * performed repeatedly, in a configurable interval, on the next positive
   * edge.
   */
  virtual void feedback_sample() = 0;
  /** Instructs the driver that the railcom cutout is starting now. The driver
   *  will use this information to enable the UART receiver. */
  virtual void start_cutout() = 0;
  /** Notifies the driver that the railcom cutout has reached the middle point,
   *  i.e., the first window is passed and the second window is starting. The
   *  driver will use this information to separate channel 1 nd channel 2
   *  data. */
  virtual void middle_cutout() = 0;
  /** Instructs the driver that the railcom cutout is over now. The driver
   *  will use this information to disable the UART receiver. */
  virtual void end_cutout() = 0;
  /** Called instead of start/mid/end-cutout at the end of the current packet
   * if there was no cutout requested. */
  virtual void no_cutout() = 0;
  /** Specifies the feedback key to write into the received railcom data
   *  packets. This feedback key is used by the application layer to correlate
   *  the stream of DCC packets to the stream of Railcom packets. This method
   *  shall be called before start_cutout. The feedback key set here is used
   *  until this method is called again. @param key is the new feedback key. */
  virtual void set_feedback_key(uint32_t key) = 0;
};


/** Empty implementation of the railcom driver for boards that have no railcom
 *  hardware. */
class NoRailcomDriver : public RailcomDriver {
  void feedback_sample() OVERRIDE {}
  void start_cutout() OVERRIDE {}
  void middle_cutout() OVERRIDE {}
  void end_cutout() OVERRIDE {}
  void no_cutout() OVERRIDE {}
  void set_feedback_key(uint32_t key) OVERRIDE {}
};

#endif // _FREERTOS_DRIVERS_COMMON_RAILCOMDRIVER_HXX_
