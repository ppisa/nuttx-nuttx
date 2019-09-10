/****************************************************************************
 * boards/arm/stm32l4/anca-l476re/src/stm32_buttons.c
 *
 *   Copyright (C) 2014-2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Pavel Pisa <ppisa@pikron.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include <nuttx/timers/rtc.h>
#include "stm32l4_rtc.h"
#include "stm32l4_rcc.h"

#include "anca-l476re.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xxx
 *
 * Description:
 *   XXX.
 *
 ****************************************************************************/

void static anca_l476re_rtc_unlock(void)
{
  irqstate_t irqs;
  uint32_t regval;

  irqs = enter_critical_section();

  regval  = getreg32(STM32L4_RCC_APB1ENR1);
  regval |= RCC_APB1ENR1_PWREN;
  putreg32(regval, STM32L4_RCC_APB1ENR1);

  (void)stm32l4_pwr_enablebkp(true);
  putreg32(0xca, STM32L4_RTC_WPR);
  putreg32(0x53, STM32L4_RTC_WPR);

  regval = getreg32(STM32L4_RTC_TAMPCR);
  regval |= RTC_TAMPCR_TAMP3NOERASE | RTC_TAMPCR_TAMP2NOERASE |
            RTC_TAMPCR_TAMP1NOERASE;
  putreg32(regval, STM32L4_RTC_TAMPCR);

  leave_critical_section(irqs);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int anca_l476re_set_reset_target(int target)
{
  irqstate_t irqs;
  uint32_t regval;
  int ret;

  irqs = enter_critical_section();

  anca_l476re_rtc_unlock();
  regval = getreg32(STM32L4_RTC_BKR(0));
  if (target == BIOC_SET_BOOT_TO_LOADER) {
    regval &= ~0xff;
    regval |= 0x55;
  } else if (target == BIOC_SET_BOOT_TO_APP) {
    regval &= ~0xff;
    regval |= 0xAA;
  } else if (target == BIOC_SET_BOOT_TO_APP_NO_TIMEOUT) {
    regval &= ~0xff;
    regval |= 0xA5;
  }
  putreg32(regval, STM32L4_RTC_BKR(0));

  regval &= 0xff;
  ret = 0;

  switch (regval) {
    case 0x55:
      ret = BIOC_SET_BOOT_TO_LOADER;
      break;
    case 0xAA:
      ret = BIOC_SET_BOOT_TO_APP;
      break;
    case 0xA5:
      ret = BIOC_SET_BOOT_TO_APP_NO_TIMEOUT;
      break;
  }

  leave_critical_section(irqs);

  return ret;
}

void anca_l476re_set_reset_reason(uint32_t reason)
{
  irqstate_t irqs;
  uint32_t regval;

  anca_l476re_rtc_unlock();

  irqs = enter_critical_section();

  anca_l476re_rtc_unlock();
  regval = getreg32(STM32L4_RTC_BKR(0));
  regval &= 0xff;
  regval |= reason << 8;
  putreg32(regval, STM32L4_RTC_BKR(0));

  leave_critical_section(irqs);
}

/****************************************************************************
 * Name: xxx
 *
 * Description:
 *   XXX.
 *
 ****************************************************************************/
