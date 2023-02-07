/**
  **************************************************************************
  * @file     at32f421_clock.c
  * @brief    system clock config program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "at32f421_clock.h"

/**
  * @brief  system clock config program
  * @note   the system clock is configured as follow:
  *         - system clock        = (hick 48)
  *         - system clock source = hick
  *         - sclk                = 48000000
  *         - ahbdiv              = 8
  *         - ahbclk              = 6000000
  *         - apb1div             = 1
  *         - apb1clk             = 6000000
  *         - apb2div             = 1
  *         - apb2clk             = 6000000
  *         - flash_wtcyc         = 1 cycle
  * @param  none
  * @retval none
  */
void system_clock_config(void)
{
  /* config flash psr register */
  flash_psr_set(FLASH_WAIT_CYCLE_1);

 
  /* reset crm */
  crm_reset();

  /* enable hick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

   /* wait till hick is ready */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_8);

  /* config apb2clk */
  crm_apb2_div_set(CRM_APB2_DIV_1);

  /* config apb1clk */
  crm_apb1_div_set(CRM_APB1_DIV_1);

  /* config sclk from hick48 */
  crm_hick_sclk_frequency_select(CRM_HICK_SCLK_48MHZ);

  /* select hick as system clock source */
  crm_sysclk_switch(CRM_SCLK_HICK);

  /* wait till hick is used as system clock source */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_HICK)
  {
  }

  /* update system_core_clock global variable */
  system_core_clock_update();
}
