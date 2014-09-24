/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    MK22D5/pal_lld.c
 * @brief   PAL subsystem low level driver.
 *
 * @addtogroup PAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief Reads a logical state from an I/O pad.
 * @note The @ref PAL provides a default software implementation of this
 * functionality, implement this function if can optimize it by using
 * special hardware functionalities or special coding.
 *
 * @param[in] port port identifier
 * @param[in] pad pad number within the port
 * @return The logical state.
 * @retval PAL_LOW low logical state.
 * @retval PAL_HIGH high logical state.
 *
 * @notapi
 */
uint8_t _pal_lld_readpad(ioportid_t port,uint8_t pad)
{
  return (port->PDIR & ((uint32_t) 1 << pad)) ? PAL_HIGH : PAL_LOW;
}

/**
 * @brief Writes a logical state on an output pad.
 * @note This function is not meant to be invoked directly by the
 * application code.
 * @note The @ref PAL provides a default software implementation of this
 * functionality, implement this function if can optimize it by using
 * special hardware functionalities or special coding.
 *
 * @param[in] port port identifier
 * @param[in] pad pad number within the port
 * @param[in] bit logical value, the value must be @p PAL_LOW or
 * @p PAL_HIGH
 *
 * @notapi
 */
void _pal_lld_writepad(ioportid_t port,uint8_t pad,uint8_t bit)
{
    if (bit == PAL_HIGH)
      port->PSOR |= 1 << pad;
    else
      port->PCOR = 1 << pad;
}

/**
 * @brief   Pad mode setup.
 * @details This function programs a pad with the specified mode.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad mode
 *
 * @notapi
 */

void _pal_lld_setpadmode(uint8_t port,uint8_t pad,iomode_t mode)
{ uint32_t Pad = 1<<pad;
  
  if(mode)
  {
    PORT_PTR(port)->PCR[pad] = PIN_CTRL_MASK & mode;

    if((PORT_MASK& mode)==PIN_INP)
      GPIO_PTR(port)->PDDR &= ~Pad;
    else
    { 
      if((PORT_MASK& mode)==PIN_OUT_L)
        GPIO_PTR(port)->PCOR=Pad;
      else
        GPIO_PTR(port)->PSOR=Pad;
      GPIO_PTR(port)->PDDR |= Pad;   
     }
   } 
 }    

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Kinetis I/O ports configuration.
 * @details Ports A-E clocks enabled.
 *
 * @param[in] config    the Kinetis ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config)
{
  int i, j;

  /* Enable clocking on all Ports */
  SIM->SCGC5 |= SIM_SCGC5_PORTA |
                SIM_SCGC5_PORTB |
                SIM_SCGC5_PORTC |
                SIM_SCGC5_PORTD |
                SIM_SCGC5_PORTE;

  /* Initial PORT and GPIO setup */
  for (i = 0; i < TOTAL_PORTS; i++)
    for (j = 0; j < PADS_PER_PORT; j++)
      pal_lld_setpadmode(config->ports[i].port,j,config->ports[i].pads[j]);
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,ioportmask_t mask,iomode_t mode)
{
  int i;
  uint8_t portnum=GPIO_NUM(port);
  
  (void)mask;

  for (i = 0; i < PADS_PER_PORT; i++)
    pal_lld_setpadmode(portnum, i, mode);
}

#endif /* HAL_USE_PAL */

/** @} */
