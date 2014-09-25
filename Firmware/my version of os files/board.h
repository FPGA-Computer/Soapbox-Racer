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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "mk22d5.h"
#include <inttypes.h>

#include "PS2.h"

/*
 * Setup for the Soapbox Racer.
 */

/*
 * Board identifier.
 */
#define BOARD_SoapboxRacer
#define BOARD_NAME                  "Soapbox Racer Rel 0"
#define SYSCLK_FREQ									48000000
#define BUSCLK_FREQ									SYSCLK_FREQ
#define PCLK												SYSCLK_FREQ

#define KINETIS_SERIAL_USE_UART0    1
#define DEFAULT_BAUD                38400
/*
 * IO pins assignments.
 */
#define PORTA_TCK                   0
#define PORTA_RX0                   1
#define PORTA_TX0                   2
#define PORTA_TMS                   3
#define PORTA_EZP_CS          		  4
#define PORTA_EXTAL0                18
#define PORTA_XTAL0                 19

#define PORTB_I2C_SCL          	    0
#define PORTB_I2C_SDA               1
#define PORTB_I2C_IRQ               2
#define PORTB_CFG_DONE              3
#define PORTB_CFG_STAT              16
#define PORTB_CONFIG                17

#define PORTC_SPI_SEL4              0
#define PORTC_SPI_SEL3              1
#define PORTC_SPI_SEL2              2
#define PORTC_MS_CLK			          3
#define PORTC_MS_DAT                4
#define PORTC_SPI_SCK               5
#define PORTC_SPI_SOUT              6
#define PORTC_SPI_SIN               7

#define PORTD_SPI_CS0               0
#define PORTD_CONF_EN               1
#define PORTD_KBD_CLK               2
#define PORTD_KBD_DAT               3
#define PORTD_SPI_SEL1              4
#define PORTD_PWR_EN                5
#define PORTD_MENU_PWR              6
#define PORTD_ENT_INT               7

// PS2 macros
#define PS2_0_Vect                  PIND_IRQn
#define PS2_1_Vect                  PINC_IRQn
#define PS2_PORTS                   2

// SPI enables
#define SPI_FPGA_PORT               &GPIOC->PDOR
#define SPI_FPGA_MASK               ((1<<PORTC_SPI_SEL3)|(1<<PORTC_SPI_SEL4))
#define SPI_FPGA_SEL0               0
#define SPI_FPGA_SEL1               (1<<PORTC_SPI_SEL3)
#define SPI_FPGA_SEL2               (1<<PORTC_SPI_SEL4)
#define SPI_FPGA_DIS                SPI_FPGA_MASK

#define SPI_CONF_PORT               &GPIOD->PDOR
#define SPI_CONF_MASK               (1<<PORTD_CONF_EN)
#define SPI_CONF_EN                 (1<<PORTD_CONF_EN)
#define SPI_CONF_DIS                0

#define SPI_SD_PORT                 &GPIOC->PDOR
#define SPI_SD_MASK                 (1<<PORTC_SPI_SEL2)
#define SPI_SD_SEL                  0
#define SPI_SD_DIS                  (1<<PORTC_SPI_SEL2)

#define SPI_FLASH_PORT              &GPIOD->PDOR
#define SPI_FLASH_MASK              (1<<PORTD_SPI_SEL1)
#define SPI_FLASH_SEL               0
#define SPI_FLASH_DIS               (1<<PORTD_SPI_SEL1)

#define SPI_CTAR(BITS,BITRATE)      (SPIx_CTARn_DBR|SPIx_CTARn_FMSZ(BITS-1)|SPIx_CTARn_BR(BITRATE))
#define SPI_CTAR_8_FAST             SPI_CTAR(8,0)
#define SPI_CTAR_8_SLOW             (SPI_CTAR(8,7)|SPIx_CTARn_PCSSCK(7)|SPIx_CTARn_PASC(7)|SPIx_CTARn_PDT(7))

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
