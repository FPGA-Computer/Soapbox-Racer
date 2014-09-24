/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : Cpu.c
**     Project     : power
**     Processor   : MK22DX256VLF5
**     Component   : MK22DX256LF5
**     Version     : Component 01.011, Driver 01.04, CPU db: 3.00.000
**     Datasheet   : K22RM, Rev.4, Feb 2013
**     Compiler    : Keil ARM C/C++ Compiler
**     Date/Time   : 2014-08-24, 09:28, # CodeGen: 2
**     Abstract    :
**
**     Settings    :
**
**     Contents    :
**         SetClockConfiguration - LDD_TError Cpu_SetClockConfiguration(LDD_TClockConfiguration ModeID);
**         GetClockConfiguration - LDD_TClockConfiguration Cpu_GetClockConfiguration(void);
**
**     Copyright : 1997 - 2014 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/

#include "mk22d5.h"
#include "core_cm4.h"
#include "power.h"
#include "ch.h"
#include "hal.h"

// LLW_IRQHandler
CH_FAST_IRQ_HANDLER(Vector94)
{
  /* LLWU_F1: WUF7=1,WUF6=1,WUF5=1,WUF4=1,WUF3=1,WUF2=1,WUF1=1,WUF0=1 */
  LLWU->F1 = 0xff;                      /* Clear external pin flags */
  /* LLWU_F2: WUF15=1,WUF14=1,WUF13=1,WUF12=1,WUF11=1,WUF10=1,WUF9=1,WUF8=1 */
  LLWU->F2 = 0xff;                      /* Clear external pin flags */

  /* LLWU_FILT1: FILTF=1 */
  LLWU->FILT1 |= LLWU_FILT1_FILTF; /* Clear filter flag */
  /* LLWU_FILT2: FILTF=1 */
  LLWU->FILT2 |= LLWU_FILT2_FILTF; /* Clear filter flag */
}

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */

void LLStop(void)
{  
  nvicEnableVector(LLWU_IRQn,0);
  LLWU->PE4 = LLWU_PE4_WUPE15(0x02);
  
  SIM->SOPT1CFG = SIM_SOPT1CFG_UVSWE;
  SIM->SOPT1 |= SIM_SOPT1_USBVSTBY;
  SIM->SOPT1CFG = SIM_SOPT1CFG_USSWE;
  SIM->SOPT1 |= SIM_SOPT1_USBSSTBY;
  
  // Allow VLPR mode
  SMC->PMPROT = SMC_PMPROT_AVLLS|SMC_PMPROT_ALLS|SMC_PMPROT_AVLP; 
  // RUNM = 10 -> VLPR mode
  SMC->PMCTRL = SMC_PMCTRL_STOPM(0x04);
  
  // VLLS3 keep RAM2 active
  SMC->VLLSCTRL = SMC_VLLSCTRL_VLLSM(0x03);
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   /* set DEEPSLEEP bit */

  /* for product, not during debugging 
  #ifdef __GNUC__
  __WFI();
  #else
  __wfi(); 
  #endif
*/
}

/**
 * @brief   MK22D5 clock initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function is meant to be invoked early during the system
 *          initialization, it is usually invoked from the file
 *          @p board.c.
 * @todo    This function needs to be more generic.
 *
 * @special
 */
void mk22d5_clock_init(void)
{
    /* Enable clocks for always used peripherals */
    SIM->SCGC6 = SIM_SCGC6_FTFL |
                 //SIM_SCGC6_DMAMUX |
                 //SIM_SCGC6_SPI0 |
                 //SIM_SCGC6_I2S |				// I2S not used
                 //SIM_SCGC6_CRC |
                 //SIM_SCGC6_USBDCD |		// USB Charger detector not used
                 //SIM_SCGC6_PDB |
                 SIM_SCGC6_PIT |
                 //SIM_SCGC6_FTM0 |
                 //SIM_SCGC6_FTM1 |
                 //SIM_SCGC6_ADC0K |			// ADC not used
                 SIM_SCGC6_RTC;
		
    /* Release I/O pins hold, if just woke up from VLLS mode */
    if (PMC->REGSC & PMC_REGSC_ACKISO_MASK) PMC->REGSC |= PMC_REGSC_ACKISO_MASK;

    /* Clock config divisors: 48 MHz core, 48 MHz bus, 24 MHz flash */
    SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(0)|SIM_CLKDIV1_OUTDIV4(1);		

    /* Enable PORTA - crystal connected to PA18/PA19 */
    SIM->SCGC5 |= SIM_SCGC5_PORTA;


    /* USB uses PLL clock, trace is CPU clock, CLKOUT = OSCERCLK0 */
    SIM->SOPT2 = SIM_SOPT2_USBSRC|SIM_SOPT2_PLLFLLSEL|
                 SIM_SOPT2_TRACECLKSEL|SIM_SOPT2_CLKOUTSEL(6);

    /* EXTAL0 and XTAL0 */
    PORTA->PCR[PORTA_EXTAL0] = 0;
    PORTA->PCR[PORTA_XTAL0] = 0;				

    /* Enable OSC, 8-32 MHz range, low power mode */
    MCG->C2 = MCG_C2_RANGE0(2)|MCG_C2_EREFS0;

    /* Enable capacitors for crystal */
		// Crystal load capacitance = 6pF -> load = 12pF
    OSC->CR = OSC_CR_ERCLKEN|OSC_CR_SC4P|OSC_CR_SC8P;		

    MCG->C1 = MCG_C1_CLKS(2)|MCG_C1_FRDIV(5);
    MCG->C5 = MCG_C5_PRDIV0(0x0B);
    MCG->C6 = MCG_C6_VDIV0(0);		
		
    /* Wait for crystal oscillator to begin */
    while ((MCG->S & MCG_S_OSCINIT0) == 0);

    /* Wait for the FLL to use the oscillator */
    while ((MCG->S & MCG_S_IREFST) != 0);

    /* Wait for the MCGOUTCLK to use the oscillator */
    while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));
    
    /* Wait until external reference clock is selected as MCG output */
	  while((MCG->S & 0x0CU)!= 0x08U);
		MCG->C6 = (MCG_C6_PLLS|MCG_C6_VDIV0(0x00));
    
    /* Wait until external reference clock is selected as MCG output */
	  while((MCG->S & 0x0CU)!= 0x08U){;} 
		
    /* Switch to PLL as clock source */
    MCG->C1 = MCG_C1_CLKS(0)|MCG_C1_FRDIV(5)|MCG_C1_IRCLKEN|MCG_C1_IREFSTEN;

    /* Wait for PLL clock to be used */
    while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3));
			
    /*
     * Now in PEE mode
     */
}

void RunMode(void)
{ 
  mk22d5_clock_init();
  GPIOD->PSOR = PORTD_PWR_EN;
  GPIOD->PSOR = PORTD_SPI_SEL1;
}
