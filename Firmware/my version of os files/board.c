#include "mk22d5.h"
#include "core_cm4.h"
#include "power.h"
#include "ch.h"
#include "board.h"
#include "hal.h"
#include "pal_lld.h"
#include "rtc.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

// PORT B Handler
OSAL_IRQ_HANDLER(PORTB_IRQHandlerR)
{

}

// PORT C Handler
OSAL_IRQ_HANDLER(PORTC_IRQHandler)
{
  OSAL_IRQ_PROLOGUE();
  
  if(PORTC->ISFR & (1<<PORTC_MS_CLK))
  { 
    PORTC->ISFR = (1<<PORTC_MS_CLK);      // Clear PS/2 mouse IRQ
//    PS2_IQR(&PS2_State[PS2_1]);
  }
  OSAL_IRQ_EPILOGUE();
}

// PORT D Handler
OSAL_IRQ_HANDLER(PORTD_IRQHandler)
{   
//  OSAL_IRQ_PROLOGUE();
  
  if(PORTD->ISFR & (1<<PORTD_KBD_CLK))
  { 
    PORTD->ISFR = (1<<PORTD_KBD_CLK);     // Clear PS/2 kbd IRQ
//    PS2_IQR(&PS2_State[PS2_0]);    
  }
  if(PORTD->ISFR & (1<<PORTD_ENT_INT))
  {
    PORTD->ISFR = (1<<PORTD_ENT_INT);     // Clear Ethernet IRQ
  }
  
  if(PORTD->ISFR & (1<<PORTD_MENU_PWR))
  {
    PORTD->ISFR = (1<<PORTD_MENU_PWR);
  }
 // OSAL_IRQ_EPILOGUE();
}

PS2 PS2_State[2]=
{ 
  { GPIOD, 1<< PORTD_KBD_CLK, 1<< PORTD_KBD_DAT },
  { GPIOC, 1<< PORTC_MS_CLK, 1<< PORTC_MS_DAT }
};
 
const PALConfig pal_default_config=
{ {
    { IOPORTA,
      {
        /* PTA0*/ PORTx_PCRn_MUX(7),		// TCK    
				/* PTA1*/ PORTx_PCRn_MUX(2),		// UART0_RX
				/* PTA2*/ PORTx_PCRn_MUX(2),		// UART0_TX
				/* PTA2*/ //PORTx_PCRn_MUX(7),	// SWD_DO
        /* PTA3*/ PORTx_PCRn_MUX(7),		// SWD_DIO
				/* PTA4*/ PIN_GPIO_OUTLOW,	    // EZP_CS_b
      },
    },
    {
      IOPORTB,
      {
        /* PTB0*/ PORTx_PCRn_MUX(2)|		// I2C_SCL - open drain
                  PIN_OPENDRAIN,
				/* PTB1*/ PORTx_PCRn_MUX(2)|		// I2C_SDA - open drain
                  PIN_OPENDRAIN,
				/* PTB2*/ PIN_GPIO_INP|		      // I2C_IRQ - IRQ falling edge
                  PORTx_PCRn_IRQC(0x0a),
        /* PTB3*/ PIN_GPIO_INP,		      // CFG_DONE
				/* PTB16*/										  // CFG_STAT
				/* PTB17*/										  // CONFIG
      },
    },
    {
      IOPORTC,
      {
        /* PTC0*/ PIN_GPIO_OUTHIGH,     // SPI_SEL4
				/* PTC1*/ PIN_GPIO_OUTHIGH,     // SPI_SEL3
				/* PTC2*/ PIN_GPIO_OUTHIGH,     // SPI_SEL2

        /* PTC3*/ PIN_GPIO_INP|         // MS_CLK - open drain, pull down, filter
                  PIN_PULLUP|           // IRQ falling edge
                  PIN_OPENDRAIN|
                  PORTx_PCRn_PFE|
                  PORTx_PCRn_IRQC(0x0a),
				/* PTC4*/ PIN_GPIO_INP|         // MS_DAT - open drain, pull down
                  PIN_PULLUP|
                  PIN_OPENDRAIN|
                  PORTx_PCRn_PFE,        
				/* PTC5*/ PORTx_PCRn_MUX(2),		// SPI0_SCK
                 // PORTx_PCRn_SRE|
                 // PORTx_PCRn_DSE,
        /* PTC6*/ PORTx_PCRn_MUX(2),		// SPI0_SOUT
                 // PORTx_PCRn_SRE|
                 // PORTx_PCRn_DSE,
				/* PTC7*/ PORTx_PCRn_MUX(2)		  // SPI0_SIN

      },
    },
    {
      IOPORTD,
      {
        /* PTD0*/ PIN_GPIO_OUTHIGH,     // SPI_CS0
				/* PTD1*/ PIN_GPIO_OUTLOW,	    // CONFIG_EN
        
				/* PTD2*/ PIN_GPIO_INP|	        // KBD_CLK - open drain, pull down
                  PIN_PULLUP|           // IRQ falling edge
                  PIN_OPENDRAIN|
                  PORTx_PCRn_IRQC(0x0a),        
        /* PTD3*/ PIN_GPIO_INP|		      // KBD_DAT - open drain, pull down
                  PIN_PULLUP|
                  PIN_OPENDRAIN|
                  PORTx_PCRn_PFE,
        
				/* PTD4*/ PIN_GPIO_OUTHIGH,     // SPI_SEL1
				/* PTD5*/ PIN_GPIO_OUTHIGH,     // PWR_EN
        /* PTD6*/ PIN_GPIO_INP|	        // MENU/PWR
                  PIN_PULLUP|
                  PORTx_PCRn_PFE|
                  PORTx_PCRn_IRQC(0x0a),// Falling edge        
				/* PTD7*/ PIN_GPIO_INP,		      // ETH_INT
      },
    },			
	}
};
#endif

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */

void __early_init(void)
{
  /* Enable minimum clocking */
  SIM->SCGC5 |= SIM_SCGC5_PORTA|SIM_SCGC5_PORTD;
  SIM->SCGC6 = SIM_SCGC6_FTFL|SIM_SCGC6_RTC;
  SIM->SCGC7 = 0;
  
  /* Unlock the watchdog and set to allow updates */
  WDOG->UNLOCK = 0xC520;
  WDOG->UNLOCK = 0xD928;
  WDOG->STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
  
  rtc_lld_init();
  pal_lld_setpadmode(IOPORTD,PORTD_MENU_PWR,PORTx_PCRn_MUX(1)|PIN_PULLUP|PORTx_PCRn_PFE);

  LLStop();  
}

void boardInit(void)
{
  pal_lld_setpadmode(IOPORTB,PORTB_CFG_STAT,PIN_GPIO_INP);
	pal_lld_setpadmode(IOPORTB,PORTB_CONFIG,PIN_GPIO_OUTLOW);
  
  // Clear I/O Data register
  GPIOC->PCOR = (1<<PORTC_MS_CLK)|(1<<PORTC_MS_DAT);
  GPIOD->PCOR = (1<<PORTD_KBD_CLK)|(1<<PORTD_KBD_DAT);
}

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
/**
 * @brief   MMC_SPI card detection.
 */
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {
  uint8_t portvalue;
  uint32_t i;

  pal_lld_setpadmode(IOPORTC,PORTC_SPI_SEL2,PIN_GPIO_OUTLOW);
  
  osalSysLock();
  pal_lld_setpadmode(IOPORTC,PORTC_SPI_SEL2,PIN_GPIO_INP);
  
  for(i=MMC_POLL_WAIT;i;i--)
    ;
  
  portvalue= GPIO_PTR(IOPORTC)->PDIR;
  osalSysUnlock();
  
  pal_lld_setpadmode(IOPORTC,PORTC_SPI_SEL2,PIN_GPIO_OUTHIGH);
  
  return (portvalue & SPI_SD_MASK)?TRUE:FALSE;
}

/**
 * @brief   MMC_SPI card write protection detection.
 */
bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return FALSE;
}
#endif
