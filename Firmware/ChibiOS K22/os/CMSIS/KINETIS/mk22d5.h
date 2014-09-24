/*
 * Copyright (C) 2014 Fabio Utzig, http://fabioutzig.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _MK22D5_H_
#define _MK22D5_H_

/* modified from MK20D5.h by K. C. Lee
   IRQn DMA4_IRQn -DMA15_IRQn added
   PMC, SMC definition
   SIM_SCGC6_DMAMUX             ((uint32_t)0x00000002)    not 0x00000010
*/
#define K22_DMA_CHANNELS	16

/*
 * ==============================================================
 * ---------- Interrupt Number Definition -----------------------
 * ==============================================================
 */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ****************/
  InitialSP_IRQn                = -15,
  InitialPC_IRQn                = -15,
  NonMaskableInt_IRQn           = -14,
  HardFault_IRQn                = -13,
  MemoryManagement_IRQn         = -12,
  BusFault_IRQn                 = -11,
  UsageFault_IRQn               = -10,
  SVCall_IRQn                   = -5,
  DebugMonitor_IRQn             = -4,
  PendSV_IRQn                   = -2,
  SysTick_IRQn                  = -1,

/******  K22 Specific Interrupt Numbers ***********************/
  DMA0_IRQn                     = 0,
  DMA1_IRQn                     = 1,
  DMA2_IRQn                     = 2,
  DMA3_IRQn                     = 3,
  DMA4_IRQn                     = 4,
  DMA5_IRQn                     = 5,
  DMA6_IRQn                     = 6,
  DMA7_IRQn                     = 7,
  DMA8_IRQn                     = 8,
  DMA9_IRQn                     = 9,
  DMA10_IRQn                    = 10,
  DMA11_IRQn                    = 11,
  DMA12_IRQn                    = 12,
  DMA13_IRQn                    = 13,
  DMA14_IRQn                    = 14,
  DMA15_IRQn                    = 15, 
  DMAError_IRQn                 = 16,
  DMA_IRQn                      = 17,
  FlashMemComplete_IRQn         = 18,
  FlashMemReadCollision_IRQn    = 19,
  LowVoltageWarning_IRQn        = 20,
  LLWU_IRQn                     = 21,
  WDOG_IRQn                     = 22,

  I2C0_IRQn                     = 24,

  SPI0_IRQn                     = 26,

  I2S0_IRQn                     = 28,
  I2S1_IRQn                     = 29,
  UART0LON_IRQn                 = 30,
  UART0Status_IRQn              = 31,
  UART0Error_IRQn               = 32,
  UART1Status_IRQn              = 33,
  UART1Error_IRQn               = 34,
  UART2Status_IRQn              = 35,
  UART2Error_IRQn               = 36,

  ADC0_IRQn                     = 39,
  CMP0_IRQn                     = 40,
  CMP1_IRQn                     = 41,
  FTM0_IRQn                     = 42,
  FTM1_IRQn                     = 43,

  CMT_IRQn                      = 45,
  RTCAlarm_IRQn                 = 46,
  RTCSeconds_IRQn               = 47,
  PITChannel0_IRQn              = 48,
  PITChannel1_IRQn              = 49,
  PITChannel2_IRQn              = 50,
  PITChannel3_IRQn              = 51,
  PDB_IRQn                      = 52,
  USB_OTG_IRQn                  = 53,
  USBChargerDetect_IRQn         = 54,

  TSI_IRQn                      = 56,
  MCG_IRQn                      = 57,
  LowPowerTimer_IRQn            = 58,
  PINA_IRQn                     = 59,
  PINB_IRQn                     = 60,
  PINC_IRQn                     = 61,
  PIND_IRQn                     = 62,
  PINE_IRQn                     = 63,
  SoftInitInt_IRQn              = 64
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/**
 * @brief K20x Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
#define __MPU_PRESENT             0
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    0

#include "core_cm4.h"            /* Cortex-M4 processor and core peripherals */

typedef struct
{
  __IO uint32_t SOPT1;
  __IO uint32_t SOPT1CFG;
       uint32_t RESERVED0[1023];
  __IO uint32_t SOPT2;
       uint32_t RESERVED1[1];
  __IO uint32_t SOPT4;
  __IO uint32_t SOPT5;
       uint32_t RESERVED2[1];
  __IO uint32_t SOPT7;
       uint32_t RESERVED3[2];
  __I  uint32_t SDID;
       uint32_t RESERVED4[3];
  __IO uint32_t SCGC4;
  __IO uint32_t SCGC5;
  __IO uint32_t SCGC6;
  __IO uint32_t SCGC7;
  __IO uint32_t CLKDIV1;
  __IO uint32_t CLKDIV2;
  __I  uint32_t FCFG1;
  __I  uint32_t FCFG2;
  __I  uint32_t UIDH;
  __I  uint32_t UIDMH;
  __I  uint32_t UIDML;
  __I  uint32_t UIDL;
} SIM_TypeDef;

typedef struct
{
  __IO uint8_t  PE1;
  __IO uint8_t  PE2;
  __IO uint8_t  PE3;
  __IO uint8_t  PE4;
  __IO uint8_t  ME;
  __IO uint8_t  F1;
  __IO uint8_t  F2;
  __I  uint8_t  F3;
  __IO uint8_t  FILT1;
  __IO uint8_t  FILT2;
} LLWU_TypeDef;

typedef struct
{
  __IO uint32_t PCR[32];
  __O  uint32_t GPCLR;
  __O  uint32_t GPCHR;
       uint32_t RESERVED0[6];
  __IO uint32_t ISFR;
} PORT_TypeDef;

typedef struct
{
  __IO uint8_t  C1;
  __IO uint8_t  C2;
  __IO uint8_t  C3;
  __IO uint8_t  C4;
  __IO uint8_t  C5;
  __IO uint8_t  C6;
  __I  uint8_t  S;
       uint8_t  RESERVED0[1];
  __IO uint8_t  SC;
       uint8_t  RESERVED1[1];
  __IO uint8_t  ATCVH;
  __IO uint8_t  ATCVL;
  __IO uint8_t  C7;
  __IO uint8_t  C8;
} MCG_TypeDef;

typedef struct
{
  __IO uint8_t  CR;
} OSC_TypeDef;

typedef struct
{
    __IO uint32_t CR;                   /**< Control Register, offset: 0x0 */
    __I  uint32_t ES;                   /**< Error Status Register, offset: 0x4 */
         uint8_t  RESERVED_0[4];
    __IO uint32_t ERQ;                  /**< Enable Request Register, offset: 0xC */
         uint8_t  RESERVED_1[4];
    __IO uint32_t EEI;                  /**< Enable Error Interrupt Register, offset: 0x14 */
    __O  uint8_t  CEEI;                  /**< Clear Enable Error Interrupt Register, offset: 0x18 */
    __O  uint8_t  SEEI;                  /**< Set Enable Error Interrupt Register, offset: 0x19 */
    __O  uint8_t  CERQ;                  /**< Clear Enable Request Register, offset: 0x1A */
    __O  uint8_t  SERQ;                  /**< Set Enable Request Register, offset: 0x1B */
    __O  uint8_t  CDNE;                  /**< Clear DONE Status Bit Register, offset: 0x1C */
    __O  uint8_t  SSRT;                  /**< Set START Bit Register, offset: 0x1D */
    __O  uint8_t  CERR;                  /**< Clear Error Register, offset: 0x1E */
    __O  uint8_t  CINT;                  /**< Clear Interrupt Request Register, offset: 0x1F */
         uint8_t  RESERVED_2[4];
    __IO uint32_t INT;                  /**< Interrupt Request Register, offset: 0x24 */
         uint8_t  RESERVED_3[4];
    __IO uint32_t ERR;                  /**< Error Register, offset: 0x2C */
         uint8_t  RESERVED_4[4];
    __IO uint32_t HRS;                  /**< Hardware Request Status Register, offset: 0x34 */
         uint8_t  RESERVED_5[200];
    __IO uint8_t  DCHPRI3;               /**< Channel n Priority Register, offset: 0x100 */
    __IO uint8_t  DCHPRI2;               /**< Channel n Priority Register, offset: 0x101 */
    __IO uint8_t  DCHPRI1;               /**< Channel n Priority Register, offset: 0x102 */
    __IO uint8_t  DCHPRI0;               /**< Channel n Priority Register, offset: 0x103 */
    __IO uint8_t  DCHPRI7;               /**< Channel n Priority Register, offset: 0x104 */
    __IO uint8_t  DCHPRI6;               /**< Channel n Priority Register, offset: 0x105 */
    __IO uint8_t  DCHPRI5;               /**< Channel n Priority Register, offset: 0x106 */
    __IO uint8_t  DCHPRI4;               /**< Channel n Priority Register, offset: 0x107 */		
    __IO uint8_t  DCHPRI11;              /**< Channel n Priority Register, offset: 0x108 */
    __IO uint8_t  DCHPRI10;              /**< Channel n Priority Register, offset: 0x109 */
    __IO uint8_t  DCHPRI9;               /**< Channel n Priority Register, offset: 0x10a */
    __IO uint8_t  DCHPRI8;               /**< Channel n Priority Register, offset: 0x10b */		
    __IO uint8_t  DCHPRI15;              /**< Channel n Priority Register, offset: 0x10c */
    __IO uint8_t  DCHPRI14;              /**< Channel n Priority Register, offset: 0x10d */
    __IO uint8_t  DCHPRI13;              /**< Channel n Priority Register, offset: 0x10e */
    __IO uint8_t  DCHPRI12;              /**< Channel n Priority Register, offset: 0x10f */		
		
         uint8_t  RESERVED_6[3824];
    struct {                            /* offset: 0x1000, array step: 0x20 */
        __IO uint32_t SADDR;                /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
        __IO uint16_t SOFF;                 /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
        __IO uint16_t ATTR;                 /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
        union {                             /* offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLNO;          /**< TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLOFFNO;       /**< TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLOFFYES;      /**< TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20 */
        };
        __IO uint32_t SLAST;                /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
        __IO uint32_t DADDR;                /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
        __IO uint16_t DOFF;                 /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
        union {                             /* offset: 0x1016, array step: 0x20 */
            __IO uint16_t CITER_ELINKNO;        /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
            __IO uint16_t CITER_ELINKYES;       /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
        };
        __IO uint32_t DLAST_SGA;            /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
        __IO uint16_t CSR;                  /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
        union {                             /* offset: 0x101E, array step: 0x20 */
            __IO uint16_t BITER_ELINKNO;        /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
            __IO uint16_t BITER_ELINKYES;       /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
        };
    } TCD[16];
} DMA_TypeDef;

typedef struct
{
    __IO uint8_t CHCFG[16];             /**< Channel Configuration Register, array offset: 0x0, array step: 0x1 */
} DMAMUX_TypeDef;

typedef struct
{
  __IO uint32_t SC;         /* Status and Control */
  __IO uint32_t CNT;        /* Counter */
  __IO uint32_t MOD;        /* Modulo */
   struct FTM_Channel {
     __IO uint32_t CnSC;     /* Channel Status and Control */
     __IO uint32_t CnV;      /* Channel Value */
   } CHANNEL[8];
  __IO uint32_t CNTIN;      /* Counter Initial Value */
  __IO uint32_t STATUS;     /* Capture and Compare Status */
  __IO uint32_t MODE;       /* Features Mode Selection */
  __IO uint32_t SYNC;       /* Synchronization */
  __IO uint32_t OUTINIT;    /* Initial State for Channels Output */
  __IO uint32_t OUTMASK;    /* Output Mask */
  __IO uint32_t COMBINE;    /* Function for Linked Channels */
  __IO uint32_t DEADTIME;   /* Deadtime Insertion Control */
  __IO uint32_t EXTTRIG;    /* FTM External Trigger */
  __IO uint32_t POL;        /* Channels Polarity */
  __IO uint32_t FMS;        /* Fault Mode Status */
  __IO uint32_t FILTER;     /* Input Capture Filter Control */
  __IO uint32_t FLTCTRL;    /* Fault Control */
  __IO uint32_t QDCTRL;     /* Quadrature Decode Control and Status */
  __IO uint32_t CONF;       /* Configuration */
  __IO uint32_t FTLPOL;     /* FTM Fault Input Polarity */
  __IO uint32_t SYNCONF;    /* Synchronization Configuration */
  __IO uint32_t INVCTRL;    /* FTM Inverting Control */
  __IO uint32_t SWOCTRL;    /* FTM Software Output Control */
  __IO uint32_t PWMLOAD;    /* FTM PWM Load */
} FTM_TypeDef;

typedef struct
{
  __IO uint32_t SC1A;           // offset: 0x00
  __IO uint32_t SC1B;           // offset: 0x04
  __IO uint32_t CFG1;           // offset: 0x08
  __IO uint32_t CFG2;           // offset: 0x0C
  __I  uint32_t RA;             // offset: 0x10
  __I  uint32_t RB;             // offset: 0x14
  __IO uint32_t CV1;            // offset: 0x18
  __IO uint32_t CV2;            // offset: 0x1C
  __IO uint32_t SC2;            // offset: 0x20
  __IO uint32_t SC3;            // offset: 0x24
  __IO uint32_t OFS;            // offset: 0x28
  __IO uint32_t PG;             // offset: 0x2C
  __IO uint32_t MG;             // offset: 0x30
  __IO uint32_t CLPD;           // offset: 0x34
  __IO uint32_t CLPS;           // offset: 0x38
  __IO uint32_t CLP4;           // offset: 0x3C
  __IO uint32_t CLP3;           // offset: 0x40
  __IO uint32_t CLP2;           // offset: 0x44
  __IO uint32_t CLP1;           // offset: 0x48
  __IO uint32_t CLP0;           // offset: 0x4C
       uint32_t RESERVED0[1];   // offset: 0x50
  __IO uint32_t CLMD;           // offset: 0x54
  __IO uint32_t CLMS;           // offset: 0x58
  __IO uint32_t CLM4;           // offset: 0x5C
  __IO uint32_t CLM3;           // offset: 0x60
  __IO uint32_t CLM2;           // offset: 0x64
  __IO uint32_t CLM1;           // offset: 0x68
  __IO uint32_t CLM0;           // offset: 0x6C
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CSR;
  __IO uint32_t PSR;
  __IO uint32_t CMR;
  __I  uint32_t CNR;
} LPTMR_TypeDef;

typedef struct
{
  __IO uint32_t GENCS;
  __IO uint32_t DATA;
  __IO uint32_t TSHD;
} TSI_TypeDef;

typedef struct
{
  __IO uint32_t PDOR;
  __IO uint32_t PSOR;
  __IO uint32_t PCOR;
  __IO uint32_t PTOR;
  __IO uint32_t PDIR;
  __IO uint32_t PDDR;
} GPIO_TypeDef;

/** SPI - Peripheral register structure */
typedef struct {
  __IO uint32_t MCR;                /**< DSPI Module Configuration Register, offset: 0x0 */
       uint32_t RESERVED0[1];
  __IO uint32_t TCR;                /**< DSPI Transfer Count Register, offset: 0x8 */
  union {                           /* offset: 0xC */
    __IO uint32_t CTAR[2];          /**< DSPI Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
    __IO uint32_t CTAR_SLAVE[1];    /**< DSPI Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
  };
       uint32_t RESERVED1[6];
  __IO uint32_t SR;                 /**< DSPI Status Register, offset: 0x2C */
  __IO uint32_t RSER;               /**< DSPI DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
  union {                           /* offset: 0x34 */
    __IO uint32_t PUSHR;            /**< DSPI PUSH TX FIFO Register In Master Mode, offset: 0x34 */
    __IO uint32_t PUSHR_SLAVE;      /**< DSPI PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
  };
  __I  uint32_t POPR;               /**< DSPI POP RX FIFO Register, offset: 0x38 */
  __I  uint32_t TXFR[4];            /**< DSPI Transmit FIFO Registers, offset: 0x3C */
       uint32_t RESERVED2[12];
  __I  uint32_t RXFR[4];            /**< DSPI Receive FIFO Registers, offset: 0x7C */
} SPI_TypeDef;

typedef struct
{
  __IO uint8_t  A1;
  __IO uint8_t  F;
  __IO uint8_t  C1;
  __IO uint8_t  S;
  __IO uint8_t  D;
  __IO uint8_t  C2;
  __IO uint8_t  FLT;
  __IO uint8_t  RA;
  __IO uint8_t  SMB;
  __IO uint8_t  A2;
  __IO uint8_t  SLTH;
  __IO uint8_t  SLTL;
} I2C_TypeDef;

typedef struct
{
  __IO uint8_t  BDH;
  __IO uint8_t  BDL;
  __IO uint8_t  C1;
  __IO uint8_t  C2;
  __I  uint8_t  S1;
  __IO uint8_t  S2;
  __IO uint8_t  C3;
  __IO uint8_t  D;
  __IO uint8_t  MA1;
  __IO uint8_t  MA2;
  __IO uint8_t  C4;
  __IO uint8_t  C5;
  __I  uint8_t  ED;
  __IO uint8_t  MODEM;
  __IO uint8_t  IR;
       uint8_t RESERVED0[1];
  __IO uint8_t  PFIFO;
  __IO uint8_t  CFIFO;
  __IO uint8_t  SFIFO;
  __IO uint8_t  TWFIFO;
  __I  uint8_t  TCFIFO;
  __IO uint8_t  RWFIFO;
  __I  uint8_t  RCFIFO;
       uint8_t RESERVED1[1];
  __IO uint8_t  C7816;
  __IO uint8_t  IE7816;
  __IO uint8_t  IS7816;
  union {
    __IO uint8_t  WP7816T0;
    __IO uint8_t  WP7816T1;
  };
  __IO uint8_t  WN7816;
  __IO uint8_t  WF7816;
  __IO uint8_t  ET7816;
  __IO uint8_t  TL7816;
       uint8_t RESERVED2[2];
  __IO uint8_t  C6;
  __IO uint8_t  PCTH;
  __IO uint8_t  PCTL;
  __IO uint8_t  B1T;
  __IO uint8_t  SDTH;
  __IO uint8_t  SDTL;
  __IO uint8_t  PRE;
  __IO uint8_t  TPL;
  __IO uint8_t  IE;
  __IO uint8_t  WB;
  __IO uint8_t  S3;
  __IO uint8_t  S4;
  __I  uint8_t  RPL;
  __I  uint8_t  RPREL;
  __IO uint8_t  CPW;
  __IO uint8_t  RIDT;
  __IO uint8_t  TIDT;
} UART_TypeDef;

typedef struct
{
  __IO uint16_t STCTRLH;
  __IO uint16_t STCTRLL;
  __IO uint16_t TOVALH;
  __IO uint16_t TOVALL;
  __IO uint16_t WINH;
  __IO uint16_t WINL;
  __IO uint16_t REFRESH;
  __IO uint16_t UNLOCK;
  __IO uint16_t TMROUTH;
  __IO uint16_t TMROUTL;
  __IO uint16_t RSTCNT;
  __IO uint16_t PRESC;
} WDOG_TypeDef;

typedef struct {
  __I  uint8_t  USB0_PERID;          // 0x00
       uint8_t  RESERVED0[3];
  __I  uint8_t  USB0_IDCOMP;         // 0x04
       uint8_t  RESERVED1[3];
  __I  uint8_t  USB0_REV;            // 0x08
       uint8_t  RESERVED2[3];
  __I  uint8_t  USB0_ADDINFO;        // 0x0C
       uint8_t  RESERVED3[3];
  __IO uint8_t  USB0_OTGISTAT;       // 0x10
       uint8_t  RESERVED4[3];
  __IO uint8_t  USB0_OTGICR;         // 0x14
       uint8_t  RESERVED5[3];
  __IO uint8_t  USB0_OTGSTAT;        // 0x18
       uint8_t  RESERVED6[3];
  __IO uint8_t  USB0_OTGCTL;         // 0x1C
       uint8_t  RESERVED7[99];
  __IO uint8_t  USB0_ISTAT;          // 0x80
       uint8_t  RESERVED8[3];
  __IO uint8_t  USB0_INTEN;          // 0x84
       uint8_t  RESERVED9[3];
  __IO uint8_t  USB0_ERRSTAT;        // 0x88
       uint8_t  RESERVED10[3];
  __IO uint8_t  USB0_ERREN;          // 0x8C
       uint8_t  RESERVED11[3];
  __I  uint8_t  USB0_STAT;           // 0x90
       uint8_t  RESERVED12[3];
  __IO uint8_t  USB0_CTL;            // 0x94
       uint8_t  RESERVED13[3];
  __IO uint8_t  USB0_ADDR;           // 0x98
       uint8_t  RESERVED14[3];
  __IO uint8_t  USB0_BDTPAGE1;       // 0x9C
       uint8_t  RESERVED15[3];
  __IO uint8_t  USB0_FRMNUML;        // 0xA0
       uint8_t  RESERVED16[3];
  __IO uint8_t  USB0_FRMNUMH;        // 0xA4
       uint8_t  RESERVED17[3];
  __IO uint8_t  USB0_TOKEN;          // 0xA8
       uint8_t  RESERVED18[3];
  __IO uint8_t  USB0_SOFTHLD;        // 0xAC
       uint8_t  RESERVED19[3];
  __IO uint8_t  USB0_BDTPAGE2;       // 0xB0
       uint8_t  RESERVED20[3];
  __IO uint8_t  USB0_BDTPAGE3;       // 0xB4
       uint8_t  RESERVED21[11];
  __IO uint8_t  USB0_ENDPT0;         // 0xC0
       uint8_t  RESERVED22[3];
  __IO uint8_t  USB0_ENDPT1;         // 0xC4
       uint8_t  RESERVED23[3];
  __IO uint8_t  USB0_ENDPT2;         // 0xC8
       uint8_t  RESERVED24[3];
  __IO uint8_t  USB0_ENDPT3;         // 0xCC
       uint8_t  RESERVED25[3];
  __IO uint8_t  USB0_ENDPT4;         // 0xD0
       uint8_t  RESERVED26[3];
  __IO uint8_t  USB0_ENDPT5;         // 0xD4
       uint8_t  RESERVED27[3];
  __IO uint8_t  USB0_ENDPT6;         // 0xD8
       uint8_t  RESERVED28[3];
  __IO uint8_t  USB0_ENDPT7;         // 0xDC
       uint8_t  RESERVED29[3];
  __IO uint8_t  USB0_ENDPT8;         // 0xE0
       uint8_t  RESERVED30[3];
  __IO uint8_t  USB0_ENDPT9;         // 0xE4
       uint8_t  RESERVED31[3];
  __IO uint8_t  USB0_ENDPT10;        // 0xE8
       uint8_t  RESERVED32[3];
  __IO uint8_t  USB0_ENDPT11;        // 0xEC
       uint8_t  RESERVED33[3];
  __IO uint8_t  USB0_ENDPT12;        // 0xF0
       uint8_t  RESERVED34[3];
  __IO uint8_t  USB0_ENDPT13;        // 0xF4
       uint8_t  RESERVED35[3];
  __IO uint8_t  USB0_ENDPT14;        // 0xF8
       uint8_t  RESERVED36[3];
  __IO uint8_t  USB0_ENDPT15;        // 0xFC
       uint8_t  RESERVED37[3];
  __IO uint8_t  USB0_USBCTRL;        // 0x100
       uint8_t  RESERVED38[3];
  __I  uint8_t  USB0_OBSERVE;        // 0x104
       uint8_t  RESERVED39[3];
  __IO uint8_t  USB0_CONTROL;        // 0x108
       uint8_t  RESERVED40[3];
  __IO uint8_t  USB0_USBTRC0;        // 0x10C
       uint8_t  RESERVED41[7];
  __IO uint8_t  USB0_USBFRMADJUST;   // 0x114
} USBOTG_TypeDef;


typedef struct
{
    __IO uint8_t LVDSC1;                /**< Low Voltage Detect Status and Control 1 Register, offset: 0x0 */
    __IO uint8_t LVDSC2;                /**< Low Voltage Detect Status and Control 2 Register, offset: 0x1 */
    __IO uint8_t REGSC;                 /**< Regulator Status and Control Register, offset: 0x2 */
} PMC_TypeDef;

typedef struct
{
    __IO uint8_t PMPROT;                /**< Power Mode Protection Register, offset: 0x0 */
    __IO uint8_t PMCTRL;                /**< Power Mode Control Register, offset: 0x1 */
    __IO uint8_t VLLSCTRL;              /**< VLLS Control Register, offset: 0x2 */
    __I  uint8_t PMSTAT;                /**< Power Mode Status Register, offset: 0x3 */
} SMC_TypeDef;

typedef struct
{
    __IO uint32_t TSR;                  /**< RTC Time Seconds Register, offset: 0x0 */
    __IO uint32_t TPR;                  /**< RTC Time Prescaler Register, offset: 0x4 */
    __IO uint32_t TAR;                  /**< RTC Time Alarm Register, offset: 0x8 */
    __IO uint32_t TCR;                  /**< RTC Time Compensation Register, offset: 0xC */
    __IO uint32_t CR;                   /**< RTC Control Register, offset: 0x10 */
    __IO uint32_t SR;                   /**< RTC Status Register, offset: 0x14 */
    __IO uint32_t LR;                   /**< RTC Lock Register, offset: 0x18 */
    __IO uint32_t IER;                  /**< RTC Interrupt Enable Register, offset: 0x1C */
         uint8_t  RESERVED_0[2016];
    __IO uint32_t WAR;                  /**< RTC Write Access Register, offset: 0x800 */
    __IO uint32_t RAR;                  /**< RTC Read Access Register, offset: 0x804 */
} RTC_TypeDef;

/****************************************************************/
/*                  Peripheral memory map                       */
/****************************************************************/
#define DMA_BASE                ((uint32_t)0x40008000)
#define DMA_TCD0_BASE           ((uint32_t)0x40009000)
#define DMA_TCD1_BASE           ((uint32_t)0x40009020)
#define DMA_TCD2_BASE           ((uint32_t)0x40009040)
#define DMA_TCD3_BASE           ((uint32_t)0x40009060)
#define DMA_TCD4_BASE           ((uint32_t)0x40009080)
#define DMA_TCD5_BASE           ((uint32_t)0x400090a0)
#define DMA_TCD6_BASE           ((uint32_t)0x400090c0)
#define DMA_TCD7_BASE           ((uint32_t)0x400090e0)
#define DMA_TCD8_BASE           ((uint32_t)0x40009100)
#define DMA_TCD9_BASE           ((uint32_t)0x40009120)
#define DMA_TCD10_BASE          ((uint32_t)0x40009140)
#define DMA_TCD11_BASE          ((uint32_t)0x40009160)
#define DMA_TCD12_BASE          ((uint32_t)0x40009180)
#define DMA_TCD13_BASE          ((uint32_t)0x400091a0)
#define DMA_TCD14_BASE          ((uint32_t)0x400091c0)
#define DMA_TCD15_BASE          ((uint32_t)0x400091e0)

#define DMAMUX_BASE             ((uint32_t)0x40021000)
#define SPI0_BASE               ((uint32_t)0x4002C000)
#define TPM0_BASE               ((uint32_t)0x40038000)
#define TPM1_BASE               ((uint32_t)0x40039000)
#define TPM2_BASE               ((uint32_t)0x4003A000)
#define ADC0_BASE               ((uint32_t)0x4003B000)

#define RTC_BASE                ((uint32_t)0x4003D000)

#define LPTMR0_BASE             ((uint32_t)0x40040000)
#define TSI0_BASE               ((uint32_t)0x40045000)
#define SIM_BASE                ((uint32_t)0x40047000)
#define PORTA_BASE              ((uint32_t)0x40049000)
#define PORTB_BASE              ((uint32_t)0x4004A000)
#define PORTC_BASE              ((uint32_t)0x4004B000)
#define PORTD_BASE              ((uint32_t)0x4004C000)
#define PORTE_BASE              ((uint32_t)0x4004D000)
#define WDOG_BASE               ((uint32_t)0x40052000)
#define MCG_BASE                ((uint32_t)0x40064000)
#define OSC0_BASE               ((uint32_t)0x40065000)
#define I2C0_BASE               ((uint32_t)0x40066000)
#define UART0_BASE              ((uint32_t)0x4006A000)
#define UART1_BASE              ((uint32_t)0x4006B000)
#define UART2_BASE              ((uint32_t)0x4006C000)
#define USBOTG_BASE             ((uint32_t)0x40072000)
#define LLWU_BASE               ((uint32_t)0x4007C000)

#define PMC_BASE                ((uint32_t)0x4007D000)
#define SMC_BASE                ((uint32_t)0x4007E000)

#define GPIOA_BASE              ((uint32_t)0x400FF000)
#define GPIOB_BASE              ((uint32_t)0x400FF040)
#define GPIOC_BASE              ((uint32_t)0x400FF080)
#define GPIOD_BASE              ((uint32_t)0x400FF0C0)
#define GPIOE_BASE              ((uint32_t)0x400FF100)

/****************************************************************/
/*                 Peripheral declaration                       */
/****************************************************************/
#define DMA                     ((DMA_TypeDef *)     DMA_BASE)
#define DMA_TCD0                ((DMA_TypeDef *)     DMA_TCD0_BASE)
#define DMA_TCD1                ((DMA_TypeDef *)     DMA_TCD1_BASE)
#define DMA_TCD2                ((DMA_TypeDef *)     DMA_TCD2_BASE)
#define DMA_TCD3                ((DMA_TypeDef *)     DMA_TCD3_BASE)
#define DMA_TCD4                ((DMA_TypeDef *)     DMA_TCD4_BASE)
#define DMA_TCD5                ((DMA_TypeDef *)     DMA_TCD5_BASE)
#define DMA_TCD6                ((DMA_TypeDef *)     DMA_TCD6_BASE)
#define DMA_TCD7                ((DMA_TypeDef *)     DMA_TCD7_BASE)
#define DMA_TCD8                ((DMA_TypeDef *)     DMA_TCD8_BASE)
#define DMA_TCD9                ((DMA_TypeDef *)     DMA_TCD9_BASE)
#define DMA_TCD10               ((DMA_TypeDef *)     DMA_TCD10_BASE)
#define DMA_TCD11               ((DMA_TypeDef *)     DMA_TCD11_BASE)
#define DMA_TCD12               ((DMA_TypeDef *)     DMA_TCD12_BASE)
#define DMA_TCD13               ((DMA_TypeDef *)     DMA_TCD13_BASE)
#define DMA_TCD14               ((DMA_TypeDef *)     DMA_TCD14_BASE)
#define DMA_TCD15               ((DMA_TypeDef *)     DMA_TCD15_BASE)
#define DMAMUX                  ((DMAMUX_TypeDef *)  DMAMUX_BASE)

#define TPM0                    ((TPM_TypeDef *)     TPM0_BASE)
#define TPM1                    ((TPM_TypeDef *)     TPM1_BASE)
#define TPM2                    ((TPM_TypeDef *)     TPM2_BASE)
#define ADC0                    ((ADC_TypeDef *)     ADC0_BASE)
#define LPTMR0                  ((LPTMR_TypeDef *)   LPTMR0_BASE)
#define TSI0                    ((TSI_TypeDef *)     TSI0_BASE)
#define SIM                     ((SIM_TypeDef  *)    SIM_BASE)
#define LLWU                    ((LLWU_TypeDef  *)   LLWU_BASE)
#define PORTA                   ((PORT_TypeDef  *)   PORTA_BASE)
#define PORTB                   ((PORT_TypeDef  *)   PORTB_BASE)
#define PORTC                   ((PORT_TypeDef  *)   PORTC_BASE)
#define PORTD                   ((PORT_TypeDef  *)   PORTD_BASE)
#define PORTE                   ((PORT_TypeDef  *)   PORTE_BASE)
#define WDOG                    ((WDOG_TypeDef  *)   WDOG_BASE)
#define USBOTG                  ((USBOTG_TypeDef *)  USBOTG_BASE)
#define MCG                     ((MCG_TypeDef  *)    MCG_BASE)
#define OSC                     ((OSC_TypeDef  *)    OSC0_BASE)

#define PMC                     ((PMC_TypeDef *)    PMC_BASE)
#define RTC                     ((RTC_TypeDef *)    RTC_BASE)
#define SMC                     ((SMC_TypeDef *)    SMC_BASE)

#define SPI0                    ((SPI_TypeDef *)     SPI0_BASE)
#define I2C0                    ((I2C_TypeDef *)     I2C0_BASE)
#define UART0                   ((UART_TypeDef *)    UART0_BASE)
#define UART1                   ((UART_TypeDef *)    UART1_BASE)
#define UART2                   ((UART_TypeDef *)    UART2_BASE)
#define GPIOA                   ((GPIO_TypeDef  *)   GPIOA_BASE)
#define GPIOB                   ((GPIO_TypeDef  *)   GPIOB_BASE)
#define GPIOC                   ((GPIO_TypeDef  *)   GPIOC_BASE)
#define GPIOD                   ((GPIO_TypeDef  *)   GPIOD_BASE)
#define GPIOE                   ((GPIO_TypeDef  *)   GPIOE_BASE)

/****************************************************************/
/*           Peripheral Registers Bits Definition               */
/****************************************************************/

/****************************************************************/
/*                                                              */
/*             System Integration Module (SIM)                  */
/*                                                              */
/****************************************************************/
/*********  Bits definition for SIM_SOPT1 register  *************/
#define SIM_SOPT1_USBREGEN           ((uint32_t)0x80000000)    /*!< USB voltage regulator enable */
#define SIM_SOPT1_USBSSTBY           ((uint32_t)0x40000000)    /*!< USB voltage regulator in standby mode during Stop, VLPS, LLS and VLLS modes */
#define SIM_SOPT1_USBVSTBY           ((uint32_t)0x20000000)    /*!< USB voltage regulator in standby mode during VLPR and VLPW modes */
#define SIM_SOPT1_OSC32KSEL_SHIFT    18                        /*!< 32K oscillator clock select (shift) */
#define SIM_SOPT1_OSC32KSEL_MASK     ((uint32_t)((uint32_t)0x3 << SIM_SOPT1_OSC32KSEL_SHIFT))                              /*!< 32K oscillator clock select (mask) */
#define SIM_SOPT1_OSC32KSEL(x)       ((uint32_t)(((uint32_t)(x) << SIM_SOPT1_OSC32KSEL_SHIFT) & SIM_SOPT1_OSC32KSEL_MASK))  /*!< 32K oscillator clock select */
#define SIM_SOPT1_RAMSIZE_SHIFT      12
#define SIM_SOPT1_RAMSIZE_MASK       ((uint32_t)((uint32_t)0xf << SIM_SOPT1_RAMSIZE_SHIFT))
#define SIM_SOPT1_RAMSIZE(x)         ((uint32_t)(((uint32_t)(x) << SIM_SOPT1_RAMSIZE_SHIFT) & SIM_SOPT1_RAMSIZE_MASK))

/*******  Bits definition for SIM_SOPT1CFG register  ************/
#define SIM_SOPT1CFG_USSWE           ((uint32_t)0x04000000)    /*!< USB voltage regulator stop standby write enable */
#define SIM_SOPT1CFG_UVSWE           ((uint32_t)0x02000000)    /*!< USB voltage regulator VLP standby write enable */
#define SIM_SOPT1CFG_URWE            ((uint32_t)0x01000000)    /*!< USB voltage regulator voltage regulator write enable */

/*******  Bits definition for SIM_SOPT2 register  ************/
#define SIM_SOPT2_USBSRC             ((uint32_t)0x00040000)    /*!< USB clock source select */
#define SIM_SOPT2_PLLFLLSEL          ((uint32_t)0x00010000)    /*!< PLL/FLL clock select */
#define SIM_SOPT2_TRACECLKSEL        ((uint32_t)0x00001000)
#define SIM_SOPT2_PTD7PAD            ((uint32_t)0x00000800)
#define SIM_SOPT2_CLKOUTSEL_SHIFT    5
#define SIM_SOPT2_CLKOUTSEL_MASK     ((uint32_t)((uint32_t)0x7 << SIM_SOPT2_CLKOUTSEL_SHIFT))
#define SIM_SOPT2_CLKOUTSEL(x)       ((uint32_t)(((uint32_t)(x) << SIM_SOPT2_CLKOUTSEL_SHIFT) & SIM_SOPT2_CLKOUTSEL_MASK))
#define SIM_SOPT2_RTCCLKOUTSEL       ((uint32_t)0x00000010)    /*!< RTC clock out select */

/*******  Bits definition for SIM_SCGC4 register  ************/
#define SIM_SCGC4_VREF               ((uint32_t)0x00100000)    /*!< VREF Clock Gate Control */
#define SIM_SCGC4_CMP                ((uint32_t)0x00080000)    /*!< Comparator Clock Gate Control */
#define SIM_SCGC4_USBOTG             ((uint32_t)0x00040000)    /*!< USB Clock Gate Control */
#define SIM_SCGC4_UART2              ((uint32_t)0x00001000)    /*!< UART2 Clock Gate Control */
#define SIM_SCGC4_UART1              ((uint32_t)0x00000800)    /*!< UART1 Clock Gate Control */
#define SIM_SCGC4_UART0              ((uint32_t)0x00000400)    /*!< UART0 Clock Gate Control */
#define SIM_SCGC4_I2C0               ((uint32_t)0x00000040)    /*!< I2C0 Clock Gate Control */
#define SIM_SCGC4_CMT                ((uint32_t)0x00000004)    /*!< CMT Clock Gate Control */
#define SIM_SCGC4_EMW                ((uint32_t)0x00000002)    /*!< EWM Clock Gate Control */

/*******  Bits definition for SIM_SCGC5 register  ************/
#define SIM_SCGC5_PORTE              ((uint32_t)0x00002000)    /*!< Port E Clock Gate Control */
#define SIM_SCGC5_PORTD              ((uint32_t)0x00001000)    /*!< Port D Clock Gate Control */
#define SIM_SCGC5_PORTC              ((uint32_t)0x00000800)    /*!< Port C Clock Gate Control */
#define SIM_SCGC5_PORTB              ((uint32_t)0x00000400)    /*!< Port B Clock Gate Control */
#define SIM_SCGC5_PORTA              ((uint32_t)0x00000200)    /*!< Port A Clock Gate Control */
#define SIM_SCGC5_TSI                ((uint32_t)0x00000020)    /*!< TSI Access Control */
#define SIM_SCGC5_LPTIMER            ((uint32_t)0x00000001)    /*!< Low Power Timer Access Control */

/*******  Bits definition for SIM_SCGC6 register  ************/
#define SIM_SCGC6_RTC                ((uint32_t)0x20000000)    /*!< RTC Access Control */
#define SIM_SCGC6_ADC0               ((uint32_t)0x08000000)    /*!< ADC0 Clock Gate Control */
#define SIM_SCGC6_FTM1               ((uint32_t)0x02000000)    /*!< FTM1 Clock Gate Control */
#define SIM_SCGC6_FTM0               ((uint32_t)0x01000000)    /*!< FTM0 Clock Gate Control */
#define SIM_SCGC6_PIT                ((uint32_t)0x00800000)    /*!< PIT Clock Gate Control */
#define SIM_SCGC6_PDB                ((uint32_t)0x00400000)    /*!< PDB Clock Gate Control */
#define SIM_SCGC6_USBDCD             ((uint32_t)0x00200000)    /*!< USB DCD Clock Gate Control */
#define SIM_SCGC6_CRC                ((uint32_t)0x00040000)    /*!< Low Power Timer Access Control */
#define SIM_SCGC6_I2S                ((uint32_t)0x00008000)    /*!< CRC Clock Gate Control */
#define SIM_SCGC6_SPI0               ((uint32_t)0x00001000)    /*!< SPI0 Clock Gate Control */
#define SIM_SCGC6_DMAMUX             ((uint32_t)0x00000002)    /*!< DMA Mux Clock Gate Control */
#define SIM_SCGC6_FTFL               ((uint32_t)0x00000001)    /*!< Flash Memory Clock Gate Control */

/*******  Bits definition for SIM_SCGC6 register  ************/
#define SIM_SCGC7_DMA                ((uint32_t)0x00000002)    /*!< DMA Clock Gate Control */

/******  Bits definition for SIM_CLKDIV1 register  ***********/
#define SIM_CLKDIV1_OUTDIV1_SHIFT    28
#define SIM_CLKDIV1_OUTDIV1_MASK     ((uint32_t)((uint32_t)0xF << SIM_CLKDIV1_OUTDIV1_SHIFT))
#define SIM_CLKDIV1_OUTDIV1(x)       ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV1_OUTDIV1_SHIFT) & SIM_CLKDIV1_OUTDIV1_MASK))
#define SIM_CLKDIV1_OUTDIV2_SHIFT    24
#define SIM_CLKDIV1_OUTDIV2_MASK     ((uint32_t)((uint32_t)0xF << SIM_CLKDIV1_OUTDIV2_SHIFT))
#define SIM_CLKDIV1_OUTDIV2(x)       ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV1_OUTDIV2_SHIFT) & SIM_CLKDIV1_OUTDIV2_MASK))
#define SIM_CLKDIV1_OUTDIV4_SHIFT    16
#define SIM_CLKDIV1_OUTDIV4_MASK     ((uint32_t)((uint32_t)0x7 << SIM_CLKDIV1_OUTDIV4_SHIFT))
#define SIM_CLKDIV1_OUTDIV4(x)       ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV1_OUTDIV4_SHIFT) & SIM_CLKDIV1_OUTDIV4_MASK))

/******  Bits definition for SIM_CLKDIV2 register  ***********/
#define SIM_CLKDIV2_USBDIV_SHIFT     1
#define SIM_CLKDIV2_USBDIV_MASK      ((uint32_t)((uint32_t)0x7 << SIM_CLKDIV2_USBDIV_SHIFT))
#define SIM_CLKDIV2_USBDIV(x)        ((uint32_t)(((uint32_t)(x) << SIM_CLKDIV2_USBDIV_SHIFT) & SIM_CLKDIV2_USBDIV_MASK))
#define SIM_CLKDIV2_USBFRAC          ((uint32_t)0x00000001)

/****************************************************************/
/*                                                              */
/*              Low-Leakage Wakeup Unit (LLWU)                  */
/*                                                              */
/****************************************************************/
/**********  Bits definition for LLWU_PE1 register  *************/
#define LLWU_PE1_WUPE3_SHIFT        6                                                                          /*!< Wakeup Pin Enable for LLWU_P3 (shift) */
#define LLWU_PE1_WUPE3_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE1_WUPE3_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P3 (mask) */
#define LLWU_PE1_WUPE3(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE1_WUPE3_SHIFT) & LLWU_PE1_WUPE3_MASK))  /*!< Wakeup Pin Enable for LLWU_P3 */
#define LLWU_PE1_WUPE2_SHIFT        4                                                                          /*!< Wakeup Pin Enable for LLWU_P2 (shift) */
#define LLWU_PE1_WUPE2_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE1_WUPE2_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P2 (mask) */
#define LLWU_PE1_WUPE2(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE1_WUPE2_SHIFT) & LLWU_PE1_WUPE2_MASK))  /*!< Wakeup Pin Enable for LLWU_P2 */
#define LLWU_PE1_WUPE1_SHIFT        2                                                                          /*!< Wakeup Pin Enable for LLWU_P1 (shift) */
#define LLWU_PE1_WUPE1_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE1_WUPE1_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P1 (mask) */
#define LLWU_PE1_WUPE1(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE1_WUPE1_SHIFT) & LLWU_PE1_WUPE1_MASK))  /*!< Wakeup Pin Enable for LLWU_P1 */
#define LLWU_PE1_WUPE0_SHIFT        0                                                                          /*!< Wakeup Pin Enable for LLWU_P0 (shift) */
#define LLWU_PE1_WUPE0_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE1_WUPE0_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P0 (mask) */
#define LLWU_PE1_WUPE0(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE1_WUPE0_SHIFT) & LLWU_PE1_WUPE0_MASK))  /*!< Wakeup Pin Enable for LLWU_P0 */

/**********  Bits definition for LLWU_PE2 register  *************/
#define LLWU_PE2_WUPE7_SHIFT        6                                                                          /*!< Wakeup Pin Enable for LLWU_P7 (shift) */
#define LLWU_PE2_WUPE7_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE2_WUPE7_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P7 (mask) */
#define LLWU_PE2_WUPE7(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE2_WUPE7_SHIFT) & LLWU_PE2_WUPE7_MASK))  /*!< Wakeup Pin Enable for LLWU_P7 */
#define LLWU_PE2_WUPE6_SHIFT        4                                                                          /*!< Wakeup Pin Enable for LLWU_P6 (shift) */
#define LLWU_PE2_WUPE6_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE2_WUPE6_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P6 (mask) */
#define LLWU_PE2_WUPE6(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE2_WUPE6_SHIFT) & LLWU_PE2_WUPE6_MASK))  /*!< Wakeup Pin Enable for LLWU_P6 */
#define LLWU_PE2_WUPE5_SHIFT        2                                                                          /*!< Wakeup Pin Enable for LLWU_P5 (shift) */
#define LLWU_PE2_WUPE5_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE2_WUPE5_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P5 (mask) */
#define LLWU_PE2_WUPE5(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE2_WUPE5_SHIFT) & LLWU_PE2_WUPE5_MASK))  /*!< Wakeup Pin Enable for LLWU_P5 */
#define LLWU_PE2_WUPE4_SHIFT        0                                                                          /*!< Wakeup Pin Enable for LLWU_P4 (shift) */
#define LLWU_PE2_WUPE4_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE2_WUPE4_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P4 (mask) */
#define LLWU_PE2_WUPE4(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE2_WUPE4_SHIFT) & LLWU_PE2_WUPE4_MASK))  /*!< Wakeup Pin Enable for LLWU_P4 */

/**********  Bits definition for LLWU_PE3 register  *************/
#define LLWU_PE3_WUPE11_SHIFT       6                                                                            /*!< Wakeup Pin Enable for LLWU_P11 (shift) */
#define LLWU_PE3_WUPE11_MASK        ((uint8_t)((uint8_t)0x03 << LLWU_PE3_WUPE11_SHIFT))                          /*!< Wakeup Pin Enable for LLWU_P11 (mask) */
#define LLWU_PE3_WUPE11(x)          ((uint8_t)(((uint8_t)(x) << LLWU_PE3_WUPE11_SHIFT) & LLWU_PE3_WUPE11_MASK))  /*!< Wakeup Pin Enable for LLWU_P11 */
#define LLWU_PE3_WUPE10_SHIFT       4                                                                            /*!< Wakeup Pin Enable for LLWU_P10 (shift) */
#define LLWU_PE3_WUPE10_MASK        ((uint8_t)((uint8_t)0x03 << LLWU_PE3_WUPE10_SHIFT))                          /*!< Wakeup Pin Enable for LLWU_P10 (mask) */
#define LLWU_PE3_WUPE10(x)          ((uint8_t)(((uint8_t)(x) << LLWU_PE3_WUPE10_SHIFT) & LLWU_PE3_WUPE10_MASK))  /*!< Wakeup Pin Enable for LLWU_P10 */
#define LLWU_PE3_WUPE13_SHIFT        2                                                                          /*!< Wakeup Pin Enable for LLWU_P9 (shift) */
#define LLWU_PE3_WUPE13_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE3_WUPE13_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P9 (mask) */
#define LLWU_PE3_WUPE13(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE3_WUPE13_SHIFT) & LLWU_PE3_WUPE13_MASK))  /*!< Wakeup Pin Enable for LLWU_P9 */
#define LLWU_PE3_WUPE8_SHIFT        0                                                                          /*!< Wakeup Pin Enable for LLWU_P8 (shift) */
#define LLWU_PE3_WUPE8_MASK         ((uint8_t)((uint8_t)0x03 << LLWU_PE3_WUPE8_SHIFT))                         /*!< Wakeup Pin Enable for LLWU_P8 (mask) */
#define LLWU_PE3_WUPE8(x)           ((uint8_t)(((uint8_t)(x) << LLWU_PE3_WUPE8_SHIFT) & LLWU_PE3_WUPE8_MASK))  /*!< Wakeup Pin Enable for LLWU_P8 */

/**********  Bits definition for LLWU_PE4 register  *************/
#define LLWU_PE4_WUPE15_SHIFT       6                                                                            /*!< Wakeup Pin Enable for LLWU_P15 (shift) */
#define LLWU_PE4_WUPE15_MASK        ((uint8_t)((uint8_t)0x03 << LLWU_PE4_WUPE15_SHIFT))                          /*!< Wakeup Pin Enable for LLWU_P15 (mask) */
#define LLWU_PE4_WUPE15(x)          ((uint8_t)(((uint8_t)(x) << LLWU_PE4_WUPE15_SHIFT) & LLWU_PE4_WUPE15_MASK))  /*!< Wakeup Pin Enable for LLWU_P15 */
#define LLWU_PE4_WUPE14_SHIFT       4                                                                            /*!< Wakeup Pin Enable for LLWU_P14 (shift) */
#define LLWU_PE4_WUPE14_MASK        ((uint8_t)((uint8_t)0x03 << LLWU_PE4_WUPE14_SHIFT))                          /*!< Wakeup Pin Enable for LLWU_P14 (mask) */
#define LLWU_PE4_WUPE14(x)          ((uint8_t)(((uint8_t)(x) << LLWU_PE4_WUPE14_SHIFT) & LLWU_PE4_WUPE14_MASK))  /*!< Wakeup Pin Enable for LLWU_P14 */
#define LLWU_PE4_WUPE13_SHIFT       2                                                                            /*!< Wakeup Pin Enable for LLWU_P13 (shift) */
#define LLWU_PE4_WUPE13_MASK        ((uint8_t)((uint8_t)0x03 << LLWU_PE4_WUPE13_SHIFT))                          /*!< Wakeup Pin Enable for LLWU_P13 (mask) */
#define LLWU_PE4_WUPE13(x)          ((uint8_t)(((uint8_t)(x) << LLWU_PE4_WUPE13_SHIFT) & LLWU_PE4_WUPE13_MASK))  /*!< Wakeup Pin Enable for LLWU_P13 */
#define LLWU_PE4_WUPE12_SHIFT       0                                                                            /*!< Wakeup Pin Enable for LLWU_P12 (shift) */
#define LLWU_PE4_WUPE12_MASK        ((uint8_t)((uint8_t)0x03 << LLWU_PE4_WUPE12_SHIFT))                          /*!< Wakeup Pin Enable for LLWU_P12 (mask) */
#define LLWU_PE4_WUPE12(x)          ((uint8_t)(((uint8_t)(x) << LLWU_PE4_WUPE12_SHIFT) & LLWU_PE4_WUPE12_MASK))  /*!< Wakeup Pin Enable for LLWU_P12 */

/**********  Bits definition for LLWU_ME register  *************/
#define LLWU_ME_WUME7               ((uint8_t)((uint8_t)1 << 7))    /*!< Wakeup Module Enable for Module 7 */
#define LLWU_ME_WUME6               ((uint8_t)((uint8_t)1 << 6))    /*!< Wakeup Module Enable for Module 6 */
#define LLWU_ME_WUME5               ((uint8_t)((uint8_t)1 << 5))    /*!< Wakeup Module Enable for Module 5 */
#define LLWU_ME_WUME4               ((uint8_t)((uint8_t)1 << 4))    /*!< Wakeup Module Enable for Module 4 */
#define LLWU_ME_WUME3               ((uint8_t)((uint8_t)1 << 3))    /*!< Wakeup Module Enable for Module 3 */
#define LLWU_ME_WUME2               ((uint8_t)((uint8_t)1 << 2))    /*!< Wakeup Module Enable for Module 2 */
#define LLWU_ME_WUME1               ((uint8_t)((uint8_t)1 << 1))    /*!< Wakeup Module Enable for Module 1 */
#define LLWU_ME_WUME0               ((uint8_t)((uint8_t)1 << 0))    /*!< Wakeup Module Enable for Module 0 */

/**********  Bits definition for LLWU_F1 register  *************/
#define LLWU_F1_WUF7                ((uint8_t)((uint8_t)1 << 7))    /*!< Wakeup Flag for LLWU_P7 */
#define LLWU_F1_WUF6                ((uint8_t)((uint8_t)1 << 6))    /*!< Wakeup Flag for LLWU_P6 */
#define LLWU_F1_WUF5                ((uint8_t)((uint8_t)1 << 5))    /*!< Wakeup Flag for LLWU_P5 */
#define LLWU_F1_WUF4                ((uint8_t)((uint8_t)1 << 4))    /*!< Wakeup Flag for LLWU_P4 */
#define LLWU_F1_WUF3                ((uint8_t)((uint8_t)1 << 3))    /*!< Wakeup Flag for LLWU_P3 */
#define LLWU_F1_WUF2                ((uint8_t)((uint8_t)1 << 2))    /*!< Wakeup Flag for LLWU_P2 */
#define LLWU_F1_WUF1                ((uint8_t)((uint8_t)1 << 1))    /*!< Wakeup Flag for LLWU_P1 */
#define LLWU_F1_WUF0                ((uint8_t)((uint8_t)1 << 0))    /*!< Wakeup Flag for LLWU_P0 */

/**********  Bits definition for LLWU_F2 register  *************/
#define LLWU_F2_WUF15               ((uint8_t)((uint8_t)1 << 7))    /*!< Wakeup Flag for LLWU_P15 */
#define LLWU_F2_WUF14               ((uint8_t)((uint8_t)1 << 6))    /*!< Wakeup Flag for LLWU_P14 */
#define LLWU_F2_WUF13               ((uint8_t)((uint8_t)1 << 5))    /*!< Wakeup Flag for LLWU_P13 */
#define LLWU_F2_WUF12               ((uint8_t)((uint8_t)1 << 4))    /*!< Wakeup Flag for LLWU_P12 */
#define LLWU_F2_WUF11               ((uint8_t)((uint8_t)1 << 3))    /*!< Wakeup Flag for LLWU_P11 */
#define LLWU_F2_WUF10               ((uint8_t)((uint8_t)1 << 2))    /*!< Wakeup Flag for LLWU_P10 */
#define LLWU_F2_WUF9                ((uint8_t)((uint8_t)1 << 1))    /*!< Wakeup Flag for LLWU_P9 */
#define LLWU_F2_WUF8                ((uint8_t)((uint8_t)1 << 0))    /*!< Wakeup Flag for LLWU_P8 */

/**********  Bits definition for LLWU_F3 register  *************/
#define LLWU_F3_MWUF7               ((uint8_t)((uint8_t)1 << 7))    /*!< Wakeup Flag for Module 7 */
#define LLWU_F3_MWUF6               ((uint8_t)((uint8_t)1 << 6))    /*!< Wakeup Flag for Module 6 */
#define LLWU_F3_MWUF5               ((uint8_t)((uint8_t)1 << 5))    /*!< Wakeup Flag for Module 5 */
#define LLWU_F3_MWUF4               ((uint8_t)((uint8_t)1 << 4))    /*!< Wakeup Flag for Module 4 */
#define LLWU_F3_MWUF3               ((uint8_t)((uint8_t)1 << 3))    /*!< Wakeup Flag for Module 3 */
#define LLWU_F3_MWUF2               ((uint8_t)((uint8_t)1 << 2))    /*!< Wakeup Flag for Module 2 */
#define LLWU_F3_MWUF1               ((uint8_t)((uint8_t)1 << 1))    /*!< Wakeup Flag for Module 1 */
#define LLWU_F3_MWUF0               ((uint8_t)((uint8_t)1 << 0))    /*!< Wakeup Flag for Module 0 */

/**********  Bits definition for LLWU_FILT1 register  *************/
#define LLWU_FILT1_FILTF            ((uint8_t)((uint8_t)1 << 7))    /*!< Filter Detect Flag */
#define LLWU_FILT1_FILTE_SHIFT      5                                                                              /*!< Digital Filter on External Pin (shift) */
#define LLWU_FILT1_FILTE_MASK       ((uint8_t)((uint8_t)0x03 << LLWU_FILT1_FILTE_SHIFT))                           /*!< Digital Filter on External Pin (mask) */
#define LLWU_FILT1_FILTE(x)         ((uint8_t)(((uint8_t)(x) << LLWU_FILT1_FILTE_SHIFT) & LLWU_FILT1_FILTE_MASK))  /*!< Digital Filter on External Pin */
#define LLWU_FILT1_FILTE_DISABLED   LLWU_FILT1_FILTE(0)  /*!< Filter disabled */
#define LLWU_FILT1_FILTE_POSEDGE    LLWU_FILT1_FILTE(1)  /*!< Filter posedge detect enabled */
#define LLWU_FILT1_FILTE_NEGEDGE    LLWU_FILT1_FILTE(2)  /*!< Filter negedge detect enabled */
#define LLWU_FILT1_FILTE_ANYEDGE    LLWU_FILT1_FILTE(3)  /*!< Filter any edge detect enabled */
#define LLWU_FILT1_FILTSEL_SHIFT    0                                                                                  /*!< Filter Pin Select (LLWU_P0 ... LLWU_P15) (shift) */
#define LLWU_FILT1_FILTSEL_MASK     ((uint8_t)((uint8_t)0x0F << LLWU_FILT1_FILTSEL_SHIFT))                             /*!< Filter Pin Select (LLWU_P0 ... LLWU_P15) (mask) */
#define LLWU_FILT1_FILTSEL(x)       ((uint8_t)(((uint8_t)(x) << LLWU_FILT1_FILTSEL_SHIFT) & LLWU_FILT1_FILTSEL_MASK))  /*!< Filter Pin Select (LLWU_P0 ... LLWU_P15) */

/**********  Bits definition for LLWU_FILT2 register  *************/
#define LLWU_FILT2_FILTF            ((uint8_t)((uint8_t)1 << 7))    /*!< Filter Detect Flag */
#define LLWU_FILT2_FILTE_SHIFT      5                                                                              /*!< Digital Filter on External Pin (shift) */
#define LLWU_FILT2_FILTE_MASK       ((uint8_t)((uint8_t)0x03 << LLWU_FILT2_FILTE_SHIFT))                           /*!< Digital Filter on External Pin (mask) */
#define LLWU_FILT2_FILTE(x)         ((uint8_t)(((uint8_t)(x) << LLWU_FILT2_FILTE_SHIFT) & LLWU_FILT2_FILTE_MASK))  /*!< Digital Filter on External Pin */
#define LLWU_FILT2_FILTE_DISABLED   LLWU_FILT2_FILTE(0)  /*!< Filter disabled */
#define LLWU_FILT2_FILTE_POSEDGE    LLWU_FILT2_FILTE(1)  /*!< Filter posedge detect enabled */
#define LLWU_FILT2_FILTE_NEGEDGE    LLWU_FILT2_FILTE(2)  /*!< Filter negedge detect enabled */
#define LLWU_FILT2_FILTE_ANYEDGE    LLWU_FILT2_FILTE(3)  /*!< Filter any edge detect enabled */
#define LLWU_FILT2_FILTSEL_SHIFT    0                                                                                  /*!< Filter Pin Select (LLWU_P0 ... LLWU_P15) (shift) */
#define LLWU_FILT2_FILTSEL_MASK     ((uint8_t)((uint8_t)0x0F << LLWU_FILT2_FILTSEL_SHIFT))                             /*!< Filter Pin Select (LLWU_P0 ... LLWU_P15) (mask) */
#define LLWU_FILT2_FILTSEL(x)       ((uint8_t)(((uint8_t)(x) << LLWU_FILT2_FILTSEL_SHIFT) & LLWU_FILT2_FILTSEL_MASK))  /*!< Filter Pin Select (LLWU_P0 ... LLWU_P15) */

/****************************************************************/
/*                                                              */
/*           Port Control and interrupts (PORT)                 */
/*                                                              */
/****************************************************************/
/********  Bits definition for PORTx_PCRn register  *************/
#define PORTx_PCRn_ISF               ((uint32_t)0x01000000)    /*!< Interrupt Status Flag */
#define PORTx_PCRn_IRQC_SHIFT        16
#define PORTx_PCRn_IRQC_MASK         ((uint32_t)((uint32_t)0xF << PORTx_PCRn_IRQC_SHIFT))
#define PORTx_PCRn_IRQC(x)           ((uint32_t)(((uint32_t)(x) << PORTx_PCRn_IRQC_SHIFT) & PORTx_PCRn_IRQC_MASK))
#define PORTx_PCRn_LK                ((uint32_t)0x00008000)    /*!< Lock Register */
#define PORTx_PCRn_MUX_SHIFT         8                         /*!< Pin Mux Control (shift) */
#define PORTx_PCRn_MUX_MASK          ((uint32_t)((uint32_t)0x7 << PORTx_PCRn_MUX_SHIFT))   /*!< Pin Mux Control (mask) */
#define PORTx_PCRn_MUX(x)            ((uint32_t)(((uint32_t)(x) << PORTx_PCRn_MUX_SHIFT) & PORTx_PCRn_MUX_MASK))  /*!< Pin Mux Control */
#define PORTx_PCRn_DSE               ((uint32_t)0x00000040)    /*!< Drive Strength Enable */
#define PORTx_PCRn_ODE               ((uint32_t)0x00000020)    /*!< Open Drain Enable */
#define PORTx_PCRn_PFE               ((uint32_t)0x00000010)    /*!< Passive Filter Enable */
#define PORTx_PCRn_SRE               ((uint32_t)0x00000004)    /*!< Slew Rate Enable */
#define PORTx_PCRn_PE                ((uint32_t)0x00000002)    /*!< Pull Enable */
#define PORTx_PCRn_PS                ((uint32_t)0x00000001)    /*!< Pull Select */

/****************************************************************/
/*                                                              */
/*                   Oscillator (OSC)                           */
/*                                                              */
/****************************************************************/
/***********  Bits definition for OSC_CR register  **************/
#define OSC_CR_ERCLKEN               ((uint8_t)0x80)    /*!< External Reference Enable */
#define OSC_CR_EREFSTEN              ((uint8_t)0x20)    /*!< External Reference Stop Enable */
#define OSC_CR_SC2P                  ((uint8_t)0x08)    /*!< Oscillator 2pF Capacitor Load Configure */
#define OSC_CR_SC4P                  ((uint8_t)0x04)    /*!< Oscillator 4pF Capacitor Load Configure */
#define OSC_CR_SC8P                  ((uint8_t)0x02)    /*!< Oscillator 8pF Capacitor Load Configure */
#define OSC_CR_SC16P                 ((uint8_t)0x01)    /*!< Oscillator 16pF Capacitor Load Configure */

/****************************************************************/
/*                                                              */
/*                 Direct Memory Access (DMA)                   */
/*                                                              */
/****************************************************************/

/* CR Bit Fields */
#define DMA_CR_EDBG_MASK                         0x2u
#define DMA_CR_EDBG_SHIFT                        1
#define DMA_CR_ERCA_MASK                         0x4u
#define DMA_CR_ERCA_SHIFT                        2
#define DMA_CR_HOE_MASK                          0x10u
#define DMA_CR_HOE_SHIFT                         4
#define DMA_CR_HALT_MASK                         0x20u
#define DMA_CR_HALT_SHIFT                        5
#define DMA_CR_CLM_MASK                          0x40u
#define DMA_CR_CLM_SHIFT                         6
#define DMA_CR_EMLM_MASK                         0x80u
#define DMA_CR_EMLM_SHIFT                        7
#define DMA_CR_ECX_MASK                          0x10000u
#define DMA_CR_ECX_SHIFT                         16
#define DMA_CR_CX_MASK                           0x20000u
#define DMA_CR_CX_SHIFT                          17
/* ES Bit Fields */
#define DMA_ES_DBE_MASK                          0x1u
#define DMA_ES_DBE_SHIFT                         0
#define DMA_ES_SBE_MASK                          0x2u
#define DMA_ES_SBE_SHIFT                         1
#define DMA_ES_SGE_MASK                          0x4u
#define DMA_ES_SGE_SHIFT                         2
#define DMA_ES_NCE_MASK                          0x8u
#define DMA_ES_NCE_SHIFT                         3
#define DMA_ES_DOE_MASK                          0x10u
#define DMA_ES_DOE_SHIFT                         4
#define DMA_ES_DAE_MASK                          0x20u
#define DMA_ES_DAE_SHIFT                         5
#define DMA_ES_SOE_MASK                          0x40u
#define DMA_ES_SOE_SHIFT                         6
#define DMA_ES_SAE_MASK                          0x80u
#define DMA_ES_SAE_SHIFT                         7
#define DMA_ES_ERRCHN_MASK                       0xF00u
#define DMA_ES_ERRCHN_SHIFT                      8
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ES_ERRCHN_SHIFT))&DMA_ES_ERRCHN_MASK)
#define DMA_ES_CPE_MASK                          0x4000u
#define DMA_ES_CPE_SHIFT                         14
#define DMA_ES_ECX_MASK                          0x10000u
#define DMA_ES_ECX_SHIFT                         16
#define DMA_ES_VLD_MASK                          0x80000000u
#define DMA_ES_VLD_SHIFT                         31
/* ERQ Bit Fields */
#define DMA_ERQ_ERQ0_MASK                        0x1u
#define DMA_ERQ_ERQ0_SHIFT                       0
#define DMA_ERQ_ERQ1_MASK                        0x2u
#define DMA_ERQ_ERQ1_SHIFT                       1
#define DMA_ERQ_ERQ2_MASK                        0x4u
#define DMA_ERQ_ERQ2_SHIFT                       2
#define DMA_ERQ_ERQ3_MASK                        0x8u
#define DMA_ERQ_ERQ3_SHIFT                       3
/* EEI Bit Fields */
#define DMA_EEI_EEI0_MASK                        0x1u
#define DMA_EEI_EEI0_SHIFT                       0
#define DMA_EEI_EEI1_MASK                        0x2u
#define DMA_EEI_EEI1_SHIFT                       1
#define DMA_EEI_EEI2_MASK                        0x4u
#define DMA_EEI_EEI2_SHIFT                       2
#define DMA_EEI_EEI3_MASK                        0x8u
#define DMA_EEI_EEI3_SHIFT                       3
/* CEEI Bit Fields */
#define DMA_CEEI_CEEI_MASK                       0xFu
#define DMA_CEEI_CEEI_SHIFT                      0
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CEEI_CEEI_SHIFT))&DMA_CEEI_CEEI_MASK)
#define DMA_CEEI_CAEE_MASK                       0x40u
#define DMA_CEEI_CAEE_SHIFT                      6
#define DMA_CEEI_NOP_MASK                        0x80u
#define DMA_CEEI_NOP_SHIFT                       7
/* SEEI Bit Fields */
#define DMA_SEEI_SEEI_MASK                       0xFu
#define DMA_SEEI_SEEI_SHIFT                      0
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SEEI_SEEI_SHIFT))&DMA_SEEI_SEEI_MASK)
#define DMA_SEEI_SAEE_MASK                       0x40u
#define DMA_SEEI_SAEE_SHIFT                      6
#define DMA_SEEI_NOP_MASK                        0x80u
#define DMA_SEEI_NOP_SHIFT                       7
/* CERQ Bit Fields */
#define DMA_CERQ_CERQ_MASK                       0xFu
#define DMA_CERQ_CERQ_SHIFT                      0
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERQ_CERQ_SHIFT))&DMA_CERQ_CERQ_MASK)
#define DMA_CERQ_CAER_MASK                       0x40u
#define DMA_CERQ_CAER_SHIFT                      6
#define DMA_CERQ_NOP_MASK                        0x80u
#define DMA_CERQ_NOP_SHIFT                       7
/* SERQ Bit Fields */
#define DMA_SERQ_SERQ_MASK                       0xFu
#define DMA_SERQ_SERQ_SHIFT                      0
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SERQ_SERQ_SHIFT))&DMA_SERQ_SERQ_MASK)
#define DMA_SERQ_SAER_MASK                       0x40u
#define DMA_SERQ_SAER_SHIFT                      6
#define DMA_SERQ_NOP_MASK                        0x80u
#define DMA_SERQ_NOP_SHIFT                       7
/* CDNE Bit Fields */
#define DMA_CDNE_CDNE_MASK                       0xFu
#define DMA_CDNE_CDNE_SHIFT                      0
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CDNE_CDNE_SHIFT))&DMA_CDNE_CDNE_MASK)
#define DMA_CDNE_CADN_MASK                       0x40u
#define DMA_CDNE_CADN_SHIFT                      6
#define DMA_CDNE_NOP_MASK                        0x80u
#define DMA_CDNE_NOP_SHIFT                       7
/* SSRT Bit Fields */
#define DMA_SSRT_SSRT_MASK                       0xFu
#define DMA_SSRT_SSRT_SHIFT                      0
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SSRT_SSRT_SHIFT))&DMA_SSRT_SSRT_MASK)
#define DMA_SSRT_SAST_MASK                       0x40u
#define DMA_SSRT_SAST_SHIFT                      6
#define DMA_SSRT_NOP_MASK                        0x80u
#define DMA_SSRT_NOP_SHIFT                       7
/* CERR Bit Fields */
#define DMA_CERR_CERR_MASK                       0xFu
#define DMA_CERR_CERR_SHIFT                      0
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERR_CERR_SHIFT))&DMA_CERR_CERR_MASK)
#define DMA_CERR_CAEI_MASK                       0x40u
#define DMA_CERR_CAEI_SHIFT                      6
#define DMA_CERR_NOP_MASK                        0x80u
#define DMA_CERR_NOP_SHIFT                       7
/* CINT Bit Fields */
#define DMA_CINT_CINT_MASK                       0xFu
#define DMA_CINT_CINT_SHIFT                      0
#define DMA_CINT_CINT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CINT_CINT_SHIFT))&DMA_CINT_CINT_MASK)
#define DMA_CINT_CAIR_MASK                       0x40u
#define DMA_CINT_CAIR_SHIFT                      6
#define DMA_CINT_NOP_MASK                        0x80u
#define DMA_CINT_NOP_SHIFT                       7
/* INT Bit Fields */
#define DMA_INT_INT0_MASK                        0x1u
#define DMA_INT_INT0_SHIFT                       0
#define DMA_INT_INT1_MASK                        0x2u
#define DMA_INT_INT1_SHIFT                       1
#define DMA_INT_INT2_MASK                        0x4u
#define DMA_INT_INT2_SHIFT                       2
#define DMA_INT_INT3_MASK                        0x8u
#define DMA_INT_INT3_SHIFT                       3
/* ERR Bit Fields */
#define DMA_ERR_ERR0_MASK                        0x1u
#define DMA_ERR_ERR0_SHIFT                       0
#define DMA_ERR_ERR1_MASK                        0x2u
#define DMA_ERR_ERR1_SHIFT                       1
#define DMA_ERR_ERR2_MASK                        0x4u
#define DMA_ERR_ERR2_SHIFT                       2
#define DMA_ERR_ERR3_MASK                        0x8u
#define DMA_ERR_ERR3_SHIFT                       3
/* HRS Bit Fields */
#define DMA_HRS_HRS0_MASK                        0x1u
#define DMA_HRS_HRS0_SHIFT                       0
#define DMA_HRS_HRS1_MASK                        0x2u
#define DMA_HRS_HRS1_SHIFT                       1
#define DMA_HRS_HRS2_MASK                        0x4u
#define DMA_HRS_HRS2_SHIFT                       2
#define DMA_HRS_HRS3_MASK                        0x8u
#define DMA_HRS_HRS3_SHIFT                       3
/* DCHPRI3 Bit Fields */
#define DMA_DCHPRI3_CHPRI_MASK                   0xFu
#define DMA_DCHPRI3_CHPRI_SHIFT                  0
#define DMA_DCHPRI3_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI3_CHPRI_SHIFT))&DMA_DCHPRI3_CHPRI_MASK)
#define DMA_DCHPRI3_DPA_MASK                     0x40u
#define DMA_DCHPRI3_DPA_SHIFT                    6
#define DMA_DCHPRI3_ECP_MASK                     0x80u
#define DMA_DCHPRI3_ECP_SHIFT                    7
/* DCHPRI2 Bit Fields */
#define DMA_DCHPRI2_CHPRI_MASK                   0xFu
#define DMA_DCHPRI2_CHPRI_SHIFT                  0
#define DMA_DCHPRI2_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI2_CHPRI_SHIFT))&DMA_DCHPRI2_CHPRI_MASK)
#define DMA_DCHPRI2_DPA_MASK                     0x40u
#define DMA_DCHPRI2_DPA_SHIFT                    6
#define DMA_DCHPRI2_ECP_MASK                     0x80u
#define DMA_DCHPRI2_ECP_SHIFT                    7
/* DCHPRI1 Bit Fields */
#define DMA_DCHPRI1_CHPRI_MASK                   0xFu
#define DMA_DCHPRI1_CHPRI_SHIFT                  0
#define DMA_DCHPRI1_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI1_CHPRI_SHIFT))&DMA_DCHPRI1_CHPRI_MASK)
#define DMA_DCHPRI1_DPA_MASK                     0x40u
#define DMA_DCHPRI1_DPA_SHIFT                    6
#define DMA_DCHPRI1_ECP_MASK                     0x80u
#define DMA_DCHPRI1_ECP_SHIFT                    7
/* DCHPRI0 Bit Fields */
#define DMA_DCHPRI0_CHPRI_MASK                   0xFu
#define DMA_DCHPRI0_CHPRI_SHIFT                  0
#define DMA_DCHPRI0_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI0_CHPRI_SHIFT))&DMA_DCHPRI0_CHPRI_MASK)
#define DMA_DCHPRI0_DPA_MASK                     0x40u
#define DMA_DCHPRI0_DPA_SHIFT                    6
#define DMA_DCHPRI0_ECP_MASK                     0x80u
#define DMA_DCHPRI0_ECP_SHIFT                    7
/* SADDR Bit Fields */
#define DMA_SADDR_SADDR_MASK                     0xFFFFFFFFu
#define DMA_SADDR_SADDR_SHIFT                    0
#define DMA_SADDR_SADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SADDR_SADDR_SHIFT))&DMA_SADDR_SADDR_MASK)
/* SOFF Bit Fields */
#define DMA_SOFF_SOFF_MASK                       0xFFFFu
#define DMA_SOFF_SOFF_SHIFT                      0
#define DMA_SOFF_SOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_SOFF_SOFF_SHIFT))&DMA_SOFF_SOFF_MASK)
/* ATTR Bit Fields */
#define DMA_ATTR_DSIZE_MASK                      0x7u
#define DMA_ATTR_DSIZE_SHIFT                     0
#define DMA_ATTR_DSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DSIZE_SHIFT))&DMA_ATTR_DSIZE_MASK)
#define DMA_ATTR_DMOD_MASK                       0xF8u
#define DMA_ATTR_DMOD_SHIFT                      3
#define DMA_ATTR_DMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DMOD_SHIFT))&DMA_ATTR_DMOD_MASK)
#define DMA_ATTR_SSIZE_MASK                      0x700u
#define DMA_ATTR_SSIZE_SHIFT                     8
#define DMA_ATTR_SSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SSIZE_SHIFT))&DMA_ATTR_SSIZE_MASK)
#define DMA_ATTR_SMOD_MASK                       0xF800u
#define DMA_ATTR_SMOD_SHIFT                      11
#define DMA_ATTR_SMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SMOD_SHIFT))&DMA_ATTR_SMOD_MASK)
/* NBYTES_MLNO Bit Fields */
#define DMA_NBYTES_MLNO_NBYTES_MASK              0xFFFFFFFFu
#define DMA_NBYTES_MLNO_NBYTES_SHIFT             0
#define DMA_NBYTES_MLNO_NBYTES(x)                (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLNO_NBYTES_SHIFT))&DMA_NBYTES_MLNO_NBYTES_MASK)
/* NBYTES_MLOFFNO Bit Fields */
#define DMA_NBYTES_MLOFFNO_NBYTES_MASK           0x3FFFFFFFu
#define DMA_NBYTES_MLOFFNO_NBYTES_SHIFT          0
#define DMA_NBYTES_MLOFFNO_NBYTES(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFNO_NBYTES_SHIFT))&DMA_NBYTES_MLOFFNO_NBYTES_MASK)
#define DMA_NBYTES_MLOFFNO_DMLOE_MASK            0x40000000u
#define DMA_NBYTES_MLOFFNO_DMLOE_SHIFT           30
#define DMA_NBYTES_MLOFFNO_SMLOE_MASK            0x80000000u
#define DMA_NBYTES_MLOFFNO_SMLOE_SHIFT           31
/* NBYTES_MLOFFYES Bit Fields */
#define DMA_NBYTES_MLOFFYES_NBYTES_MASK          0x3FFu
#define DMA_NBYTES_MLOFFYES_NBYTES_SHIFT         0
#define DMA_NBYTES_MLOFFYES_NBYTES(x)            (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_NBYTES_SHIFT))&DMA_NBYTES_MLOFFYES_NBYTES_MASK)
#define DMA_NBYTES_MLOFFYES_MLOFF_MASK           0x3FFFFC00u
#define DMA_NBYTES_MLOFFYES_MLOFF_SHIFT          10
#define DMA_NBYTES_MLOFFYES_MLOFF(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_MLOFF_SHIFT))&DMA_NBYTES_MLOFFYES_MLOFF_MASK)
#define DMA_NBYTES_MLOFFYES_DMLOE_MASK           0x40000000u
#define DMA_NBYTES_MLOFFYES_DMLOE_SHIFT          30
#define DMA_NBYTES_MLOFFYES_SMLOE_MASK           0x80000000u
#define DMA_NBYTES_MLOFFYES_SMLOE_SHIFT          31
/* SLAST Bit Fields */
#define DMA_SLAST_SLAST_MASK                     0xFFFFFFFFu
#define DMA_SLAST_SLAST_SHIFT                    0
#define DMA_SLAST_SLAST(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SLAST_SLAST_SHIFT))&DMA_SLAST_SLAST_MASK)
/* DADDR Bit Fields */
#define DMA_DADDR_DADDR_MASK                     0xFFFFFFFFu
#define DMA_DADDR_DADDR_SHIFT                    0
#define DMA_DADDR_DADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_DADDR_DADDR_SHIFT))&DMA_DADDR_DADDR_MASK)
/* DOFF Bit Fields */
#define DMA_DOFF_DOFF_MASK                       0xFFFFu
#define DMA_DOFF_DOFF_SHIFT                      0
#define DMA_DOFF_DOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_DOFF_DOFF_SHIFT))&DMA_DOFF_DOFF_MASK)
/* CITER_ELINKNO Bit Fields */
#define DMA_CITER_ELINKNO_CITER_MASK             0x7FFFu
#define DMA_CITER_ELINKNO_CITER_SHIFT            0
#define DMA_CITER_ELINKNO_CITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKNO_CITER_SHIFT))&DMA_CITER_ELINKNO_CITER_MASK)
#define DMA_CITER_ELINKNO_ELINK_MASK             0x8000u
#define DMA_CITER_ELINKNO_ELINK_SHIFT            15
/* CITER_ELINKYES Bit Fields */
#define DMA_CITER_ELINKYES_CITER_MASK            0x1FFu
#define DMA_CITER_ELINKYES_CITER_SHIFT           0
#define DMA_CITER_ELINKYES_CITER(x)              (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_CITER_SHIFT))&DMA_CITER_ELINKYES_CITER_MASK)
#define DMA_CITER_ELINKYES_LINKCH_MASK           0x1E00u
#define DMA_CITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_CITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_LINKCH_SHIFT))&DMA_CITER_ELINKYES_LINKCH_MASK)
#define DMA_CITER_ELINKYES_ELINK_MASK            0x8000u
#define DMA_CITER_ELINKYES_ELINK_SHIFT           15
/* DLAST_SGA Bit Fields */
#define DMA_DLAST_SGA_DLASTSGA_MASK              0xFFFFFFFFu
#define DMA_DLAST_SGA_DLASTSGA_SHIFT             0
#define DMA_DLAST_SGA_DLASTSGA(x)                (((uint32_t)(((uint32_t)(x))<<DMA_DLAST_SGA_DLASTSGA_SHIFT))&DMA_DLAST_SGA_DLASTSGA_MASK)
/* CSR Bit Fields */
#define DMA_CSR_START_MASK                       0x1u
#define DMA_CSR_START_SHIFT                      0
#define DMA_CSR_INTMAJOR_MASK                    0x2u
#define DMA_CSR_INTMAJOR_SHIFT                   1
#define DMA_CSR_INTHALF_MASK                     0x4u
#define DMA_CSR_INTHALF_SHIFT                    2
#define DMA_CSR_DREQ_MASK                        0x8u
#define DMA_CSR_DREQ_SHIFT                       3
#define DMA_CSR_ESG_MASK                         0x10u
#define DMA_CSR_ESG_SHIFT                        4
#define DMA_CSR_MAJORELINK_MASK                  0x20u
#define DMA_CSR_MAJORELINK_SHIFT                 5
#define DMA_CSR_ACTIVE_MASK                      0x40u
#define DMA_CSR_ACTIVE_SHIFT                     6
#define DMA_CSR_DONE_MASK                        0x80u
#define DMA_CSR_DONE_SHIFT                       7
#define DMA_CSR_MAJORLINKCH_MASK                 0xF00u
#define DMA_CSR_MAJORLINKCH_SHIFT                8
#define DMA_CSR_MAJORLINKCH(x)                   (((uint16_t)(((uint16_t)(x))<<DMA_CSR_MAJORLINKCH_SHIFT))&DMA_CSR_MAJORLINKCH_MASK)
#define DMA_CSR_BWC_MASK                         0xC000u
#define DMA_CSR_BWC_SHIFT                        14
#define DMA_CSR_BWC(x)                           (((uint16_t)(((uint16_t)(x))<<DMA_CSR_BWC_SHIFT))&DMA_CSR_BWC_MASK)
/* BITER_ELINKNO Bit Fields */
#define DMA_BITER_ELINKNO_BITER_MASK             0x7FFFu
#define DMA_BITER_ELINKNO_BITER_SHIFT            0
#define DMA_BITER_ELINKNO_BITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKNO_BITER_SHIFT))&DMA_BITER_ELINKNO_BITER_MASK)
#define DMA_BITER_ELINKNO_ELINK_MASK             0x8000u
#define DMA_BITER_ELINKNO_ELINK_SHIFT            15
/* BITER_ELINKYES Bit Fields */
#define DMA_BITER_ELINKYES_BITER_MASK            0x1FFu
#define DMA_BITER_ELINKYES_BITER_SHIFT           0
#define DMA_BITER_ELINKYES_BITER(x)              (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_BITER_SHIFT))&DMA_BITER_ELINKYES_BITER_MASK)
#define DMA_BITER_ELINKYES_LINKCH_MASK           0x1E00u
#define DMA_BITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_BITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_LINKCH_SHIFT))&DMA_BITER_ELINKYES_LINKCH_MASK)
#define DMA_BITER_ELINKYES_ELINK_MASK            0x8000u
#define DMA_BITER_ELINKYES_ELINK_SHIFT           15

/****************************************************************/
/*                                                              */
/*         Direct Memory Access Multiplexer (DMAMUX)            */
/*                                                              */
/****************************************************************/
/********  Bits definition for DMAMUX_CHCFGn register  **********/
#define DMAMUX_CHCFGn_ENBL           ((uint8_t)((uint8_t)1 << 7))  /*!< DMA Channel Enable */
#define DMAMUX_CHCFGn_TRIG           ((uint8_t)((uint8_t)1 << 6))  /*!< DMA Channel Trigger Enable */
#define DMAMUX_CHCFGn_SOURCE_SHIFT   0                                                                                      /*!< DMA Channel Source (Slot) (shift) */
#define DMAMUX_CHCFGn_SOURCE_MASK    ((uint8_t)((uint8_t)0x3F << DMAMUX_CHCFGn_SOURCE_SHIFT))                               /*!< DMA Channel Source (Slot) (mask) */
#define DMAMUX_CHCFGn_SOURCE(x)      ((uint8_t)(((uint8_t)(x) << DMAMUX_CHCFGn_SOURCE_SHIFT) & DMAMUX_CHCFGn_SOURCE_MASK))  /*!< DMA Channel Source (Slot) */

/****************************************************************/
/*                                                              */
/*              Analog-to-Digital Converter (ADC)               */
/*                                                              */
/****************************************************************/
/***********  Bits definition for ADCx_SC1n register  ***********/
#define ADCx_SC1n_COCO          ((uint32_t)((uint32_t)1 << 7))  /*!< Conversion Complete Flag */
#define ADCx_SC1n_AIEN          ((uint32_t)((uint32_t)1 << 6))  /*!< Interrupt Enable */
#define ADCx_SC1n_DIFF          ((uint32_t)((uint32_t)1 << 5))  /*!< Differential Mode Enable */
#define ADCx_SC1n_ADCH_SHIFT    0                                                                            /*!< Input channel select (shift) */
#define ADCx_SC1n_ADCH_MASK     ((uint32_t)((uint32_t)0x1F << ADCx_SC1n_ADCH_SHIFT))                         /*!< Input channel select (mask) */
#define ADCx_SC1n_ADCH(x)       ((uint32_t)(((uint32_t)(x) << ADCx_SC1n_ADCH_SHIFT) & ADCx_SC1n_ADCH_MASK))  /*!< Input channel select */

/***********  Bits definition for ADCx_CFG1 register  ***********/
#define ADCx_CFG1_ADLPC         ((uint32_t)((uint32_t)1 << 7))  /*!< Low-Power Configuration */
#define ADCx_CFG1_ADIV_SHIFT    5                                                                            /*!< Clock Divide Select (shift) */
#define ADCx_CFG1_ADIV_MASK     ((uint32_t)((uint32_t)0x03 << ADCx_CFG1_ADIV_SHIFT))                         /*!< Clock Divide Select (mask) */
#define ADCx_CFG1_ADIV(x)       ((uint32_t)(((uint32_t)(x) << ADCx_CFG1_ADIV_SHIFT) & ADCx_CFG1_ADIV_MASK))  /*!< Clock Divide Select */
#define ADCx_CFG1_ADLSMP        ((uint32_t)((uint32_t)1 << 4))  /*!< Sample time configuration */
#define ADCx_CFG1_MODE_SHIFT    2                                                                            /*!< Conversion mode (resolution) selection (shift) */
#define ADCx_CFG1_MODE_MASK     ((uint32_t)((uint32_t)0x03 << ADCx_CFG1_MODE_SHIFT))                         /*!< Conversion mode (resolution) selection (mask) */
#define ADCx_CFG1_MODE(x)       ((uint32_t)(((uint32_t)(x) << ADCx_CFG1_MODE_SHIFT) & ADCx_CFG1_MODE_MASK))  /*!< Conversion mode (resolution) selection */
#define ADCx_CFG1_ADICLK_SHIFT  0                                                                                /*!< Input Clock Select (shift) */
#define ADCx_CFG1_ADICLK_MASK   ((uint32_t)((uint32_t)0x03 << ADCx_CFG1_ADICLK_SHIFT))                           /*!< Input Clock Select (mask) */
#define ADCx_CFG1_ADICLK(x)     ((uint32_t)(((uint32_t)(x) << ADCx_CFG1_ADICLK_SHIFT) & ADCx_CFG1_ADICLK_MASK))  /*!< Input Clock Select */

/***********  Bits definition for ADCx_CFG2 register  ***********/
#define ADCx_CFG2_MUXSEL        ((uint32_t)((uint32_t)1 << 4))  /*!< ADC Mux Select */
#define ADCx_CFG2_ADACKEN       ((uint32_t)((uint32_t)1 << 3))  /*!< Asynchronous Clock Output Enable */
#define ADCx_CFG2_ADHSC         ((uint32_t)((uint32_t)1 << 2))  /*!< High-Speed Configuration */
#define ADCx_CFG2_ADLSTS_SHIFT  0                                                                                /*!< Long Sample Time Select (shift) */
#define ADCx_CFG2_ADLSTS_MASK   ((uint32_t)((uint32_t)0x03 << ADCx_CFG2_ADLSTS_SHIFT))                           /*!< Long Sample Time Select (mask) */
#define ADCx_CFG2_ADLSTS(x)     ((uint32_t)(((uint32_t)(x) << ADCx_CFG2_ADLSTS_SHIFT) & ADCx_CFG2_ADLSTS_MASK))  /*!< Long Sample Time Select */

/***********  Bits definition for ADCx_SC2 register  ***********/
#define ADCx_SC2_ADACT          ((uint32_t)((uint32_t)1 << 7))  /*!< Conversion Active */
#define ADCx_SC2_ADTRG          ((uint32_t)((uint32_t)1 << 6))  /*!< Conversion Trigger Select */
#define ADCx_SC2_ACFE           ((uint32_t)((uint32_t)1 << 5))  /*!< Compare Function Enable */
#define ADCx_SC2_ACFGT          ((uint32_t)((uint32_t)1 << 4))  /*!< Compare Function Greater Than Enable */
#define ADCx_SC2_ACREN          ((uint32_t)((uint32_t)1 << 3))  /*!< Compare Function Range Enable */
#define ADCx_SC2_DMAEN          ((uint32_t)((uint32_t)1 << 2))  /*!< DMA Enable */
#define ADCx_SC2_REFSEL_SHIFT   0                                                                              /*!< Voltage Reference Selection (shift) */
#define ADCx_SC2_REFSEL_MASK    ((uint32_t)((uint32_t)0x03 << ADCx_SC2_REFSEL_SHIFT))                          /*!< Voltage Reference Selection (mask) */
#define ADCx_SC2_REFSEL(x)      ((uint32_t)(((uint32_t)(x) << ADCx_SC2_REFSEL_SHIFT) & ADCx_SC2_REFSEL_MASK))  /*!< Voltage Reference Selection */

/***********  Bits definition for ADCx_SC3 register  ***********/
#define ADCx_SC3_CAL            ((uint32_t)((uint32_t)1 << 7))  /*!< Calibration */
#define ADCx_SC3_CALF           ((uint32_t)((uint32_t)1 << 6))  /*!< Calibration Failed Flag */
#define ADCx_SC3_ADCO           ((uint32_t)((uint32_t)1 << 3))  /*!< Continuous Conversion Enable */
#define ADCx_SC3_AVGE           ((uint32_t)((uint32_t)1 << 2))  /*!< Hardware Average Enable */
#define ADCx_SC3_AVGS_SHIFT     0                                                                          /*!< Hardware Average Select (shift) */
#define ADCx_SC3_AVGS_MASK      ((uint32_t)((uint32_t)0x03 << ADCx_SC3_AVGS_SHIFT))                        /*!< Hardware Average Select (mask) */
#define ADCx_SC3_AVGS(x)        ((uint32_t)(((uint32_t)(x) << ADCx_SC3_AVGS_SHIFT) & ADCx_SC3_AVGS_MASK))  /*!< Hardware Average Select */

/****************************************************************/
/*                                                              */
/*                   Low-Power Timer (LPTMR)                    */
/*                                                              */
/****************************************************************/
/**********  Bits definition for LPTMRx_CSR register  ***********/
#define LPTMRx_CSR_TCF              ((uint32_t)((uint32_t)1 << 7))  /*!< Timer Compare Flag */
#define LPTMRx_CSR_TIE              ((uint32_t)((uint32_t)1 << 6))  /*!< Timer Interrupt Enable */
#define LPTMRx_CSR_TPS_SHIFT        4                                                                            /*!< Timer Pin Select (shift) */
#define LPTMRx_CSR_TPS_MASK         ((uint32_t)((uint32_t)0x03 << LPTMRx_CSR_TPS_SHIFT))                         /*!< Timer Pin Select (mask) */
#define LPTMRx_CSR_TPS(x)           ((uint32_t)(((uint32_t)(x) << LPTMRx_CSR_TPS_SHIFT) & LPTMRx_CSR_TPS_MASK))  /*!< Timer Pin Select */
#define LPTMRx_CSR_TPP              ((uint32_t)((uint32_t)1 << 3))  /*!< Timer Pin Polarity */
#define LPTMRx_CSR_TFC              ((uint32_t)((uint32_t)1 << 2))  /*!< Timer Free-Running Counter */
#define LPTMRx_CSR_TMS              ((uint32_t)((uint32_t)1 << 1))  /*!< Timer Mode Select */
#define LPTMRx_CSR_TEN              ((uint32_t)((uint32_t)1 << 0))  /*!< Timer Enable */

/**********  Bits definition for LPTMRx_PSR register  ***********/
#define LPTMRx_PSR_PRESCALE_SHIFT   3                                                                                      /*!< Prescale Value (shift) */
#define LPTMRx_PSR_PRESCALE_MASK    ((uint32_t)((uint32_t)0x0F << LPTMRx_PSR_PRESCALE_SHIFT))                              /*!< Prescale Value (mask) */
#define LPTMRx_PSR_PRESCALE(x)      ((uint32_t)(((uint32_t)(x) << LPTMRx_PSR_PRESCALE_SHIFT) & LPTMRx_PSR_PRESCALE_MASK))  /*!< Prescale Value */
#define LPTMRx_PSR_PBYP             ((uint32_t)((uint32_t)1 << 2))  /*!< Prescaler Bypass */
#define LPTMRx_PSR_PCS_SHIFT        0                                                                            /*!< Prescaler Clock Select (shift) */
#define LPTMRx_PSR_PCS_MASK         ((uint32_t)((uint32_t)0x03 << LPTMRx_PSR_PCS_SHIFT))                         /*!< Prescaler Clock Select (mask) */
#define LPTMRx_PSR_PCS(x)           ((uint32_t)(((uint32_t)(x) << LPTMRx_PSR_PCS_SHIFT) & LPTMRx_PSR_PCS_MASK))  /*!< Prescaler Clock Select */

/**********  Bits definition for LPTMRx_CMR register  ***********/
#define LPTMRx_CMR_COMPARE_SHIFT    0                                                                                    /*!< Compare Value (shift) */
#define LPTMRx_CMR_COMPARE_MASK     ((uint32_t)((uint32_t)0xFFFF << LPTMRx_CMR_COMPARE_SHIFT))                           /*!< Compare Value (mask) */
#define LPTMRx_CMR_COMPARE(x)       ((uint32_t)(((uint32_t)(x) << LPTMRx_CMR_COMPARE_SHIFT) & LPTMRx_CMR_COMPARE_MASK))  /*!< Compare Value */

/**********  Bits definition for LPTMRx_CNR register  ***********/
#define LPTMRx_CNR_COUNTER_SHIFT    0                                                                                    /*!< Counter Value (shift) */
#define LPTMRx_CNR_COUNTER_MASK     ((uint32_t)((uint32_t)0xFFFF << LPTMRx_CNR_COUNTER_SHIFT))                           /*!< Counter Value (mask) */
#define LPTMRx_CNR_COUNTER(x)       ((uint32_t)(((uint32_t)(x) << LPTMRx_CNR_COUNTER_SHIFT) & LPTMRx_CNR_COUNTER_MASK))  /*!< Counter Value */

/****************************************************************/
/*                                                              */
/*                  Touch Sensing Input (TSI)                   */
/*                                                              */
/****************************************************************/
/**********  Bits definition for TSIx_GENCS register  ***********/
#define TSIx_GENCS_OUTRGF           ((uint32_t)((uint32_t)1 << 31))  /*!< Out of Range Flag */
#define TSIx_GENCS_ESOR             ((uint32_t)((uint32_t)1 << 28))  /*!< End-of-scan/Out-of-Range Interrupt Selection */
#define TSIx_GENCS_MODE_SHIFT       24                                                                                 /*!< TSI analog modes setup and status bits (shift) */
#define TSIx_GENCS_MODE_MASK        ((uint32_t)((uint32_t)0x0F << TSIx_GENCS_MODE_SHIFT))                            /*!< TSI analog modes setup and status bits (mask) */
#define TSIx_GENCS_MODE(x)          ((uint32_t)(((uint32_t)(x) << TSIx_GENCS_MODE_SHIFT) & TSIx_GENCS_MODE_MASK))  /*!< TSI analog modes setup and status bits */
#define TSIx_GENCS_REFCHRG_SHIFT    21                                                                                       /*!< Reference oscillator charge/discharge current (shift) */
#define TSIx_GENCS_REFCHRG_MASK     ((uint32_t)((uint32_t)0x07 << TSIx_GENCS_REFCHRG_SHIFT))                               /*!< Reference oscillator charge/discharge current (mask) */
#define TSIx_GENCS_REFCHRG(x)       ((uint32_t)(((uint32_t)(x) << TSIx_GENCS_REFCHRG_SHIFT) & TSIx_GENCS_REFCHRG_MASK))  /*!< Reference oscillator charge/discharge current */
#define TSIx_GENCS_DVOLT_SHIFT      19                                                                                   /*!< Oscillator voltage rails (shift) */
#define TSIx_GENCS_DVOLT_MASK       ((uint32_t)((uint32_t)0x03 << TSIx_GENCS_DVOLT_SHIFT))                             /*!< Oscillator voltage rails (mask) */
#define TSIx_GENCS_DVOLT(x)         ((uint32_t)(((uint32_t)(x) << TSIx_GENCS_DVOLT_SHIFT) & TSIx_GENCS_DVOLT_MASK))  /*!< Oscillator voltage rails */
#define TSIx_GENCS_EXTCHRG_SHIFT    16                                                                                       /*!< Electrode oscillator charge/discharge current (shift) */
#define TSIx_GENCS_EXTCHRG_MASK     ((uint32_t)((uint32_t)0x07 << TSIx_GENCS_EXTCHRG_SHIFT))                               /*!< Electrode oscillator charge/discharge current (mask) */
#define TSIx_GENCS_EXTCHRG(x)       ((uint32_t)(((uint32_t)(x) << TSIx_GENCS_EXTCHRG_SHIFT) & TSIx_GENCS_EXTCHRG_MASK))  /*!< Electrode oscillator charge/discharge current */
#define TSIx_GENCS_PS_SHIFT         13                                                                             /*!< Electrode oscillator prescaler (shift) */
#define TSIx_GENCS_PS_MASK          ((uint32_t)((uint32_t)0x07 << TSIx_GENCS_PS_SHIFT))                          /*!< Electrode oscillator prescaler (mask) */
#define TSIx_GENCS_PS(x)            ((uint32_t)(((uint32_t)(x) << TSIx_GENCS_PS_SHIFT) & TSIx_GENCS_PS_MASK))  /*!< Electrode oscillator prescaler */
#define TSIx_GENCS_NSCN_SHIFT       8                                                                                  /*!< Number of scans per electrode minus 1 (shift) */
#define TSIx_GENCS_NSCN_MASK        ((uint32_t)((uint32_t)0x1F << TSIx_GENCS_NSCN_SHIFT))                            /*!< Number of scans per electrode minus 1 (mask) */
#define TSIx_GENCS_NSCN(x)          ((uint32_t)(((uint32_t)(x) << TSIx_GENCS_NSCN_SHIFT) & TSIx_GENCS_NSCN_MASK))  /*!< Number of scans per electrode minus 1 */
#define TSIx_GENCS_TSIEN            ((uint32_t)((uint32_t)1 << 7))  /*!< TSI Module Enable */
#define TSIx_GENCS_TSIIEN           ((uint32_t)((uint32_t)1 << 6))  /*!< TSI Interrupt Enable */
#define TSIx_GENCS_STPE             ((uint32_t)((uint32_t)1 << 5))  /*!< TSI STOP Enable */
#define TSIx_GENCS_STM              ((uint32_t)((uint32_t)1 << 4))  /*!< Scan Trigger Mode (0=software; 1=hardware) */
#define TSIx_GENCS_SCNIP            ((uint32_t)((uint32_t)1 << 3))  /*!< Scan in Progress Status */
#define TSIx_GENCS_EOSF             ((uint32_t)((uint32_t)1 << 2))  /*!< End of Scan Flag */
#define TSIx_GENCS_CURSW            ((uint32_t)((uint32_t)1 << 1))  /*!< Swap electrode and reference current sources */

/**********  Bits definition for TSIx_DATA register  ************/
#define TSIx_DATA_TSICH_SHIFT       28                                                                             /*!< Specify channel to be measured (shift) */
#define TSIx_DATA_TSICH_MASK        ((uint32_t)((uint32_t)0x0F << TSIx_DATA_TSICH_SHIFT))                          /*!< Specify channel to be measured (mask) */
#define TSIx_DATA_TSICH(x)          ((uint32_t)(((uint32_t)(x) << TSIx_DATA_TSICH_SHIFT) & TSIx_DATA_TSICH_MASK))  /*!< Specify channel to be measured */
#define TSIx_DATA_DMAEN             ((uint32_t)((uint32_t)1 << 23))  /*!< DMA Transfer Enabled */
#define TSIx_DATA_SWTS              ((uint32_t)((uint32_t)1 << 22))  /*!< Software Trigger Start */
#define TSIx_DATA_TSICNT_SHIFT      0                                                                                /*!< TSI Conversion Counter Value (shift) */
#define TSIx_DATA_TSICNT_MASK       ((uint32_t)((uint32_t)0xFFFF << TSIx_DATA_TSICNT_SHIFT))                         /*!< TSI Conversion Counter Value (mask) */
#define TSIx_DATA_TSICNT(x)         ((uint32_t)(((uint32_t)(x) << TSIx_DATA_TSICNT_SHIFT) & TSIx_DATA_TSICNT_MASK))  /*!< TSI Conversion Counter Value */

/**********  Bits definition for TSIx_TSHD register  ************/
#define TSIx_TSHD_THRESH_SHIFT      16                                                                               /*!< TSI Wakeup Channel High-Threshold (shift) */
#define TSIx_TSHD_THRESH_MASK       ((uint32_t)((uint32_t)0xFFFF << TSIx_TSHD_THRESH_SHIFT))                         /*!< TSI Wakeup Channel High-Threshold (mask) */
#define TSIx_TSHD_THRESH(x)         ((uint32_t)(((uint32_t)(x) << TSIx_TSHD_THRESH_SHIFT) & TSIx_TSHD_THRESH_MASK))  /*!< TSI Wakeup Channel High-Threshold */
#define TSIx_TSHD_THRESL_SHIFT      0                                                                                /*!< TSI Wakeup Channel Low-Threshold (shift) */
#define TSIx_TSHD_THRESL_MASK       ((uint32_t)((uint32_t)0xFFFF << TSIx_TSHD_THRESL_SHIFT))                         /*!< TSI Wakeup Channel Low-Threshold (mask) */
#define TSIx_TSHD_THRESL(x)         ((uint32_t)(((uint32_t)(x) << TSIx_TSHD_THRESL_SHIFT) & TSIx_TSHD_THRESL_MASK))  /*!< TSI Wakeup Channel Low-Threshold */

/****************************************************************/
/*                                                              */
/*             Multipurpose Clock Generator (MCG)               */
/*                                                              */
/****************************************************************/
/***********  Bits definition for MCG_C1 register  **************/
#define MCG_C1_CLKS_SHIFT           6                                                           /*!< Clock source select (shift) */
#define MCG_C1_CLKS_MASK            ((uint8_t)((uint8_t)0x3 << MCG_C1_CLKS_SHIFT))             /*!< Clock source select (mask) */
#define MCG_C1_CLKS(x)              ((uint8_t)(((uint8_t)(x) << MCG_C1_CLKS_SHIFT) & MCG_C1_CLKS_MASK))  /*!< Clock source select */
#define MCG_C1_CLKS_FLLPLL          MCG_C1_CLKS(0)  /*!< Select output of FLL or PLL, depending on PLLS control bit */
#define MCG_C1_CLKS_IRCLK           MCG_C1_CLKS(1)  /*!< Select internal reference clock */
#define MCG_C1_CLKS_ERCLK           MCG_C1_CLKS(2)  /*!< Select external reference clock */
#define MCG_C1_FRDIV_SHIFT          3                                                           /*!< FLL External Reference Divider (shift) */
#define MCG_C1_FRDIV_MASK           ((uint8_t)((uint8_t)0x7 << MCG_C1_FRDIV_SHIFT))            /*!< FLL External Reference Divider (mask) */
#define MCG_C1_FRDIV(x)             ((uint8_t)(((uint8_t)(x) << MCG_C1_FRDIV_SHIFT) & MCG_C1_FRDIV_MASK))  /*!< FLL External Reference Divider */
#define MCG_C1_IREFS                ((uint8_t)0x04) /*!< Internal Reference Select (0=ERCLK; 1=slow IRCLK) */
#define MCG_C1_IRCLKEN              ((uint8_t)0x02) /*!< Internal Reference Clock Enable */
#define MCG_C1_IREFSTEN             ((uint8_t)0x01) /*!< Internal Reference Stop Enable */

/***********  Bits definition for MCG_C2 register  **************/
#define MCG_C2_LOCRE0               ((uint8_t)0x80) /*!< Loss of Clock Reset Enable */
#define MCG_C2_RANGE0_SHIFT         4               /*!< Frequency Range Select (shift) */
#define MCG_C2_RANGE0_MASK          ((uint8_t)((uint8_t)0x3 << MCG_C2_RANGE0_SHIFT))  /*!< Frequency Range Select (mask) */
#define MCG_C2_RANGE0(x)            ((uint8_t)(((uint8_t)(x) << MCG_C2_RANGE0_SHIFT) & MCG_C2_RANGE0_MASK))  /*!< Frequency Range Select */
#define MCG_C2_HGO0                 ((uint8_t)0x08) /*!< High Gain Oscillator Select (0=low power; 1=high gain) */
#define MCG_C2_EREFS0               ((uint8_t)0x04) /*!< External Reference Select (0=clock; 1=oscillator) */
#define MCG_C2_LP                   ((uint8_t)0x02) /*!< Low Power Select (1=FLL/PLL disabled in bypass modes) */
#define MCG_C2_IRCS                 ((uint8_t)0x01) /*!< Internal Reference Clock Select (0=slow; 1=fast) */

/***********  Bits definition for MCG_C4 register  **************/
#define MCG_C4_DMX32                ((uint8_t)0x80) /*!< DCO Maximum Frequency with 32.768 kHz Reference */
#define MCG_C4_DRST_DRS_SHIFT       5               /*!< DCO Range Select (shift) */
#define MCG_C4_DRST_DRS_MASK        ((uint8_t)((uint8_t)0x3 << MCG_C4_DRST_DRS_SHIFT)) /*!< DCO Range Select (mask) */
#define MCG_C4_DRST_DRS(x)          ((uint8_t)(((uint8_t)(x) << MCG_C4_DRST_DRS_SHIFT) & MCG_C4_DRST_DRS_MASK))  /*!< DCO Range Select */
#define MCG_C4_FCTRIM_SHIFT         1               /*!< Fast Internal Reference Clock Trim Setting (shift) */
#define MCG_C4_FCTRIM_MASK          ((uint8_t)((uint8_t)0xF << MCG_C4_FCTRIM_SHIFT))   /*!< Fast Internal Reference Clock Trim Setting (mask) */
#define MCG_C4_FCTRIM(x)            ((uint8_t)(((uint8_t)(x) << MCG_C4_FCTRIM_SHIFT) & MCG_C4_FCTRIM_MASK))  /*!< Fast Internal Reference Clock Trim Setting */
#define MCG_C4_SCFTRIM              ((uint8_t)0x01) /*!< Slow Internal Reference Clock Fine Trim */

/***********  Bits definition for MCG_C5 register  **************/
#define MCG_C5_PLLCLKEN0            ((uint8_t)0x40) /*!< PLL Clock Enable */
#define MCG_C5_PLLSTEN0             ((uint8_t)0x20) /*!< PLL Stop Enable */
#define MCG_C5_PRDIV0_MASK          ((uint8_t)0x1F) /*!< PLL External Reference Divider (mask) */
#define MCG_C5_PRDIV0(x)            ((uint8_t)((uint8_t)(x) & MCG_C5_PRDIV0_MASK))  /*!< PLL External Reference Divider */

/***********  Bits definition for MCG_C6 register  **************/
#define MCG_C6_LOLIE0               ((uint8_t)0x80) /*!< Loss of Lock Interrupt Enable */
#define MCG_C6_PLLS                 ((uint8_t)0x40) /*!< PLL Select */
#define MCG_C6_CME0                 ((uint8_t)0x20) /*!< Clock Monitor Enable */
#define MCG_C6_VDIV0_MASK           ((uint8_t)0x1F) /*!< VCO 0 Divider (mask) */
#define MCG_C6_VDIV0(x)             ((uint8_t)((uint8_t)(x) & MCG_C6_VDIV0_MASK))  /*!< VCO 0 Divider */

/************  Bits definition for MCG_S register  **************/
#define MCG_S_LOLS                  ((uint8_t)0x80) /*!< Loss of Lock Status */
#define MCG_S_LOCK0                 ((uint8_t)0x40) /*!< Lock Status */
#define MCG_S_PLLST                 ((uint8_t)0x20) /*!< PLL Select Status */
#define MCG_S_IREFST                ((uint8_t)0x10) /*!< Internal Reference Status */
#define MCG_S_CLKST_SHIFT           2               /*!< Clock Mode Status (shift) */
#define MCG_S_CLKST_MASK            ((uint8_t)((uint8_t)0x3 << MCG_S_CLKST_SHIFT))  /*!< Clock Mode Status (mask) */
#define MCG_S_CLKST(x)              ((uint8_t)(((uint8_t)(x) << MCG_S_CLKST_SHIFT) & MCG_S_CLKST_MASK))  /*!< Clock Mode Status */
#define MCG_S_CLKST_FLL             MCG_S_CLKST(0)   /*!< Output of the FLL is selected */
#define MCG_S_CLKST_IRCLK           MCG_S_CLKST(1)   /*!< Internal reference clock is selected */
#define MCG_S_CLKST_ERCLK           MCG_S_CLKST(2)   /*!< External reference clock is selected */
#define MCG_S_CLKST_PLL             MCG_S_CLKST(3)   /*!< Output of the PLL is selected */
#define MCG_S_OSCINIT0              ((uint8_t)0x02)  /*!< OSC Initialization */
#define MCG_S_IRCST                 ((uint8_t)0x01)  /*!< Internal Reference Clock Status */

/************  Bits definition for MCG_SC register  **************/
#define MCG_SC_ATME                 ((uint8_t)0x80)  /*!< Automatic Trim Machine Enable */
#define MCG_SC_ATMS                 ((uint8_t)0x40)  /*!< Automatic Trim Machine Select */
#define MCG_SC_ATMF                 ((uint8_t)0x20)  /*!< Automatic Trim Machine Fail Flag */
#define MCG_SC_FLTPRSRV             ((uint8_t)0x10)  /*!< FLL Filter Preserve Enable */
#define MCG_SC_FCRDIV_SHIFT         1                /*!< Fast Clock Internal Reference Divider (shift) */
#define MCG_SC_FCRDIV_MASK          ((uint8_t)((uint8_t)0x7 << MCG_SC_FCRDIV_SHIFT))  /*!< Fast Clock Internal Reference Divider (mask) */
#define MCG_SC_FCRDIV(x)            ((uint8_t)(((uint8_t)(x) << MCG_SC_FCRDIV_SHIFT) & MCG_SC_FCRDIV_MASK))  /*!< Fast Clock Internal Reference Divider */
#define MCG_SC_FCRDIV_DIV1          MCG_SC_FCRDIV(0)  /*!< Divide Factor is 1 */
#define MCG_SC_FCRDIV_DIV2          MCG_SC_FCRDIV(1)  /*!< Divide Factor is 2 */
#define MCG_SC_FCRDIV_DIV4          MCG_SC_FCRDIV(2)  /*!< Divide Factor is 4 */
#define MCG_SC_FCRDIV_DIV8          MCG_SC_FCRDIV(3)  /*!< Divide Factor is 8 */
#define MCG_SC_FCRDIV_DIV16         MCG_SC_FCRDIV(4)  /*!< Divide Factor is 16 */
#define MCG_SC_FCRDIV_DIV32         MCG_SC_FCRDIV(5)  /*!< Divide Factor is 32 */
#define MCG_SC_FCRDIV_DIV64         MCG_SC_FCRDIV(6)  /*!< Divide Factor is 64 */
#define MCG_SC_FCRDIV_DIV128        MCG_SC_FCRDIV(7)  /*!< Divide Factor is 128 */
#define MCG_SC_LOCS0                ((uint8_t)0x01)   /*!< OSC0 Loss of Clock Status */

/************  Bits definition for MCG_C7 register  **************/
#define MCG_C7_OSCSEL               ((uint8_t)0x01)   /*!< MCG OSC Clock Select */

/************  Bits definition for MCG_C8 register  **************/
#define MCG_C8_LOCRE1               ((uint8_t)0x80)   /*!< PLL Loss of Clock Reset Enable */
#define MCG_C8_LOLRE                ((uint8_t)0x40)   /*!< PLL Loss of Lock Reset Enable */
#define MCG_C8_CME1                 ((uint8_t)0x20)   /*!< PLL Clock Monitor Enable */
#define MCG_C8_LOCS1                ((uint8_t)0x01)   /*!< RTC Loss of Clock Status */

/****************************************************************/
/*                                                              */
/*             Serial Peripheral Interface (SPI)                */
/*                                                              */
/****************************************************************/

/***********  Bits definition for SPIx_MCR register  *************/
#define SPIx_MCR_MSTR            ((uint32_t)0x80000000)      // Master/Slave Mode Select
#define SPIx_MCR_CONT_SCKE       ((uint32_t)0x40000000)      // Continuous SCK Enable
#define SPIx_MCR_DCONF(n)        (((n) & 3) << 28)           // DSPI Configuration
#define SPIx_MCR_FRZ             ((uint32_t)0x08000000)      // Freeze
#define SPIx_MCR_MTFE            ((uint32_t)0x04000000)      // Modified Timing Format Enable
#define SPIx_MCR_ROOE            ((uint32_t)0x01000000)      // Receive FIFO Overflow Overwrite Enable
#define SPIx_MCR_PCSIS(n)        (((n) & 0x1F) << 16)        // Peripheral Chip Select x Inactive State
#define SPIx_MCR_DOZE            ((uint32_t)0x00008000)      // Doze Enable
#define SPIx_MCR_MDIS            ((uint32_t)0x00004000)      // Module Disable
#define SPIx_MCR_DIS_TXF         ((uint32_t)0x00002000)      // Disable Transmit FIFO
#define SPIx_MCR_DIS_RXF         ((uint32_t)0x00001000)      // Disable Receive FIFO
#define SPIx_MCR_CLR_TXF         ((uint32_t)0x00000800)      // Clear the TX FIFO and counter
#define SPIx_MCR_CLR_RXF         ((uint32_t)0x00000400)      // Clear the RX FIFO and counter
#define SPIx_MCR_SMPL_PT(n)      (((n) & 3) << 8)            // Sample Point
#define SPIx_MCR_HALT            ((uint32_t)0x00000001)      // Halt

/***********  Bits definition for SPIx_TCR register  *************/
#define SPIx_TCR_TCNT(n)         (((n) & 0xffff) << 16)      // DSPI Transfer Count Register

/***********  Bits definition for SPIx_CTARn register  *************/
#define SPIx_CTARn_DBR            ((uint32_t)0x80000000)     // Double Baud Rate
#define SPIx_CTARn_FMSZ(n)        (((n) & 15) << 27)         // Frame Size (+1)
#define SPIx_CTARn_CPOL           ((uint32_t)0x04000000)     // Clock Polarity
#define SPIx_CTARn_CPHA           ((uint32_t)0x02000000)     // Clock Phase
#define SPIx_CTARn_LSBFE          ((uint32_t)0x01000000)     // LSB First
#define SPIx_CTARn_PCSSCK(n)      (((n) & 3) << 22)          // PCS to SCK Delay Prescaler
#define SPIx_CTARn_PASC(n)        (((n) & 3) << 20)          // After SCK Delay Prescaler
#define SPIx_CTARn_PDT(n)         (((n) & 3) << 18)          // Delay after Transfer Prescaler
#define SPIx_CTARn_PBR(n)         (((n) & 3) << 16)          // Baud Rate Prescaler
#define SPIx_CTARn_CSSCK(n)       (((n) & 15) << 12)         // PCS to SCK Delay Scaler
#define SPIx_CTARn_ASC(n)         (((n) & 15) << 8)          // After SCK Delay Scaler
#define SPIx_CTARn_DT(n)          (((n) & 15) << 4)          // Delay After Transfer Scaler
#define SPIx_CTARn_BR(n)          (((n) & 15) << 0)          // Baud Rate Scaler
#define SPIx_CTARn_FMSZ_SHIFT     27
#define SPIx_CTARn_FMSZ_MASK      0x78000000u

/***********  Bits definition for SPIx_CTARn_SLAVE register  *************/
#define SPIx_CTARn_SLAVE_FMSZ(n)  (((n) & 15) << 27)         // Frame Size (+1)
#define SPIx_CTARn_SLAVE_CPOL     ((uint32_t)0x04000000)     // Clock Polarity
#define SPIx_CTARn_SLAVE_CPHA     ((uint32_t)0x02000000)     // Clock Phase

/***********  Bits definition for SPIx_SR register  *************/
#define SPIx_SR_TCF               ((uint32_t)0x80000000)     // Transfer Complete Flag
#define SPIx_SR_TXRXS             ((uint32_t)0x40000000)     // TX and RX Status
#define SPIx_SR_EOQF              ((uint32_t)0x10000000)     // End of Queue Flag
#define SPIx_SR_TFUF              ((uint32_t)0x08000000)     // Transmit FIFO Underflow Flag
#define SPIx_SR_TFFF              ((uint32_t)0x02000000)     // Transmit FIFO Fill Flag
#define SPIx_SR_RFOF              ((uint32_t)0x00080000)     // Receive FIFO Overflow Flag
#define SPIx_SR_RFDF              ((uint32_t)0x00020000)     // Receive FIFO Drain Flag
#define SPIx_SR_TXCTR             (((n) & 15) << 12)         // TX FIFO Counter
#define SPIx_SR_TXNXPTR           (((n) & 15) << 8)          // Transmit Next Pointer
#define SPIx_SR_RXCTR             (((n) & 15) << 4)          // RX FIFO Counter
#define SPIx_SR_POPNXTPTR         ((n) & 15)                 // POP Next Pointer

/***********  Bits definition for SPIx_SR register  *************/
#define SPIx_RSER_TCF_RE         ((uint32_t)0x80000000)      // Transmission Complete Request Enable
#define SPIx_RSER_EOQF_RE        ((uint32_t)0x10000000)      // DSPI Finished Request Request Enable
#define SPIx_RSER_TFUF_RE        ((uint32_t)0x08000000)      // Transmit FIFO Underflow Request Enable
#define SPIx_RSER_TFFF_RE        ((uint32_t)0x02000000)      // Transmit FIFO Fill Request Enable
#define SPIx_RSER_TFFF_DIRS      ((uint32_t)0x01000000)      // Transmit FIFO FIll Dma or Interrupt Request Select
#define SPIx_RSER_RFOF_RE        ((uint32_t)0x00080000)      // Receive FIFO Overflow Request Enable
#define SPIx_RSER_RFDF_RE        ((uint32_t)0x00020000)      // Receive FIFO Drain Request Enable
#define SPIx_RSER_RFDF_DIRS      ((uint32_t)0x00010000)      // Receive FIFO Drain DMA or Interrupt Request Select

/***********  Bits definition for SPIx_PUSHR register  *************/
#define SPIx_PUSHR_CONT          ((uint32_t)0x80000000)      // Continuous Peripheral Chip Select Enable
#define SPIx_PUSHR_CTAS(n)       (((n) & 7) << 28)           // Clock and Transfer Attributes Select
#define SPIx_PUSHR_EOQ           ((uint32_t)0x08000000)      // End Of Queue
#define SPIx_PUSHR_CTCNT         ((uint32_t)0x04000000)      // Clear Transfer Counter
#define SPIx_PUSHR_PCS(n)        (((n) & 31) << 16)          // Peripheral Chip Select
#define SPIx_PUSHR_TXDATA(n)     ((n) & 0xffff)              // Transmit Data

/***********  Bits definition for SPIx_PUSHR_SLAVE register  *************/
#define SPIx_PUSHR_SLAVE_TXDATA(n) (((n) & 0xffff) << 0)     // Transmit Data in slave mode

/***********  Bits definition for SPIx_POPR register  *************/
#define SPIx_POPR_RXDATA(n)      (((n) & 0xffff) << 16)      // Received Data

/***********  Bits definition for SPIx_TXFRn register  *************/
#define SPIx_TXFRn_TXCMD_TXDATA  (((n) & 0xffff) << 16)      // Transmit Command (in master mode)
#define SPIx_TXFRn_TXDATA(n)     (((n) & 0xffff) << 0)       // Transmit Data

/***********  Bits definition for SPIx_RXFRn register  *************/
#define SPIx_RXFRn_RXDATA(n)     (((n) & 0xffff) << 0)       // Receive Data

/****************************************************************/
/*                                                              */
/*             Inter-Integrated Circuit (I2C)                   */
/*                                                              */
/****************************************************************/
/***********  Bits definition for I2Cx_A1 register  *************/
#define I2Cx_A1_AD                   ((uint8_t)0xFE)    /*!< Address [7:1] */

#define I2Cx_A1_AD_SHIT              1

/***********  Bits definition for I2Cx_F register  **************/
#define I2Cx_F_MULT                  ((uint8_t)0xC0)    /*!< Multiplier factor */
#define I2Cx_F_ICR                   ((uint8_t)0x3F)    /*!< Clock rate */

#define I2Cx_F_MULT_SHIFT            5

/***********  Bits definition for I2Cx_C1 register  *************/
#define I2Cx_C1_IICEN                ((uint8_t)0x80)    /*!< I2C Enable */
#define I2Cx_C1_IICIE                ((uint8_t)0x40)    /*!< I2C Interrupt Enable */
#define I2Cx_C1_MST                  ((uint8_t)0x20)    /*!< Master Mode Select */
#define I2Cx_C1_TX                   ((uint8_t)0x10)    /*!< Transmit Mode Select */
#define I2Cx_C1_TXAK                 ((uint8_t)0x08)    /*!< Transmit Acknowledge Enable */
#define I2Cx_C1_RSTA                 ((uint8_t)0x04)    /*!< Repeat START */
#define I2Cx_C1_WUEN                 ((uint8_t)0x02)    /*!< Wakeup Enable */
#define I2Cx_C1_DMAEN                ((uint8_t)0x01)    /*!< DMA Enable */

/***********  Bits definition for I2Cx_S register  **************/
#define I2Cx_S_TCF                   ((uint8_t)0x80)    /*!< Transfer Complete Flag */
#define I2Cx_S_IAAS                  ((uint8_t)0x40)    /*!< Addressed As A Slave */
#define I2Cx_S_BUSY                  ((uint8_t)0x20)    /*!< Bus Busy */
#define I2Cx_S_ARBL                  ((uint8_t)0x10)    /*!< Arbitration Lost */
#define I2Cx_S_RAM                   ((uint8_t)0x08)    /*!< Range Address Match */
#define I2Cx_S_SRW                   ((uint8_t)0x04)    /*!< Slave Read/Write */
#define I2Cx_S_IICIF                 ((uint8_t)0x02)    /*!< Interrupt Flag */
#define I2Cx_S_RXAK                  ((uint8_t)0x01)    /*!< Receive Acknowledge */

/***********  Bits definition for I2Cx_D register  **************/
#define I2Cx_D_DATA                  ((uint8_t)0xFF)    /*!< Data */

/***********  Bits definition for I2Cx_C2 register  *************/
#define I2Cx_C2_GCAEN                ((uint8_t)0x80)    /*!< General Call Address Enable */
#define I2Cx_C2_ADEXT                ((uint8_t)0x40)    /*!< Address Extension */
#define I2Cx_C2_HDRS                 ((uint8_t)0x20)    /*!< High Drive Select */
#define I2Cx_C2_SBRC                 ((uint8_t)0x10)    /*!< Slave Baud Rate Control */
#define I2Cx_C2_RMEN                 ((uint8_t)0x08)    /*!< Range Address Matching Enable */
#define I2Cx_C2_AD_10_8              ((uint8_t)0x03)    /*!< Slave Address [10:8] */

/***********  Bits definition for I2Cx_FLT register  ************/
#define I2Cx_FLT_SHEN                ((uint8_t)0x80)    /*!< Stop Hold Enable */
#define I2Cx_FLT_STOPF               ((uint8_t)0x40)    /*!< I2C Bus Stop Detect Flag */
#define I2Cx_FLT_STOPIE              ((uint8_t)0x20)    /*!< I2C Bus Stop Interrupt Enable */
#define I2Cx_FLT_FLT                 ((uint8_t)0x1F)    /*!< I2C Programmable Filter Factor */

/***********  Bits definition for I2Cx_RA register  *************/
#define I2Cx_RA_RAD                  ((uint8_t)0xFE)    /*!< Range Slave Address */

#define I2Cx_RA_RAD_SHIFT            1

/***********  Bits definition for I2Cx_SMB register  ************/
#define I2Cx_SMB_FACK                ((uint8_t)0x80)    /*!< Fast NACK/ACK Enable */
#define I2Cx_SMB_ALERTEN             ((uint8_t)0x40)    /*!< SMBus Alert Response Address Enable */
#define I2Cx_SMB_SIICAEN             ((uint8_t)0x20)    /*!< Second I2C Address Enable */
#define I2Cx_SMB_TCKSEL              ((uint8_t)0x10)    /*!< Timeout Counter Clock Select */
#define I2Cx_SMB_SLTF                ((uint8_t)0x08)    /*!< SCL Low Timeout Flag */
#define I2Cx_SMB_SHTF1               ((uint8_t)0x04)    /*!< SCL High Timeout Flag 1 */
#define I2Cx_SMB_SHTF2               ((uint8_t)0x02)    /*!< SCL High Timeout Flag 2 */
#define I2Cx_SMB_SHTF2IE             ((uint8_t)0x01)    /*!< SHTF2 Interrupt Enable */

/***********  Bits definition for I2Cx_A2 register  *************/
#define I2Cx_A2_SAD                  ((uint8_t)0xFE)    /*!< SMBus Address */

#define I2Cx_A2_SAD_SHIFT            1

/***********  Bits definition for I2Cx_SLTH register  ***********/
#define I2Cx_SLTH_SSLT               ((uint8_t)0xFF)    /*!< MSB of SCL low timeout value */

/***********  Bits definition for I2Cx_SLTL register  ***********/
#define I2Cx_SLTL_SSLT               ((uint8_t)0xFF)    /*!< LSB of SCL low timeout value */

/****************************************************************/
/*                                                              */
/*     Universal Asynchronous Receiver/Transmitter (UART)       */
/*                                                              */
/****************************************************************/
/*********  Bits definition for UARTx_BDH register  *************/
#define UARTx_BDH_LBKDIE             ((uint8_t)0x80)    /*!< LIN Break Detect Interrupt Enable */
#define UARTx_BDH_RXEDGIE            ((uint8_t)0x40)    /*!< RxD Input Active Edge Interrupt Enable */
#define UARTx_BDH_SBR_MASK           ((uint8_t)0x1F)
#define UARTx_BDH_SBR(x)             ((uint8_t)((uint8_t)(x) & UARTx_BDH_SBR_MASK))  /*!< Baud Rate Modulo Divisor */

/*********  Bits definition for UARTx_BDL register  *************/
#define UARTx_BDL_SBR_MASK           ((uint8_t)0xFF)    /*!< Baud Rate Modulo Divisor */

/*********  Bits definition for UARTx_C1 register  **************/
#define UARTx_C1_LOOPS               ((uint8_t)0x80)    /*!< Loop Mode Select */
#define UARTx_C1_DOZEEN              ((uint8_t)0x40)    /*!< Doze Enable */
#define UARTx_C1_UARTSWAI            ((uint8_t)0x40)    /*!< UART Stops in Wait Mode */
#define UARTx_C1_RSRC                ((uint8_t)0x20)    /*!< Receiver Source Select */
#define UARTx_C1_M                   ((uint8_t)0x10)    /*!< 9-Bit or 8-Bit Mode Select */
#define UARTx_C1_WAKE                ((uint8_t)0x08)    /*!< Receiver Wakeup Method Select */
#define UARTx_C1_ILT                 ((uint8_t)0x04)    /*!< Idle Line Type Select */
#define UARTx_C1_PE                  ((uint8_t)0x02)    /*!< Parity Enable */
#define UARTx_C1_PT                  ((uint8_t)0x01)    /*!< Parity Type */

/*********  Bits definition for UARTx_C2 register  **************/
#define UARTx_C2_TIE                 ((uint8_t)0x80)    /*!< Transmit Interrupt Enable for TDRE */
#define UARTx_C2_TCIE                ((uint8_t)0x40)    /*!< Transmission Complete Interrupt Enable for TC */
#define UARTx_C2_RIE                 ((uint8_t)0x20)    /*!< Receiver Interrupt Enable for RDRF */
#define UARTx_C2_ILIE                ((uint8_t)0x10)    /*!< Idle Line Interrupt Enable for IDLE */
#define UARTx_C2_TE                  ((uint8_t)0x08)    /*!< Transmitter Enable */
#define UARTx_C2_RE                  ((uint8_t)0x04)    /*!< Receiver Enable */
#define UARTx_C2_RWU                 ((uint8_t)0x02)    /*!< Receiver Wakeup Control */
#define UARTx_C2_SBK                 ((uint8_t)0x01)    /*!< Send Break */

/*********  Bits definition for UARTx_S1 register  **************/
#define UARTx_S1_TDRE                ((uint8_t)0x80)    /*!< Transmit Data Register Empty Flag */
#define UARTx_S1_TC                  ((uint8_t)0x40)    /*!< Transmission Complete Flag */
#define UARTx_S1_RDRF                ((uint8_t)0x20)    /*!< Receiver Data Register Full Flag */
#define UARTx_S1_IDLE                ((uint8_t)0x10)    /*!< Idle Line Flag */
#define UARTx_S1_OR                  ((uint8_t)0x08)    /*!< Receiver Overrun Flag */
#define UARTx_S1_NF                  ((uint8_t)0x04)    /*!< Noise Flag */
#define UARTx_S1_FE                  ((uint8_t)0x02)    /*!< Framing Error Flag */
#define UARTx_S1_PF                  ((uint8_t)0x01)    /*!< Parity Error Flag */

/*********  Bits definition for UARTx_S2 register  **************/
#define UARTx_S2_LBKDIF              ((uint8_t)0x80)    /*!< LIN Break Detect Interrupt Flag */
#define UARTx_S2_RXEDGIF             ((uint8_t)0x40)    /*!< UART_RX Pin Active Edge Interrupt Flag */
#define UARTx_S2_MSBF                ((uint8_t)0x20)    /*!< MSB First */
#define UARTx_S2_RXINV               ((uint8_t)0x10)    /*!< Receive Data Inversion */
#define UARTx_S2_RWUID               ((uint8_t)0x08)    /*!< Receive Wake Up Idle Detect */
#define UARTx_S2_BRK13               ((uint8_t)0x04)    /*!< Break Character Generation Length */
#define UARTx_S2_LBKDE               ((uint8_t)0x02)    /*!< LIN Break Detect Enable */
#define UARTx_S2_RAF                 ((uint8_t)0x01)    /*!< Receiver Active Flag */

/*********  Bits definition for UARTx_C3 register  **************/
#define UARTx_C3_R8                  ((uint8_t)0x80)    /*!< Ninth Data Bit for Receiver */
#define UARTx_C3_T8                  ((uint8_t)0x40)    /*!< Ninth Data Bit for Transmitter */
#define UARTx_C3_TXDIR               ((uint8_t)0x20)    /*!< UART_TX Pin Direction in Single-Wire Mode */
#define UARTx_C3_TXINV               ((uint8_t)0x10)    /*!< Transmit Data Inversion */
#define UARTx_C3_ORIE                ((uint8_t)0x08)    /*!< Overrun Interrupt Enable */
#define UARTx_C3_NEIE                ((uint8_t)0x04)    /*!< Noise Error Interrupt Enable */
#define UARTx_C3_FEIE                ((uint8_t)0x02)    /*!< Framing Error Interrupt Enable */
#define UARTx_C3_PEIE                ((uint8_t)0x01)    /*!< Parity Error Interrupt Enable */

/*********  Bits definition for UARTx_D register  ***************/
#define UARTx_D_R7T7                 ((uint8_t)0x80)    /*!< Read receive data buffer 7 or write transmit data buffer 7 */
#define UARTx_D_R6T6                 ((uint8_t)0x40)    /*!< Read receive data buffer 6 or write transmit data buffer 6 */
#define UARTx_D_R5T5                 ((uint8_t)0x20)    /*!< Read receive data buffer 5 or write transmit data buffer 5 */
#define UARTx_D_R4T4                 ((uint8_t)0x10)    /*!< Read receive data buffer 4 or write transmit data buffer 4 */
#define UARTx_D_R3T3                 ((uint8_t)0x08)    /*!< Read receive data buffer 3 or write transmit data buffer 3 */
#define UARTx_D_R2T2                 ((uint8_t)0x04)    /*!< Read receive data buffer 2 or write transmit data buffer 2 */
#define UARTx_D_R1T1                 ((uint8_t)0x02)    /*!< Read receive data buffer 1 or write transmit data buffer 1 */
#define UARTx_D_R0T0                 ((uint8_t)0x01)    /*!< Read receive data buffer 0 or write transmit data buffer 0 */

/*********  Bits definition for UARTx_MA1 register  *************/
#define UARTx_MA1_MA                 ((uint8_t)0xFF)    /*!< Match Address */

/*********  Bits definition for UARTx_MA2 register  *************/
#define UARTx_MA2_MA                 ((uint8_t)0xFF)    /*!< Match Address */

/*********  Bits definition for UARTx_C4 register  **************/
#define UARTx_C4_MAEN1               ((uint8_t)0x80)    /*!< Match Address Mode Enable 1 */
#define UARTx_C4_MAEN2               ((uint8_t)0x40)    /*!< Match Address Mode Enable 2 */
#define UARTx_C4_M10                 ((uint8_t)0x20)    /*!< 10-bit Mode Select */
#define UARTx_C4_BRFA_MASK           ((uint8_t)0x1F)
#define UARTx_C4_BRFA(x)             ((uint8_t)((uint8_t)(x) & UARTx_C4_BRFA_MASK))  /*!< Baud Rate Fine Adjust */

/*********  Bits definition for UARTx_C5 register  **************/
#define UARTx_C5_TDMAE               ((uint8_t)0x80)    /*!< Transmitter DMA Enable */
#define UARTx_C5_RDMAE               ((uint8_t)0x20)    /*!< Receiver Full DMA Enable */
#define UARTx_C5_BOTHEDGE            ((uint8_t)0x02)    /*!< Both Edge Sampling */
#define UARTx_C5_RESYNCDIS           ((uint8_t)0x01)    /*!< Resynchronization Disable */

/*******  Bits definition for UARTx_CFIFO register  ************/
#define UARTx_CFIFO_TXFLUSH          ((uint8_t)0x80)    /*!< Transmit FIFO/Buffer Flush */
#define UARTx_CFIFO_RXFLUSH          ((uint8_t)0x40)    /*!< Receive FIFO/Buffer Flush */
#define UARTx_CFIFO_RXOFE            ((uint8_t)0x04)    /*!< Receive FIFO Overflow Interrupt Enable */
#define UARTx_CFIFO_TXOFE            ((uint8_t)0x02)    /*!< Transmit FIFO Overflow Interrupt Enable */
#define UARTx_CFIFO_RXUFE            ((uint8_t)0x01)    /*!< Receive FIFO Underflow Interrupt Enable */

/*******  Bits definition for UARTx_PFIFO register  ************/
#define UARTx_PFIFO_TXFE             ((uint8_t)0x80)    /*!< Transmit FIFO Enable */
#define UARTx_PFIFO_TXFIFOSIZE_SHIFT 4
#define UARTx_PFIFO_TXFIFOSIZE_MASK  ((uint8_t)((uint8_t)0x7 << UARTx_PFIFO_TXFIFOSIZE_SHIFT))
#define UARTx_PFIFO_TXFIFOSIZE(x)    ((uint8_t)(((uint8_t)(x) << UARTx_PFIFO_TXFIFOSIZE_SHIFT) & UARTx_PFIFO_TXFIFOSIZE_MASK))  /*!< Transmit FIFO Buffer depth */
#define UARTx_PFIFO_RXFE             ((uint8_t)0x08)    /*!< Receive FIFOh */
#define UARTx_PFIFO_RXFIFOSIZE_SHIFT 0
#define UARTx_PFIFO_RXFIFOSIZE_MASK  ((uint8_t)((uint8_t)0x7 << UARTx_PFIFO_RXFIFOSIZE_SHIFT))
#define UARTx_PFIFO_RXFIFOSIZE(x)    ((uint8_t)(((uint8_t)(x) << UARTx_PFIFO_RXFIFOSIZE_SHIFT) & UARTx_PFIFO_RXFIFOSIZE_MASK))  /*!< Receive FIFO Buffer depth */

/****************************************************************/
/*                                                              */
/*                         Watchdog                             */
/*                                                              */
/****************************************************************/
/********  Bits definition for WDOG_STCTRLH register  ***********/
#define WDOG_STCTRLH_DISTESTWDOG     ((uint16_t)0x4000)
#define WDOG_STCTRLH_BYTESEL_1_0     ((uint16_t)0x3000)
#define WDOG_STCTRLH_TESTSEL         ((uint16_t)0x0800)
#define WDOG_STCTRLH_TESTWDOG        ((uint16_t)0x0400)
#define WDOG_STCTRLH_WAITEN          ((uint16_t)0x0080)
#define WDOG_STCTRLH_STOPEN          ((uint16_t)0x0040)
#define WDOG_STCTRLH_DBGEN           ((uint16_t)0x0020)
#define WDOG_STCTRLH_ALLOWUPDATE     ((uint16_t)0x0010)
#define WDOG_STCTRLH_WINEN           ((uint16_t)0x0008)
#define WDOG_STCTRLH_IRQRSTEN        ((uint16_t)0x0004)
#define WDOG_STCTRLH_CLKSRC          ((uint16_t)0x0002)
#define WDOG_STCTRLH_WDOGEN          ((uint16_t)0x0001)

/********  Bits definition for WDOG_STCTRLL register  ***********/
#define WDOG_STCTRLL_INTFLG          ((uint16_t)0x8000)

/*********  Bits definition for WDOG_PRESC register  ************/
#define WDOG_PRESC_PRESCVAL          ((uint16_t)0x0700)

/* LVDSC1 Bit Fields */
#define PMC_LVDSC1_LVDV_MASK          0x3u
#define PMC_LVDSC1_LVDV_SHIFT         0
#define PMC_LVDSC1_LVDV(x)            (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC1_LVDV_SHIFT))&PMC_LVDSC1_LVDV_MASK)
#define PMC_LVDSC1_LVDRE_MASK         0x10u
#define PMC_LVDSC1_LVDRE_SHIFT        4
#define PMC_LVDSC1_LVDIE_MASK         0x20u
#define PMC_LVDSC1_LVDIE_SHIFT        5
#define PMC_LVDSC1_LVDACK_MASK        0x40u
#define PMC_LVDSC1_LVDACK_SHIFT       6
#define PMC_LVDSC1_LVDF_MASK          0x80u
#define PMC_LVDSC1_LVDF_SHIFT         7
/* LVDSC2 Bit Fields */
#define PMC_LVDSC2_LVWV_MASK          0x3u
#define PMC_LVDSC2_LVWV_SHIFT         0
#define PMC_LVDSC2_LVWV(x)            (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC2_LVWV_SHIFT))&PMC_LVDSC2_LVWV_MASK)
#define PMC_LVDSC2_LVWIE_MASK         0x20u
#define PMC_LVDSC2_LVWIE_SHIFT        5
#define PMC_LVDSC2_LVWACK_MASK        0x40u
#define PMC_LVDSC2_LVWACK_SHIFT       6
#define PMC_LVDSC2_LVWF_MASK          0x80u
#define PMC_LVDSC2_LVWF_SHIFT         7
/* REGSC Bit Fields */
#define PMC_REGSC_BGBE_MASK           0x1u
#define PMC_REGSC_BGBE_SHIFT          0
#define PMC_REGSC_REGONS_MASK         0x4u
#define PMC_REGSC_REGONS_SHIFT        2
#define PMC_REGSC_ACKISO_MASK         0x8u
#define PMC_REGSC_ACKISO_SHIFT        3

/* TSR Bit Fields */
#define RTC_TSR_TSR_MASK              0xFFFFFFFFu
#define RTC_TSR_TSR_SHIFT             0
#define RTC_TSR_TSR(x)                (((uint32_t)(((uint32_t)(x))<<RTC_TSR_TSR_SHIFT))&RTC_TSR_TSR_MASK)
/* TPR Bit Fields */
#define RTC_TPR_TPR_MASK              0xFFFFu
#define RTC_TPR_TPR_SHIFT             0
#define RTC_TPR_TPR(x)                (((uint32_t)(((uint32_t)(x))<<RTC_TPR_TPR_SHIFT))&RTC_TPR_TPR_MASK)
/* TAR Bit Fields */
#define RTC_TAR_TAR_MASK              0xFFFFFFFFu
#define RTC_TAR_TAR_SHIFT             0
#define RTC_TAR_TAR(x)                (((uint32_t)(((uint32_t)(x))<<RTC_TAR_TAR_SHIFT))&RTC_TAR_TAR_MASK)
/* TCR Bit Fields */
#define RTC_TCR_TCR_MASK              0xFFu
#define RTC_TCR_TCR_SHIFT             0
#define RTC_TCR_TCR(x)                (((uint32_t)(((uint32_t)(x))<<RTC_TCR_TCR_SHIFT))&RTC_TCR_TCR_MASK)
#define RTC_TCR_CIR_MASK              0xFF00u
#define RTC_TCR_CIR_SHIFT             8
#define RTC_TCR_CIR(x)                (((uint32_t)(((uint32_t)(x))<<RTC_TCR_CIR_SHIFT))&RTC_TCR_CIR_MASK)
#define RTC_TCR_TCV_MASK              0xFF0000u
#define RTC_TCR_TCV_SHIFT             16
#define RTC_TCR_TCV(x)                (((uint32_t)(((uint32_t)(x))<<RTC_TCR_TCV_SHIFT))&RTC_TCR_TCV_MASK)
#define RTC_TCR_CIC_MASK              0xFF000000u
#define RTC_TCR_CIC_SHIFT             24
#define RTC_TCR_CIC(x)                (((uint32_t)(((uint32_t)(x))<<RTC_TCR_CIC_SHIFT))&RTC_TCR_CIC_MASK)
/* CR Bit Fields */
#define RTC_CR_SWR                    0x1u
#define RTC_CR_SWR_SHIFT              0
#define RTC_CR_WPE                    0x2u
#define RTC_CR_WPE_SHIFT              1
#define RTC_CR_SUP                    0x4u
#define RTC_CR_SUP_SHIFT              2
#define RTC_CR_UM                     0x8u
#define RTC_CR_UM_SHIFT               3
#define RTC_CR_OSCE                   0x100u
#define RTC_CR_OSCE_SHIFT             8
#define RTC_CR_CLKO                   0x200u
#define RTC_CR_CLKO_SHIFT             9
#define RTC_CR_SC16P                  0x400u
#define RTC_CR_SC16P_SHIFT            10
#define RTC_CR_SC8P                   0x800u
#define RTC_CR_SC8P_SHIFT             11
#define RTC_CR_SC4P                   0x1000u
#define RTC_CR_SC4P_SHIFT             12
#define RTC_CR_SC2P                   0x2000u
#define RTC_CR_SC2P_SHIFT             13
/* SR Bit Fields */
#define RTC_SR_TIF                    0x1u
#define RTC_SR_TIF_SHIFT              0
#define RTC_SR_TOF                    0x2u
#define RTC_SR_TOF_SHIFT              1
#define RTC_SR_TAF                    0x4u
#define RTC_SR_TAF_SHIFT              2
#define RTC_SR_TCE                    0x10u
#define RTC_SR_TCE_SHIFT              4
/* LR Bit Fields */
#define RTC_LR_TCL                    0x8u
#define RTC_LR_TCL_SHIFT              3
#define RTC_LR_CRL                    0x10u
#define RTC_LR_CRL_SHIFT              4
#define RTC_LR_SRL                    0x20u
#define RTC_LR_SRL_SHIFT              5
#define RTC_LR_LRL                    0x40u
#define RTC_LR_LRL_SHIFT              6
/* IER Bit Fields */
#define RTC_IER_TIIE                  0x1u
#define RTC_IER_TIIE_SHIFT            0
#define RTC_IER_TOIE                  0x2u
#define RTC_IER_TOIE_SHIFT            1
#define RTC_IER_TAIE                  0x4u
#define RTC_IER_TAIE_SHIFT            2
#define RTC_IER_TSIE                  0x10u
#define RTC_IER_TSIE_SHIFT            4
/* WAR Bit Fields */
#define RTC_WAR_TSRW                  0x1u
#define RTC_WAR_TSRW_SHIFT            0
#define RTC_WAR_TPRW                  0x2u
#define RTC_WAR_TPRW_SHIFT            1
#define RTC_WAR_TARW                  0x4u
#define RTC_WAR_TARW_SHIFT            2
#define RTC_WAR_TCRW                  0x8u
#define RTC_WAR_TCRW_SHIFT            3
#define RTC_WAR_CRW                   0x10u
#define RTC_WAR_CRW_SHIFT             4
#define RTC_WAR_SRW                   0x20u
#define RTC_WAR_SRW_SHIFT             5
#define RTC_WAR_LRW                   0x40u
#define RTC_WAR_LRW_SHIFT             6
#define RTC_WAR_IERW                  0x80u
#define RTC_WAR_IERW_SHIFT            7
/* RAR Bit Fields */
#define RTC_RAR_TSRR                  0x1u
#define RTC_RAR_TSRR_SHIFT            0
#define RTC_RAR_TPRR                  0x2u
#define RTC_RAR_TPRR_SHIFT            1
#define RTC_RAR_TARR                  0x4u
#define RTC_RAR_TARR_SHIFT            2
#define RTC_RAR_TCRR                  0x8u
#define RTC_RAR_TCRR_SHIFT            3
#define RTC_RAR_CRR                   0x10u
#define RTC_RAR_CRR_SHIFT             4
#define RTC_RAR_SRR                   0x20u
#define RTC_RAR_SRR_SHIFT             5
#define RTC_RAR_LRR                   0x40u
#define RTC_RAR_LRR_SHIFT             6
#define RTC_RAR_IERR                  0x80u
#define RTC_RAR_IERR_SHIFT            7

/* PMPROT Bit Fields */
#define SMC_PMPROT_AVLLS              0x2u
#define SMC_PMPROT_AVLLS_SHIFT        1
#define SMC_PMPROT_ALLS               0x8u
#define SMC_PMPROT_ALLS_SHIFT         3
#define SMC_PMPROT_AVLP               0x20u
#define SMC_PMPROT_AVLP_SHIFT         5
/* PMCTRL Bit Fields */
#define SMC_PMCTRL_STOPM_MASK         0x7u
#define SMC_PMCTRL_STOPM_SHIFT        0
#define SMC_PMCTRL_STOPM(x)           (((uint8_t)(((uint8_t)(x))<<SMC_PMCTRL_STOPM_SHIFT))&SMC_PMCTRL_STOPM_MASK)
#define SMC_PMCTRL_STOPA              0x8u
#define SMC_PMCTRL_STOPA_SHIFT        3
#define SMC_PMCTRL_RUNM_MASK          0x60u
#define SMC_PMCTRL_RUNM_SHIFT         5
#define SMC_PMCTRL_RUNM(x)            (((uint8_t)(((uint8_t)(x))<<SMC_PMCTRL_RUNM_SHIFT))&SMC_PMCTRL_RUNM_MASK)
#define SMC_PMCTRL_LPWUI              0x80u
#define SMC_PMCTRL_LPWUI_SHIFT        7
/* VLLSCTRL Bit Fields */
#define SMC_VLLSCTRL_VLLSM_MASK       0x7u
#define SMC_VLLSCTRL_VLLSM_SHIFT      0
#define SMC_VLLSCTRL_VLLSM(x)         (((uint8_t)(((uint8_t)(x))<<SMC_VLLSCTRL_VLLSM_SHIFT))&SMC_VLLSCTRL_VLLSM_MASK)
#define SMC_VLLSCTRL_PORPO_MASK       0x20u
#define SMC_VLLSCTRL_PORPO_SHIFT      5
/* PMSTAT Bit Fields */
#define SMC_PMSTAT_PMSTAT_MASK        0x7Fu
#define SMC_PMSTAT_PMSTAT_SHIFT       0
#define SMC_PMSTAT_PMSTAT(x)          (((uint8_t)(((uint8_t)(x))<<SMC_PMSTAT_PMSTAT_SHIFT))&SMC_PMSTAT_PMSTAT_MASK)


#endif
