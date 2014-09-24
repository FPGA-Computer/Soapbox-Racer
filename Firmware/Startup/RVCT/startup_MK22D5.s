;/*****************************************************************************
; * @file:    startup_MK22D5.s
; * @purpose: CMSIS Cortex-M4 Core Device Startup File for the
; *           MK22D5
; * @version: 1.4
; * @date:    2013-1-24
; *
; * Copyright: 1997 - 2013 Freescale Semiconductor, Inc. All Rights Reserved.
;*
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; *****************************************************************************/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

main_stack_size EQU     0x00000400
proc_stack_size EQU     0x00000400
heap_size       EQU     0x00000400
	
                AREA    MSTACK, NOINIT, READWRITE, ALIGN=3
main_stack_mem  SPACE   main_stack_size
                EXPORT  __initial_msp
__initial_msp

                AREA    CSTACK, NOINIT, READWRITE, ALIGN=3
__main_thread_stack_base__
                EXPORT  __main_thread_stack_base__
proc_stack_mem  SPACE   proc_stack_size
                EXPORT  __initial_sp
__initial_sp

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
Heap_Mem        SPACE   heap_size
	
                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
									
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_msp  ; Top of Stack
                DCD     Reset_Handler  ; Reset Handler
                DCD     NMI_Handler  ; NMI Handler
                DCD     HardFault_Handler  ; Hard Fault Handler
                DCD     MemManage_Handler  ; MPU Fault Handler
                DCD     BusFault_Handler  ; Bus Fault Handler
                DCD     UsageFault_Handler  ; Usage Fault Handler
                DCD     0  ; Reserved
                DCD     0  ; Reserved
                DCD     0  ; Reserved
                DCD     0  ; Reserved
                DCD     SVC_Handler  ; SVCall Handler
                DCD     DebugMon_Handler  ; Debug Monitor Handler
                DCD     0  ; Reserved
                DCD     PendSV_Handler  ; PendSV Handler
                DCD     SysTick_Handler  ; SysTick Handler

                ; External Interrupts
                DCD     DMA0_IRQHandler  ; DMA Channel 0 Transfer Complete
                DCD     DMA1_IRQHandler  ; DMA Channel 1 Transfer Complete
                DCD     DMA2_IRQHandler  ; DMA Channel 2 Transfer Complete
                DCD     DMA3_IRQHandler  ; DMA Channel 3 Transfer Complete
                DCD     DMA4_IRQHandler  ; DMA Channel 4 Transfer Complete
                DCD     DMA5_IRQHandler  ; DMA Channel 5 Transfer Complete
                DCD     DMA6_IRQHandler  ; DMA Channel 6 Transfer Complete
                DCD     DMA7_IRQHandler  ; DMA Channel 7 Transfer Complete
                DCD     DMA8_IRQHandler  ; DMA Channel 8 Transfer Complete
                DCD     DMA9_IRQHandler  ; DMA Channel 9 Transfer Complete
                DCD     DMA10_IRQHandler  ; DMA Channel 10 Transfer Complete
                DCD     DMA11_IRQHandler  ; DMA Channel 11 Transfer Complete
                DCD     DMA12_IRQHandler  ; DMA Channel 12 Transfer Complete
                DCD     DMA13_IRQHandler  ; DMA Channel 13 Transfer Complete
                DCD     DMA14_IRQHandler  ; DMA Channel 14 Transfer Complete
                DCD     DMA15_IRQHandler  ; DMA Channel 15 Transfer Complete
                DCD     DMA_Error_IRQHandler  ; DMA Error Interrupt
                DCD     MCM_IRQHandler  ; Normal Interrupt
                DCD     FTFL_IRQHandler  ; FTFL Command complete interrupt
                DCD     Read_Collision_IRQHandler  ; Read Collision Interrupt
                DCD     LVD_LVW_IRQHandler  ; Low Voltage Detect, Low Voltage Warning
                DCD     LLW_IRQHandler  ; Low Leakage Wakeup
                DCD     Watchdog_IRQHandler  ; WDOG Interrupt
                DCD     Reserved39_IRQHandler  ; Reserved Interrupt 39
                DCD     I2C0_IRQHandler  ; I2C0 interrupt
                DCD     I2C1_IRQHandler  ; I2C1 interrupt
                DCD     SPI0_IRQHandler  ; SPI0 Interrupt
                DCD     SPI1_IRQHandler  ; SPI1 Interrupt
                DCD     I2S0_Tx_IRQHandler  ; I2S0 transmit interrupt
                DCD     I2S0_Rx_IRQHandler  ; I2S0 receive interrupt
                DCD     Reserved46_IRQHandler   ; Reserved Interrupt 46
                DCD     UART0_RX_TX_IRQHandler  ; UART0 Receive/Transmit interrupt
                DCD     UART0_ERR_IRQHandler  ; UART0 Error interrupt
                DCD     UART1_RX_TX_IRQHandler  ; UART1 Receive/Transmit interrupt
                DCD     UART1_ERR_IRQHandler  ; UART1 Error interrupt
                DCD     UART2_RX_TX_IRQHandler  ; UART2 Receive/Transmit interrupt
                DCD     UART2_ERR_IRQHandler  ; UART2 Error interrupt
                DCD     UART3_RX_TX_IRQHandler  ; UART3 Receive/Transmit interrupt
                DCD     UART3_ERR_IRQHandler  ; UART3 Error interrupt
                DCD     ADC0_IRQHandler  ; ADC0 interrupt
                DCD     CMP0_IRQHandler  ; CMP0 interrupt
                DCD     CMP1_IRQHandler  ; CMP1 interrupt
                DCD     FTM0_IRQHandler  ; FTM0 fault, overflow and channels interrupt
                DCD     FTM1_IRQHandler  ; FTM1 fault, overflow and channels interrupt
                DCD     FTM2_IRQHandler  ; FTM2 fault, overflow and channels interrupt
                DCD     CMT_IRQHandler  ; CMT interrupt
                DCD     RTC_IRQHandler  ; RTC interrupt
                DCD     RTC_Seconds_IRQHandler  ; RTC seconds interrupt
                DCD     PIT0_IRQHandler  ; PIT timer channel 0 interrupt
                DCD     PIT1_IRQHandler  ; PIT timer channel 1 interrupt
                DCD     PIT2_IRQHandler  ; PIT timer channel 2 interrupt
                DCD     PIT3_IRQHandler  ; PIT timer channel 3 interrupt
                DCD     PDB0_IRQHandler  ; PDB0 Interrupt
                DCD     USB0_IRQHandler  ; USB0 interrupt
                DCD     USBDCD_IRQHandler  ; USBDCD Interrupt
                DCD     Reserved71_IRQHandler  ; Reserved interrupt 71
                DCD     DAC0_IRQHandler  ; DAC0 interrupt
                DCD     MCG_IRQHandler  ; MCG Interrupt
                DCD     LPTimer_IRQHandler  ; LPTimer interrupt
                DCD     PORTA_IRQHandler  ; Port A interrupt
                DCD     PORTB_IRQHandler  ; Port B interrupt
                DCD     PORTC_IRQHandler  ; Port C interrupt
                DCD     PORTD_IRQHandler  ; Port D interrupt
                DCD     PORTE_IRQHandler  ; Port E interrupt
                DCD     SWI_IRQHandler  ; Software interrupt
					
__Vectors_End

__Vectors_Size 	EQU     __Vectors_End - __Vectors

; <h> Flash Configuration
;   <i> 16-byte flash configuration field that stores default protection settings (loaded on reset)
;   <i> and security information that allows the MCU to restrict acces to the FTFL module.
;   <h> Backdoor Comparison Key
;     <o0>  Backdoor Key 0  <0x0-0xFF:2>
;     <o1>  Backdoor Key 1  <0x0-0xFF:2>
;     <o2>  Backdoor Key 2  <0x0-0xFF:2>
;     <o3>  Backdoor Key 3  <0x0-0xFF:2>
;     <o4>  Backdoor Key 4  <0x0-0xFF:2>
;     <o5>  Backdoor Key 5  <0x0-0xFF:2>
;     <o6>  Backdoor Key 6  <0x0-0xFF:2>
;     <o7>  Backdoor Key 7  <0x0-0xFF:2>
BackDoorK0      EQU     0xFF
BackDoorK1      EQU     0xFF
BackDoorK2      EQU     0xFF
BackDoorK3      EQU     0xFF
BackDoorK4      EQU     0xFF
BackDoorK5      EQU     0xFF
BackDoorK6      EQU     0xFF
BackDoorK7      EQU     0xFF
;   </h>
;   <h> Program flash protection bytes (FPROT)
;     <i> Each program flash region can be protected from program and erase operation by setting the associated PROT bit.
;     <i> Each bit protects a 1/32 region of the program flash memory.
;     <h> FPROT0
;       <i> Program flash protection bytes
;       <i> 1/32 - 8/32 region
;       <o.0>   FPROT0.0
;       <o.1>   FPROT0.1
;       <o.2>   FPROT0.2
;       <o.3>   FPROT0.3
;       <o.4>   FPROT0.4
;       <o.5>   FPROT0.5
;       <o.6>   FPROT0.6
;       <o.7>   FPROT0.7
nFPROT0         EQU     0x00
FPROT0          EQU     nFPROT0:EOR:0xFF
;     </h>
;     <h> FPROT1
;       <i> Program Flash Region Protect Register 1
;       <i> 9/32 - 16/32 region
;       <o.0>   FPROT1.0
;       <o.1>   FPROT1.1
;       <o.2>   FPROT1.2
;       <o.3>   FPROT1.3
;       <o.4>   FPROT1.4
;       <o.5>   FPROT1.5
;       <o.6>   FPROT1.6
;       <o.7>   FPROT1.7
nFPROT1         EQU     0x00
FPROT1          EQU     nFPROT1:EOR:0xFF
;     </h>
;     <h> FPROT2
;       <i> Program Flash Region Protect Register 2
;       <i> 17/32 - 24/32 region
;       <o.0>   FPROT2.0
;       <o.1>   FPROT2.1
;       <o.2>   FPROT2.2
;       <o.3>   FPROT2.3
;       <o.4>   FPROT2.4
;       <o.5>   FPROT2.5
;       <o.6>   FPROT2.6
;       <o.7>   FPROT2.7
nFPROT2         EQU     0x00
FPROT2          EQU     nFPROT2:EOR:0xFF
;     </h>
;     <h> FPROT3
;       <i> Program Flash Region Protect Register 3
;       <i> 25/32 - 32/32 region
;       <o.0>   FPROT3.0
;       <o.1>   FPROT3.1
;       <o.2>   FPROT3.2
;       <o.3>   FPROT3.3
;       <o.4>   FPROT3.4
;       <o.5>   FPROT3.5
;       <o.6>   FPROT3.6
;       <o.7>   FPROT3.7
nFPROT3         EQU     0x00
FPROT3          EQU     nFPROT3:EOR:0xFF
;     </h>
;   </h>
;   <h> Data flash protection byte (FDPROT)
;     <i> Each bit protects a 1/8 region of the data flash memory.
;     <i> (Program flash only devices: Reserved)
;     <o.0>   FDPROT.0
;     <o.1>   FDPROT.1
;     <o.2>   FDPROT.2
;     <o.3>   FDPROT.3
;     <o.4>   FDPROT.4
;     <o.5>   FDPROT.5
;     <o.6>   FDPROT.6
;     <o.7>   FDPROT.7
nFDPROT         EQU     0x00
FDPROT          EQU     nFDPROT:EOR:0xFF
;   </h>
;   <h> EEPROM protection byte (FEPROT)
;     <i> FlexNVM devices: Each bit protects a 1/8 region of the EEPROM.
;     <i> (Program flash only devices: Reserved)
;     <o.0>   FEPROT.0
;     <o.1>   FEPROT.1
;     <o.2>   FEPROT.2
;     <o.3>   FEPROT.3
;     <o.4>   FEPROT.4
;     <o.5>   FEPROT.5
;     <o.6>   FEPROT.6
;     <o.7>   FEPROT.7
nFEPROT         EQU     0x00
FEPROT          EQU     nFEPROT:EOR:0xFF
;   </h>
;   <h> Flash nonvolatile option byte (FOPT)
;     <i> Allows the user to customize the operation of the MCU at boot time.
;     <o.0>  LPBOOT
;       <0=> Low-power boot
;       <1=> normal boot
;     <o.1>  EZPORT_DIS
;       <0=> EzPort operation is enabled
;       <1=> EzPort operation is disabled
FOPT            EQU     0xFF
;   </h>
;   <h> Flash security byte (FSEC)
;     <i> WARNING: If SEC field is configured as "MCU security status is secure" and MEEN field is configured as "Mass erase is disabled",
;     <i> MCU's security status cannot be set back to unsecure state since Mass erase via the debugger is blocked !!!
;     <o.0..1> SEC
;       <2=> MCU security status is unsecure
;       <3=> MCU security status is secure
;         <i> Flash Security
;         <i> This bits define the security state of the MCU.
;     <o.2..3> FSLACC
;       <2=> Freescale factory access denied
;       <3=> Freescale factory access granted
;         <i> Freescale Failure Analysis Access Code
;         <i> This bits define the security state of the MCU.
;     <o.4..5> MEEN
;       <2=> Mass erase is disabled
;       <3=> Mass erase is enabled
;         <i> Mass Erase Enable Bits
;         <i> Enables and disables mass erase capability of the FTFL module
;     <o.6..7> KEYEN
;       <2=> Backdoor key access enabled
;       <3=> Backdoor key access disabled
;         <i> Backdoor key Security Enable
;         <i> These bits enable and disable backdoor key access to the FTFL module.
FSEC            EQU     0xFE
;   </h>
; </h>
                IF      :LNOT::DEF:RAM_TARGET
                AREA    |.ARM.__at_0x400|, CODE, READONLY
                DCB     BackDoorK0, BackDoorK1, BackDoorK2, BackDoorK3
                DCB     BackDoorK4, BackDoorK5, BackDoorK6, BackDoorK7
                DCB     FPROT0,     FPROT1,     FPROT2,     FPROT3
                DCB     FSEC,       FOPT,       FEPROT,     FDPROT
                ENDIF

                AREA    |.text|, CODE, READONLY
;/*
; * Reset handler.
; */


CONTROL_MODE_PRIVILEGED     EQU     0
CONTROL_MODE_UNPRIVILEGED   EQU     1
CONTROL_USE_MSP             EQU     0
CONTROL_USE_PSP             EQU     2

                IMPORT  __main

                PRESERVE8
                THUMB
                EXPORT  Reset_Handler					
					
Reset_Handler   PROC
                cpsid   i
                ldr     r0, =__initial_sp
                msr     PSP, r0
                movs    r0, #CONTROL_MODE_PRIVILEGED :OR: CONTROL_USE_PSP
                msr     CONTROL, r0
                isb
                bl      __early_init
                ldr     r0, =__main
                bx      r0
                ENDP

__early_init    PROC
                EXPORT  __early_init            [WEAK]
                bx      lr
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

Default_Handler PROC


					
					        EXPORT  NMI_Handler				[WEAK]                        
NMI_Handler
							EXPORT  HardFault_Handler		[WEAK]
HardFault_Handler 
							EXPORT  MemManage_Handler		[WEAK]
MemManage_Handler
							EXPORT  BusFault_Handler		[WEAK]
BusFault_Handler
							EXPORT  UsageFault_Handler		[WEAK]
UsageFault_Handler
							EXPORT  SVC_Handler				[WEAK]
SVC_Handler
							EXPORT  DebugMon_Handler		[WEAK]
DebugMon_Handler
							EXPORT  PendSV_Handler			[WEAK]
PendSV_Handler
							EXPORT  SysTick_Handler			[WEAK]
SysTick_Handler
							EXPORT  DMA0_IRQHandler			[WEAK]
DMA0_IRQHandler
							EXPORT  DMA1_IRQHandler			[WEAK]
DMA1_IRQHandler
							EXPORT  DMA2_IRQHandler			[WEAK]
DMA2_IRQHandler
							EXPORT  DMA3_IRQHandler			[WEAK]
DMA3_IRQHandler
							EXPORT  DMA4_IRQHandler			[WEAK]
DMA4_IRQHandler
							EXPORT  DMA5_IRQHandler			[WEAK]
DMA5_IRQHandler
							EXPORT  DMA6_IRQHandler			[WEAK]
DMA6_IRQHandler
							EXPORT  DMA7_IRQHandler			[WEAK]
DMA7_IRQHandler
							EXPORT  DMA8_IRQHandler			[WEAK]
DMA8_IRQHandler
							EXPORT  DMA9_IRQHandler			[WEAK]
DMA9_IRQHandler
							EXPORT  DMA10_IRQHandler		[WEAK]
DMA10_IRQHandler
							EXPORT  DMA11_IRQHandler		[WEAK]
DMA11_IRQHandler
							EXPORT  DMA12_IRQHandler		[WEAK]
DMA12_IRQHandler
							EXPORT  DMA13_IRQHandler		[WEAK]
DMA13_IRQHandler
							EXPORT  DMA14_IRQHandler		[WEAK]
DMA14_IRQHandler
							EXPORT  DMA15_IRQHandler		[WEAK]
DMA15_IRQHandler
							EXPORT  DMA_Error_IRQHandler	[WEAK]
DMA_Error_IRQHandler
							EXPORT  MCM_IRQHandler			[WEAK]
MCM_IRQHandler
							EXPORT  FTFL_IRQHandler			[WEAK]
FTFL_IRQHandler
							EXPORT  Read_Collision_IRQHandler     [WEAK]
Read_Collision_IRQHandler
							EXPORT  LVD_LVW_IRQHandler		[WEAK]
LVD_LVW_IRQHandler
							EXPORT  LLW_IRQHandler			[WEAK]
LLW_IRQHandler
							EXPORT  Watchdog_IRQHandler		[WEAK]
Watchdog_IRQHandler
							EXPORT  Reserved39_IRQHandler	[WEAK]
Reserved39_IRQHandler
							EXPORT  I2C0_IRQHandler			[WEAK]
I2C0_IRQHandler
							EXPORT  I2C1_IRQHandler			[WEAK]
I2C1_IRQHandler
							EXPORT  SPI0_IRQHandler			[WEAK]
SPI0_IRQHandler
							EXPORT  SPI1_IRQHandler			[WEAK]
SPI1_IRQHandler
							EXPORT  I2S0_Tx_IRQHandler		[WEAK]
I2S0_Tx_IRQHandler
							EXPORT  I2S0_Rx_IRQHandler		[WEAK]
I2S0_Rx_IRQHandler
							EXPORT  Reserved46_IRQHandler	[WEAK]
Reserved46_IRQHandler
							EXPORT  UART0_RX_TX_IRQHandler	[WEAK]
UART0_RX_TX_IRQHandler
							EXPORT  UART0_ERR_IRQHandler	[WEAK]
UART0_ERR_IRQHandler
							EXPORT  UART1_RX_TX_IRQHandler	[WEAK]
UART1_RX_TX_IRQHandler
							EXPORT  UART1_ERR_IRQHandler	[WEAK]
UART1_ERR_IRQHandler
							EXPORT  UART2_RX_TX_IRQHandler	[WEAK]
UART2_RX_TX_IRQHandler
							EXPORT  UART2_ERR_IRQHandler	[WEAK]
UART2_ERR_IRQHandler
							EXPORT  UART3_RX_TX_IRQHandler	[WEAK]
UART3_RX_TX_IRQHandler
							EXPORT  UART3_ERR_IRQHandler	[WEAK]
UART3_ERR_IRQHandler
							EXPORT  ADC0_IRQHandler			[WEAK]
ADC0_IRQHandler
							EXPORT  CMP0_IRQHandler			[WEAK]
CMP0_IRQHandler
							EXPORT  CMP1_IRQHandler			[WEAK]
CMP1_IRQHandler

							EXPORT  FTM0_IRQHandler			[WEAK]
FTM0_IRQHandler

							EXPORT  FTM1_IRQHandler			[WEAK]
FTM1_IRQHandler
							EXPORT  FTM2_IRQHandler			[WEAK]
FTM2_IRQHandler
							EXPORT  CMT_IRQHandler			[WEAK]
CMT_IRQHandler
							EXPORT  RTC_IRQHandler			[WEAK]
RTC_IRQHandler
							EXPORT  RTC_Seconds_IRQHandler	[WEAK]
RTC_Seconds_IRQHandler
							EXPORT  PIT0_IRQHandler			[WEAK]
PIT0_IRQHandler
							EXPORT  PIT1_IRQHandler			[WEAK]
PIT1_IRQHandler
							EXPORT  PIT2_IRQHandler			[WEAK]
PIT2_IRQHandler
							EXPORT  PIT3_IRQHandler			[WEAK]
PIT3_IRQHandler
							EXPORT  PDB0_IRQHandler			[WEAK]
PDB0_IRQHandler
							EXPORT  USB0_IRQHandler			[WEAK]
USB0_IRQHandler
							EXPORT  USBDCD_IRQHandler		[WEAK]
USBDCD_IRQHandler
							EXPORT  Reserved71_IRQHandler	[WEAK]
Reserved71_IRQHandler
							EXPORT  DAC0_IRQHandler			[WEAK]
DAC0_IRQHandler
							EXPORT  MCG_IRQHandler			[WEAK]
MCG_IRQHandler
							EXPORT  LPTimer_IRQHandler		[WEAK]
LPTimer_IRQHandler
							EXPORT  PORTA_IRQHandler		[WEAK]
PORTA_IRQHandler
							EXPORT  PORTB_IRQHandler		[WEAK]
PORTB_IRQHandler
							EXPORT  PORTC_IRQHandler		[WEAK]
PORTC_IRQHandler
							EXPORT  PORTD_IRQHandler		[WEAK]
PORTD_IRQHandler
							EXPORT  PORTE_IRQHandler		[WEAK]
PORTE_IRQHandler
							EXPORT  SWI_IRQHandler			[WEAK]
SWI_IRQHandler
							EXPORT  DefaultISR				[WEAK]
DefaultISR
                B       .

                ENDP
                ALIGN


;/*
; * User Initial Stack & Heap.
; */
                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap
                ldr     r0, =Heap_Mem
                ldr     r1, =(proc_stack_mem + proc_stack_size)
                ldr     r2, =(Heap_Mem + heap_size)
                ldr     r3, =proc_stack_mem
                bx      lr

                ALIGN

                ENDIF

                END
