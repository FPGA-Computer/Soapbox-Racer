#include "i2c_lld.h"
#include "ch.h"
#include "nvic.h"
#include "osal.h"
#include "board.h"
#include "lib.h"

I2C_State i2c_state;

// I2C interrupt
OSAL_IRQ_HANDLER(I2C0_IRQHandler)
{ 
  uint8_t Status = I2C0->S;
  I2C0->S |= I2Cx_S_IICIF;    // Clear interrupt
  
  switch(i2c_state.State)
  {        
    case I2C_TX:
      if(Status & I2Cx_S_RXAK)    // NAK
      {
        i2c_state.State= I2C_TX_NAK;
        break;
      }
      if(i2c_state.TxSize)
      {
        I2C0->D = *i2c_state.TxBuf++;
        i2c_state.TxSize--;
        return;
      }
      else if(i2c_state.RxSize)
      { 
        // Restart
        I2C0->C1 = I2Cx_C1_IICEN|I2Cx_C1_IICIE|I2Cx_C1_MST|I2Cx_C1_TX|I2Cx_C1_RSTA;
        // resend device address + read mode
        I2C0->D = i2c_state.I2C_Addr|I2C_READ_MODE;
        i2c_state.State = I2C_DummyRead;        
        return;        
      }
      else
      {
        I2C0->C1 = I2Cx_C1_IICEN;
        i2c_state.State = I2C_IDLE;
       } 
      break;
      
    case I2C_DummyRead:  // flush I2C pipeline after switching to read mode 

        if(i2c_state.RxSize==1) // Next byte = Last byte
          I2C0->C1 = I2Cx_C1_IICEN|I2Cx_C1_IICIE|I2Cx_C1_MST|I2Cx_C1_TXAK;
        else
          I2C0->C1 = I2Cx_C1_IICEN|I2Cx_C1_IICIE|I2Cx_C1_MST;
        
        I2C0->D;

        i2c_state.State = I2C_RX; 
      return;
      
    case I2C_RX:
      if(i2c_state.RxSize)
      {  
        *i2c_state.RxBuf++=I2C0->D;  
        i2c_state.RxSize--;
        
        if(i2c_state.RxSize==1)  //(pipelined!) Next byte = Last byte 
          I2C0->C1=I2Cx_C1_IICEN|I2Cx_C1_IICIE|I2Cx_C1_MST|I2Cx_C1_TXAK;
        
        if (i2c_state.RxSize)
          return;       
        
        i2c_state.State = I2C_IDLE;
      }       
  }   

  // Arbitration lost
  if(Status & I2Cx_S_ARBL)
    i2c_state.State = I2C_ARBIT_LOST;
  
  if((i2c_state.State==I2C_IDLE)||
     (i2c_state.State==I2C_TX_NAK)||
     (i2c_state.State==I2C_ARBIT_LOST))
  { 
    I2C0->C1 = I2Cx_C1_IICEN;
    osalSysLockFromISR();
    chBSemSignalI(&i2c_state.Semaphore);
    osalSysUnlockFromISR();
    
    I2C0->C1 = 0;
  }
}

void i2c_Init(void)
{
  // Turn on clock
  SIM->SCGC4 |= SIM_SCGC4_I2C0;
  I2C0->F = I2C_400kHz;
  
  I2C0->C1 = 0;
  
  // I2C filter = (Max) 31 bus cycles (645ns)
  I2C0->FLT = 0x8;
  
  nvicEnableVector(I2C0_IRQn,I2C0_IRQ_PRIORITY);
}

void i2c_Stop(void)
{
  nvicDisableVector(I2C0_IRQn);
  
  // Disable I2C & interrupt
  I2C0->C1 = 0;
  // Turn off clock  
  SIM->SCGC4 &= ~SIM_SCGC4_I2C0;
}

msg_t i2c_transfer(uint8_t Addr, uint8_t *TxBuf, uint8_t TxSize,
                   uint8_t *RxBuf, uint8_t RxSize)
{ uint8_t i;
  
  i2c_state.TxBuf = TxBuf;
  i2c_state.RxBuf = RxBuf;
  i2c_state.TxSize = TxSize;
  i2c_state.RxSize = RxSize;
  i2c_state.I2C_Addr = Addr;
  
  // bus master
  I2C0->C1 = I2Cx_C1_MST;
  I2C0->SMB = I2Cx_SMB_TCKSEL;
  
  // sleep until bus is free
  
  for(i=I2C_BUSY_TIMEOUT;(I2C0->S & I2Cx_S_BUSY)&&i;i--)
    chThdSleepMicroseconds(I2C_BUSY_WAIT_us);
  
  if(i)
  {
    // Send slave address
    chBSemObjectInit(&i2c_state.Semaphore,1);    
    I2C0->C1 = I2Cx_C1_IICEN|I2Cx_C1_IICIE|I2Cx_C1_MST|I2Cx_C1_TX;
    I2C0->D = Addr;
    i2c_state.State = I2C_TX;
    
    // The rest of transfer is done in interrupt.
    // Wait until it is done or timeout.
    
    if(chBSemWaitTimeout(&i2c_state.Semaphore,I2C_BUSY_TIMEOUT)==MSG_OK)
      return(i2c_state.State);
  }
  return(MSG_TIMEOUT);
}
