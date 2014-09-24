#include "board.h"
#include "chtypes.h"

// I2C macros
#define I2C_400kHz                  0x1b
#define I2C_TIMEOUT                 5000
#define I2C_SLT_CYC                 I2C_TIMEOUT

#define I2C0_IRQ_PRIORITY           8

#define I2C_BUSY_WAIT_us            1000
#define I2C_BUSY_TIMEOUT            100

#define I2C_READ_MODE               0x01
#define XRA1201_ADDR                0x40
#define SI5351A_ADDR                0x60

enum XRA1201_REG{ XRA1201_GSR1, XRA1201_GSR2, XRA1201_OCR1, XRA1201_OCR2,
                  XRA1201_PIR1, XRA1201_PIR2, XRA1201_GCR1, XRA1201_GCR2,
                  XRA1201_PUR1, XRA1201_PUR2, XRA1201_IER1, XRA1201_IER2,
                  XRA1201_TSCR1,XRA1201_TSCR2,XRA1201_ISR1, XRA1201_ISR2,
                  XRA1201_REIR1,XRA1201_REIR2,XRA1201_FEIR1,XRA1201_FEIR2,
                  XRA1201_IFR1, XRA1201_IFR2, XRA1201_REGSIZE };
       
void i2c_Init(void);
void i2c_Stop(void);
msg_t i2c_transfer(uint8_t Addr, uint8_t *TxBuf, uint8_t TxSize,
                   uint8_t *RxBuf, uint8_t RxSize);
                  
enum I2C_States { I2C_IDLE, I2C_TX, I2C_DummyRead, I2C_RX, I2C_TX_NAK, I2C_ARBIT_LOST };

typedef struct
{
  uint8_t             *TxBuf;
  uint8_t             *RxBuf;
  binary_semaphore_t  Semaphore;      // for synchronization
  uint8_t             TxSize;
  uint8_t             RxSize;
  uint8_t             I2C_Addr;
  uint8_t             State;
} I2C_State;
