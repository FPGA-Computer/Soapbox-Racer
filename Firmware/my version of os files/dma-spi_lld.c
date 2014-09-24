#include "hal.h"
#include "dma.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI0 driver identifier.*/
SPIDriver SPID1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint8_t SPI_dummy;

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Shared end-of-tx service routine.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void spi_lld_serve_tx_interrupt(SPIDriver *spip)
{ 
  /* Stop Tx DREQ */
  DMA->CERQ = spip->DMA_Tx;
  
  /* Portable SPI ISR code defined in the high level driver, note, it is
     a macro.*/
  
  _spi_isr_code(spip);  
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void)
{
#if KINETIS_SPI_USE_SPI0
  
  SPI_dummy = 0xff;  
  spiObjectInit(&SPID1);
  SPID1.spi = SPI0;
  
  SPID1.DMA_Rx = DMA_Alloc(FALSE,SPI_DMA_PRIORITY,NULL,NULL);
  SPID1.DMA_Tx = DMA_Alloc(FALSE,
                           SPI_DMA_PRIORITY,(void (*)(void *))spi_lld_serve_tx_interrupt,&SPID1); 
  
  // can't allocate DMA channels!?
  if((SPID1.DMA_Rx==DMA_NONE_AVAIL)||(SPID1.DMA_Tx==DMA_NONE_AVAIL))
  {
    spi_lld_stop(&SPID1);
    return;
  }
  
  // Enable SPI clock
  SIM->SCGC6 |= SIM_SCGC6_SPI0;
  
  // Set up SPI FIFO as fixed DMA Src/Dst addr, single byte src/dst & minor loop
  DMA->TCD[SPID1.DMA_Rx].SADDR = (uint32_t) &SPI0->POPR;
  DMA->TCD[SPID1.DMA_Rx].SOFF=0;
  DMA->TCD[SPID1.DMA_Rx].SLAST=0;
  DMA->TCD[SPID1.DMA_Rx].NBYTES_MLNO=1;
  DMA->TCD[SPID1.DMA_Rx].ATTR = DMA_ATTR_DSIZE(0)|DMA_ATTR_SSIZE(0);
  DMA->TCD[SPID1.DMA_Rx].CSR = DMA_CSR_DREQ_MASK;  
  
  DMA->TCD[SPID1.DMA_Tx].DADDR = (uint32_t) &SPI0->PUSHR;
  DMA->TCD[SPID1.DMA_Tx].DOFF=0;  
  DMA->TCD[SPID1.DMA_Tx].DLAST_SGA=0;
  DMA->TCD[SPID1.DMA_Tx].NBYTES_MLNO=1;  
  DMA->TCD[SPID1.DMA_Tx].ATTR = DMA_ATTR_DSIZE(0)|DMA_ATTR_SSIZE(0);
  DMA->TCD[SPID1.DMA_Tx].CSR = DMA_CSR_DREQ_MASK|DMA_CSR_INTMAJOR_MASK;

  // Enable DMA IRQ
  nvicEnableVector(DMA0_IRQn+SPID1.DMA_Tx,SPI_INT_PRIORITY);
  
  // Set up DMA MUX for SPI
  DMAMUX->CHCFG[SPID1.DMA_Rx]= DMAMUX_CHCFGn_ENBL|
                               DMAMUX_CHCFGn_SOURCE(DMA_SLOT_SPI_RX);
  DMAMUX->CHCFG[SPID1.DMA_Tx]= DMAMUX_CHCFGn_ENBL|
                               DMAMUX_CHCFGn_SOURCE(DMA_SLOT_SPI_TX);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip)
{
  /* If in stopped state then renables the SPI and DMA clocks.*/
  if ((spip->state == SPI_STOP)&&(&SPID1 == spip))
  {
    /* enable the clock for SPI0 */
    SIM->SCGC6 |= SIM_SCGC6_SPI0;
    
    /* clear SPI halt */
    spip->spi->MCR &= ~SPIx_MCR_HALT;
   }
  
  /* SPI setup and enable.*/
  if(spip->config)
    spip->spi->CTAR[0] = spip->config->ctar;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip)
{
  /* If in ready state then disables the SPI clock.*/
  if ((spip->state == SPI_READY)&&(&SPID1 == spip))
  {
    
//    DMA_Release(spip->DMA_Rx);
//    DMA_Release(spip->DMA_Tx);    
    
    /* SPI halt.*/
    spip->spi->MCR |= SPIx_MCR_HALT;    
    /* Disable the clock for SPI0 */
    SIM->SCGC6 &= ~SIM_SCGC6_SPI0;
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) 
{ uint8_t PortValue;
  
  if((spip->config) && (spip->config->Port))
  {
    PortValue = *spip->config->Port & ~spip->config->CS_Mask;
    *spip->config->Port = PortValue|spip->config->CS_Enable;
  }
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *          This has no effect if we are using the KINETIS PCS mode to
 *          manage slave select.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip)
{ uint8_t PortValue;
  
  if((spip->config) && (spip->config->Port))
  {
    PortValue =  *spip->config->Port & ~spip->config->CS_Mask;
    *spip->config->Port = PortValue|spip->config->CS_Disable;
  }
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */

void spi_lld_exchange(SPIDriver *spip, size_t n, const void *txbuf, void *rxbuf)
{
  // clear all SPI flags
  spip->spi->SR= SPIx_SR_TCF|SPIx_SR_TXRXS|SPIx_SR_EOQF|SPIx_SR_TFUF|
                 SPIx_SR_TFFF|SPIx_SR_RFOF|SPIx_SR_RFDF;
  
  spip->spi->MCR = SPIx_MCR_MSTR|SPIx_MCR_CLR_TXF|SPIx_MCR_CLR_RXF;
  
  if(rxbuf)
  {
    DMA->TCD[spip->DMA_Rx].CITER_ELINKNO = 
    DMA->TCD[spip->DMA_Rx].BITER_ELINKNO = n;
    DMA->TCD[spip->DMA_Rx].DADDR = (uint32_t) rxbuf;
    DMA->TCD[spip->DMA_Rx].DOFF = 1;
    
    spip->spi->RSER = SPI_TX_DMA_EN|SPI_RX_DMA_EN;
    
    // Enable Rx DMA Request    
    DMA->SERQ = spip->DMA_Rx;
  }
  else
  {
    spip->spi->RSER = SPI_TX_DMA_EN;
  } 
  
  if(txbuf)
  { 
    DMA->TCD[spip->DMA_Tx].SADDR = (uint32_t) txbuf;
    DMA->TCD[spip->DMA_Tx].SOFF = 1;
  }
  else
  {
    DMA->TCD[spip->DMA_Tx].SADDR = (uint32_t) &SPI_dummy;
    DMA->TCD[spip->DMA_Tx].SOFF = 0;
  }
  
  DMA->TCD[spip->DMA_Tx].CITER_ELINKNO =
  DMA->TCD[spip->DMA_Tx].BITER_ELINKNO = n;
  
  // Enable Tx DMA Request
  DMA->SERQ = spip->DMA_Tx;
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */

inline void spi_lld_ignore(SPIDriver *spip,size_t n)
{  
  spi_lld_exchange(spip,n,NULL,NULL);
}
/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */

inline void spi_lld_send(SPIDriver *spip,size_t n,const void *txbuf)
{
  spi_lld_exchange(spip,n,txbuf,NULL);
}
/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
inline void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf)
{
  spi_lld_exchange(spip,n,NULL,rxbuf);
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame)
{
  SPI0->RSER = 0;
  spip->spi->MCR = SPIx_MCR_MSTR|SPIx_MCR_ROOE|
                   SPIx_MCR_CLR_TXF|SPIx_MCR_CLR_RXF;

  spip->spi->PUSHR = SPIx_PUSHR_TXDATA(frame);

  while ((spip->spi->SR & SPIx_SR_RFDF) == 0)
    ;

  frame = spip->spi->POPR;
  
  spip->spi->MCR |= SPIx_MCR_HALT;
  return frame;
}

#endif /* HAL_USE_SPI */

/** @} */
