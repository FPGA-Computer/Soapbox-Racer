/*
    ChibiOS/HAL - Copyright (C) 2014 Derek Mulcahy

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
 * @file    KINETIS/spi_lld.c
 * @brief   KINETIS SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"
#include "dma.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if !defined(KINETIS_SPI_USE_SPI0)
#define KINETIS_SPI_USE_SPI0                TRUE
#endif

#if !defined(KINETIS_SPI0_IRQ_PRIORITY)
#define KINETIS_SPI0_IRQ_PRIORITY    8
#endif

#if !defined(SPI_DMA_PRIORITY)
#define SPI_DMA_PRIORITY        8
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI0 driver identifier.*/
#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

volatile uint16_t dmaTxDummy, dmaRxDummy;

static void spi_start_xfer(SPIDriver *spip)
{
  /*
   * Enable the DSPI peripheral in master mode.
   * Clear the TX and RX FIFOs.
   * */
  spip->spi->MCR = SPIx_MCR_MSTR | SPIx_MCR_CLR_TXF | SPIx_MCR_CLR_RXF;

  /* not polling */
  {
    
    /* Enable receive dma and transmit dma */
    spip->spi->RSER = SPIx_RSER_RFDF_DIRS | SPIx_RSER_RFDF_RE |
        SPIx_RSER_TFFF_RE | SPIx_RSER_TFFF_DIRS;

    /* Use dmaDummy as the source/destination when a buffer is not provided */
    dmaTxDummy = SPI_IGNORE_SENDBYTE;

    /* Configure RX DMA */
    if (spip->rxbuf) {
      DMA->TCD[spip->DMA_Rx].DADDR = (uint32_t)spip->rxbuf;
      DMA->TCD[spip->DMA_Rx].DOFF = spip->word_size;
    } else {
      DMA->TCD[spip->DMA_Rx].DADDR = (uint32_t)&dmaRxDummy;
      DMA->TCD[spip->DMA_Rx].DOFF = 0;
    }
    DMA->TCD[spip->DMA_Rx].BITER_ELINKNO = spip->count;
    DMA->TCD[spip->DMA_Rx].CITER_ELINKNO = spip->count;

    /* Enable Request Register (ERQ) for RX by writing 0 to SERQ */
    DMA->SERQ = spip->DMA_Rx;

    /* Configure TX DMA */
    if (spip->txbuf) {
      DMA->TCD[spip->DMA_Tx].SADDR = (uint32_t)spip->txbuf;
      DMA->TCD[spip->DMA_Tx].SOFF = spip->word_size;
    } else {
      DMA->TCD[spip->DMA_Tx].SADDR =  (uint32_t)&dmaTxDummy;
      DMA->TCD[spip->DMA_Tx].SOFF = 0;
    }
    DMA->TCD[spip->DMA_Tx].BITER_ELINKNO = spip->count;
    DMA->TCD[spip->DMA_Tx].CITER_ELINKNO = spip->count;

    /* Enable Request Register (ERQ) for TX by writing 1 to SERQ */
    DMA->SERQ = spip->DMA_Tx;
  }
}

static void spi_stop_xfer(SPIDriver *spip)
{
  /* Halt the DSPI peripheral */
  spip->spi->MCR = SPIx_MCR_MSTR | SPIx_MCR_HALT;

  /* Clear all the flags which are currently set. */
  spip->spi->SR |= spip->spi->SR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/
static void spi_lld_serve_rx_interrupt(SPIDriver *spip)
{
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
void spi_lld_init(void) {
#if KINETIS_SPI_USE_SPI0
  spiObjectInit(&SPID1);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {
    
    #if KINETIS_SPI_USE_SPI0 
    if (&SPID1 == spip) {
    
      /* Enable the clock for SPI0 */
      SIM->SCGC6 |= SIM_SCGC6_SPI0;

      SPID1.spi = SPI0;
    }
    #endif
   
    spip->DMA_Rx = DMA_Alloc(FALSE,SPI_DMA_PRIORITY,(void (*)(void *))
                             spi_lld_serve_rx_interrupt,spip);
    spip->DMA_Tx = DMA_Alloc(FALSE,SPI_DMA_PRIORITY,NULL,NULL); 
  
    // can't allocate DMA channels!?
    chDbgAssert((SPID1.DMA_Rx==DMA_NONE_AVAIL)||
                (SPID1.DMA_Tx==DMA_NONE_AVAIL),"No DMA Ch.");
 
    nvicEnableVector(DMA0_IRQn+spip->DMA_Rx,KINETIS_SPI0_IRQ_PRIORITY);
  }
    /* Clear DMA error flags for Tx Rx DMA channels */
    DMA->ERR |= (1<<spip->DMA_Rx)|(1<<spip->DMA_Tx);

    /* Rx, select SPI Rx FIFO */
    DMAMUX->CHCFG[spip->DMA_Rx] = DMAMUX_CHCFGn_ENBL |
        DMAMUX_CHCFGn_SOURCE(DMAMUX_SPI_RX_SOURCE);

    /* Tx, select SPI Tx FIFO */
    DMAMUX->CHCFG[spip->DMA_Tx] = DMAMUX_CHCFGn_ENBL |
        DMAMUX_CHCFGn_SOURCE(DMAMUX_SPI_TX_SOURCE);

    /* Extract the frame size from the TAR */
    uint16_t frame_size = ((spip->spi->CTAR[0] >> SPIx_CTARn_FMSZ_SHIFT) &
        SPIx_CTARn_FMSZ_MASK) + 1;

    /* DMA transfer size is 16 bits for a frame size > 8 bits */
    uint16_t dma_size = frame_size > 8 ? 1 : 0;

    /* DMA word size is 2 for a 16 bit frame size */
    spip->word_size = frame_size > 8 ? 2 : 1;

    /* configure DMA RX fixed values */
    DMA->TCD[spip->DMA_Rx].SADDR = (uint32_t)&SPI0->POPR;
    DMA->TCD[spip->DMA_Rx].SOFF = 0;
    DMA->TCD[spip->DMA_Rx].SLAST = 0;
    DMA->TCD[spip->DMA_Rx].DLAST_SGA = 0;
    DMA->TCD[spip->DMA_Rx].ATTR = DMA_ATTR_SSIZE(dma_size) |
        DMA_ATTR_DSIZE(dma_size);
    DMA->TCD[spip->DMA_Rx].NBYTES_MLNO = spip->word_size;
    DMA->TCD[spip->DMA_Rx].CSR = DMA_CSR_DREQ_MASK|
        DMA_CSR_INTMAJOR_MASK;

    /* configure DMA TX fixed values */
    DMA->TCD[spip->DMA_Tx].SLAST = 0;
    DMA->TCD[spip->DMA_Tx].DADDR = (uint32_t)&SPI0->PUSHR;
    DMA->TCD[spip->DMA_Tx].DOFF = 0;
    DMA->TCD[spip->DMA_Tx].DLAST_SGA = 0;
    DMA->TCD[spip->DMA_Tx].ATTR = DMA_ATTR_SSIZE(dma_size) |
        DMA_ATTR_DSIZE(dma_size);
    DMA->TCD[spip->DMA_Tx].NBYTES_MLNO = spip->word_size;
    DMA->TCD[spip->DMA_Tx].CSR = DMA_CSR_DREQ_MASK;

    #if KINETIS_SPI_USE_SPI0

    if (spip->config->tar0) {
      spip->spi->CTAR[0] = spip->config->tar0;
    } else {
      spip->spi->CTAR[0] = KINETIS_SPI_TAR0_DEFAULT;
    }
    #endif
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  /* If in ready state then disables the SPI clock.*/
  if (spip->state == SPI_READY) {
    nvicDisableVector(spip->DMA_Rx);
    
  DMA_Release(spip->DMA_Rx);
  DMA_Release(spip->DMA_Tx);    
    
#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {
      /* SPI halt.*/
      spip->spi->MCR |= SPIx_MCR_HALT;
    }
#endif

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
void spi_lld_select(SPIDriver *spip) {
  uint8_t PortValue;
  
  PortValue = *spip->config->Port & ~spip->config->CS_Mask;
  *spip->config->Port = PortValue|spip->config->CS_Enable;
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {
  uint8_t PortValue;
  
  PortValue =  *spip->config->Port & ~spip->config->CS_Mask;
  *spip->config->Port = PortValue|spip->config->CS_Disable;
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
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  spip->count = n;
  spip->rxbuf = NULL;
  spip->txbuf = NULL;

  spi_start_xfer(spip);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  spip->count = n;
  spip->rxbuf = rxbuf;
  spip->txbuf = txbuf;

  spi_start_xfer(spip);
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
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  spip->count = n;
  spip->rxbuf = NULL;
  spip->txbuf = (void *)txbuf;

  spi_start_xfer(spip);
  
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
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  spip->count = n;
  spip->rxbuf = rxbuf;
  spip->txbuf = NULL;

  spi_start_xfer(spip);
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
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  spi_start_xfer(spip);

  spip->spi->PUSHR = SPIx_PUSHR_TXDATA(frame);

  while ((spip->spi->SR & SPIx_SR_RFDF) == 0)
    ;

  frame = spip->spi->POPR;

  spi_stop_xfer(spip);

  return frame;
}

#endif /* HAL_USE_SPI */

/** @} */
