/**
 * @file stm32f429xx_spi_driver.h
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Header file for SPI driver targeted at STM32F429XX devices
 * @version 0.1
 * @date 2022-06-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _STM32F429XX_SPI_DRIVER_H_
#define _STM32F429XX_SPI_DRIVER_H_
#include "stm32f429xx.h"

/**
 * @brief Configuration structure for SPIx peripheral
 * 
 */
typedef struct {
    uint8_t SPI_DeviceMode ;                                        /*!< Possible values from @ SPI_DeviceMode >*/
    uint8_t SPI_BusConfig ;                                         /*!< Possible values from @ SPI_BusConfig >*/
    uint8_t SPI_SclkSpeed ;                                         /*!< Possible values from @ SPI_SclkSpeed >*/
    uint8_t SPI_DFF ;                                               /*!< Possible values from @ SPI_DFF >*/
    uint8_t SPI_CPOL ;                                              /*!< Possible values from @ SPI_CPOL >*/
    uint8_t SPI_CPHA ;                                              /*!< Possible values from @ SPI_CPHA >*/
    uint8_t SPI_SSM ;                                               /*!< Possible values from @ SPI_SSM >*/
} SPI_Config_t ;

/**
 * @brief Handle structure for SPIx peripheral
 * 
 */
typedef struct {
    SPI_RegDef_t *pSPIx ;                                           /*!< This holds the base address of the SPIx(x:1,2,3,4,5,6) peripheral >*/
    SPI_Config_t SPIConfig ;                                        /*!< This holds the configuration of the SPI peripheral >*/
    uint8_t *pTxBuffer ; 											/*!< To store the app. Tx buffer address >*/
    uint8_t *pRxBuffer ; 											/*!< To store the app. Rx buffer address >*/
    uint32_t TxLen ; 												/*!< To store Tx len >*/
    uint32_t RxLen ; 												/*!< To store Rx len >*/
    uint8_t TxState ; 												/*!< To store Tx state >*/
    uint8_t RxState ; 												/*!< To store Rx state >*/
} SPI_Handle_t ;

/*
 * SPI Application States
 */

#define SPI_READY 			0
#define SPI_BUSY_IN_RX 		1
#define SPI_BUSY_IN_TX 		2

/*
 * @SPI_DeviceMode
 * SPI Configuration: Device Mode as a controller or peripheral
 */

#define SPI_DEVICE_MODE_CONTROLLER          1                       /*!< Let us try to move away from MASTER notation */
#define SPI_DEVICE_MODE_PERIPHERAL          0                       /*!< Let us try to move away from SLAVE notation */

/*
 * @SPI_BusConfig
 * SPI Configuration: Set device mode as full duplex, half duplex, or simplex (TX-only or RX-only)
 */

#define SPI_BUS_CONFIG_FD                   1                       /*!< Full Duplex Mode >*/
#define SPI_BUS_CONFIG_HD                   2                       /*!< Half Duplex Mode >*/
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY       3                       /*!< Simplex Mode - Tx-only >*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY       4                       /*!< Simplex Mode - Rx-only >*/

/*
 * @SPI_SclkSpeed
 * SPI Configuration: SPI Serial Clock Speed division
 */

#define SPI_SCLK_SPEED_DIV2                 0                       /*!< Serial clock prescaler >*/
#define SPI_SCLK_SPEED_DIV4                 1                       /*!< Serial clock prescaler >*/
#define SPI_SCLK_SPEED_DIV8                 2                       /*!< Serial clock prescaler >*/
#define SPI_SCLK_SPEED_DIV16                3                       /*!< Serial clock prescaler >*/
#define SPI_SCLK_SPEED_DIV32                4                       /*!< Serial clock prescaler >*/
#define SPI_SCLK_SPEED_DIV64                5                       /*!< Serial clock prescaler >*/
#define SPI_SCLK_SPEED_DIV128               6                       /*!< Serial clock prescaler >*/
#define SPI_SCLK_SPEED_DIV256               7                       /*!< Serial clock prescaler >*/

/*
 * @SPI_DFF
 * SPI Configuration: Data Frame Format - 8-bit or 16-bit TX or RX
 */

#define SPI_DFF_8BITS                       0                       /*!< Transmit or receive 8 bits >*/
#define SPI_DFF_16BITS                      1                       /*!< Transmit or receive 16 bits >*/

/*
 * @SPI_CPOL
 * SPI Configuration: Clock Polarity - 0 or 1 when idle
 */

#define SPI_CPOL_LOW                        0                       /*!< Idle when clock is LOW >*/
#define SPI_CPOL_HIGH                       1                       /*!< Idle when clock is HIGH >*/

/*
 * @SPI_CPHA
 * SPI Configuration: Clock Phase - first or second edge of clock is first capture edge
 */

#define SPI_CPHA_LOW                        0                       /*!< First clock transition is the first data capture edge >*/
#define SPI_CPHA_HIGH                       1                       /*!< Second clock transition is the first data capture edge >*/

/*
 * @SPI_SSM
 * SPI Configuration: Software Slave (Peripheral) Management
 */

#define SPI_SSM_DI                          0                       /*!< Software slave management disabled >*/
#define SPI_SSM_EN                          1                       /*!< Software slave management enabled >*/

/*
 * SPI-related status flag definitions
 */

#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)      /*!< Status Register RX EnableZ flag >*/
#define SPI_TXE_FLAG                        (1 << SPI_SR_TXE)       /*!< Status Register TX Enable flag >*/
#define SPI_BSY_FLAG                        (1 << SPI_SR_BSY)       /*!< Status Register SPI Busy flag >*/





/****************************************************************************************************
 *                                   APIs supported by this driver                                  *
 *                For more information about the APIs, check the function definitions               *
 ***************************************************************************************************/

/*
 * Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) ;

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle) ;
void SPI_DeInit(SPI_RegDef_t *pSPIx) ;

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) ;
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) ;


uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) ;
void SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) ;

/*
 * IRQ configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) ;
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) ;
void SPI_IRQHandler(SPI_Handle_t *pHandle) ;

/*
 * Other peripheral control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) ;
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) ;
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) ;
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) ;

#endif /* _STM32F429XX_SPI_DRIVER_H_ */
