/**
 * @file stm32f429xx_spi_driver.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Source file for SPI API driver implementation targeted at STM32F429XX devices
 * @version 0.1
 * @date 2022-06-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stm32f429xx_spi_driver.h"

/****************************************************************************************************
 *                                   APIs supported by this driver                                  *
 *                For more information about the APIs, check the function definitions               *
 ***************************************************************************************************/

/*
 * Peripheral clock setup
 */


/****************************************************************************************************
 * @fn                  SPI_PeriClockControl
 * 
 * @brief               Enables or disables peripheral clock for the given SPI peripheral
 * 
 * @param pSPIx         Pointer to base address of the SPI peripheral
 * @param EnorDi        ENABLE or DISABLE macros
 * 
 * @return              none
 * 
 * @note                none
 * 
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

}



/*
 * Init and De-init
 */

/****************************************************************************************************
 * @fn                      SPI_Init
 * 
 * @brief                   Initializes operation of an SPI peripheral
 * 
 * @param pSPIHandle        Struct containing SPI peripheral number and SPI config settings
 * 
 * @return                  none
 * 
 * @note                    none
 * 
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

}

/****************************************************************************************************
 * @fn                      SPI_DeInit
 * 
 * @brief                   Deinitializes operation of an SPI peripheral
 * 
 * @param pSPIx             Base address of an SPI peripheral
 * 
 * @return                  none
 * 
 * @note                    none
 * 
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

}



/*
 * Data send and receive
 */

/****************************************************************************************************
 * @fn                      SPI_SendData
 * 
 * @brief                   Shifts data from Tx line into Rx line of peripheral (slave) device
 * 
 * @param pSPIx             Base address of the SPI peripheral
 * @param pTxBuffer         Pointer to the transmitter buffer
 * @param len               Length (size) of the data transfer in bytes
 * 
 * @return                  none
 * 
 * @note                    none
 * 
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

}

/****************************************************************************************************
 * @fn                      SPI_ReceiveData
 * 
 * @brief                   Shifts data from the Tx line into Rx buffer
 * 
 * @param pSPIx             Base address of the SPI peripheral
 * @param pRxBuffer         Pointer to the receiver buffer
 * @param len               Length (size) of the data transfer in bytes
 * 
 * @return                  none
 * 
 * @note                    none
 * 
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t pRxBuffer, uint32_t len) {

}



/*
 * IRQ configuration and ISR handling
 */

/****************************************************************************************************
 * @fn                      SPI_IRQInterruptConfig
 * 
 * @brief                   Configures the IRQ of an SPI peripheral
 * 
 * @param IRQNumber         IRQ number from NVIC table of ARM Cortex M4
 * @param EnorDi            ENABLE or DISABLE macros
 * 
 * @return                  none
 * 
 * @note                    none
 * 
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

}

/****************************************************************************************************
 * @fn                      SPI_IRQPriorityConfig
 * 
 * @brief                   Configures the IRQ priority of an SPI peripheral
 * 
 * @param IRQNumber         IRQ number from NVIC table of ARM Cortex M4 UM
 * @param IRQPriority       IRQ priority number from NVIC table of ARM Cortex M4 UM
 *
 * @return                  none
 *  
 * @note                    none
 * 
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

}

/****************************************************************************************************
 * @fn                      SPI_IRQHandler
 * 
 * @brief                   Checks if the pending register is set (0) and clears it (1)
 * 
 * @param pHandle           Struct pointer to SPI handle containing SPI register definition and configurations
 * 
 * @return                  none
 * 
 * @note                    none
 * 
 */
void SPI_IRQHandler(SPI_Handle_t *pHandle) {

}



/*
 * Other peripheral control APIs
 */
