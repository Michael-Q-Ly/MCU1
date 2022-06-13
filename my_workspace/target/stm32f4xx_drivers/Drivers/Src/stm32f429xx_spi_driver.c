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
    // If enabling,
    if (EnorDi == ENABLE) {
        // Check which SPI peripheral and enable
        switch ((unsigned long int) pSPIx) {
                case SPI1_BASE_ADDR:        SPI1_PCLK_EN() ;        break ;
                case SPI2_BASE_ADDR:        SPI2_PCLK_EN() ;        break ;
                case SPI3_BASE_ADDR:        SPI3_PCLK_EN() ;        break ;
                case SPI4_BASE_ADDR:        SPI4_PCLK_EN() ;        break ;
                case SPI5_BASE_ADDR:        SPI5_PCLK_EN() ;        break ;
                case SPI6_BASE_ADDR:        SPI6_PCLK_EN() ;        break ;
                default:                    return ;                break ;         // TODO: return error or cause a user fault
        }
    }
    else {
        // Disable the corresponding SPI peripheral otherwise
        switch ((unsigned long int) pSPIx) {
                case SPI1_BASE_ADDR:        SPI1_PCLK_DI() ;        break ;
                case SPI2_BASE_ADDR:        SPI2_PCLK_DI() ;        break ;
                case SPI3_BASE_ADDR:        SPI3_PCLK_DI() ;        break ;
                case SPI4_BASE_ADDR:        SPI4_PCLK_DI() ;        break ;
                case SPI5_BASE_ADDR:        SPI5_PCLK_DI() ;        break ;
                case SPI6_BASE_ADDR:        SPI6_PCLK_DI() ;        break ;
                default:                    return ;                break ;         // TODO: return error or cause a user fault
        }
    }

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
    // Configure the SPI_CR1 register
    uint32_t tempReg = 0 ;

    // 1. Configure the device mode
    tempReg |=  pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

    // 2. Configure the bus mode
    switch (pSPIHandle->SPIConfig.SPI_BusConfig) {
        case SPI_BUS_CONFIG_FD:
            // BIDI mode should be cleared
            tempReg &= ~(1 << SPI_CR1_BIDIMODE) ;
            break ;
        case SPI_BUS_CONFIG_HD:
            // BIDI mode should be set
            tempReg |= (1 << SPI_CR1_BIDIMODE) ;
            break ;
        case SPI_BUS_CONFIG_SIMPLEX_RXONLY:
            // BIDI mode should be cleared and RXONLY bit must be set
            tempReg &= ~(1 << SPI_CR1_BIDIMODE) ;
            tempReg |= (1 << SPI_CR1_RXONLY) ;
            break ;
        default:
            // Set as Full Duplex by default otherwise
            tempReg &= ~(1 << SPI_CR1_BIDIMODE) ;
            break ;
    }

    // 3. configure the SPI serial clock speed (baud rate)
    tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR ;

    // 4. Configure the DFF
    tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF ;

    // 5. Configure the CPOL
    tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL ;

    //6. Configure the CPHA
    tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA ;

    pSPIHandle->pSPIx->CR1 = tempReg ;
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
 * @note                    Refer to the APB1RSTR and APB2RSTR RCC registers in the RM
 * 
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
        // Select which SPI peripheral to deinitialize
        switch ((unsigned long int) pSPIx) {
                case SPI1_BASE_ADDR:        SPI1_REG_RESET() ;          break ;
                case SPI2_BASE_ADDR:        SPI2_REG_RESET() ;          break ;
                case SPI3_BASE_ADDR:        SPI3_REG_RESET() ;          break ;
                case SPI4_BASE_ADDR:        SPI4_REG_RESET() ;          break ;
                case SPI5_BASE_ADDR:        SPI5_REG_RESET() ;          break ;
                case SPI6_BASE_ADDR:        SPI6_REG_RESET() ;          break ;
                default:                    return ;                    break ;         // TODO: return error or cause a user fault
        }
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
