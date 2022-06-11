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
    uint8_t SPI_DeviceMode ;
    uint8_t SPI_BusConfig ;
    uint8_t SPI_SclkSpeed ;
    uint8_t SPI_DFF ;
    uint8_t SPI_CPOL ;
    uint8_t SPI_CPHA ;
    uint8_t SPI_SSM ;
} SPI_Config_t ;

/**
 * @brief Handle structure for SPIx peripheral
 * 
 */
typedef struct {
    SPI_RegDef_t *pSPIx ;                                           /*!< This holds the base address of the SPIx(x:1,2,3,4,5,6) peripheral */
    SPI_Config_t SPIConfig ;
} SPI_Handle_t ;




/****************************************************************************************************
 *                                   APIs supported by this driver                                  *
 *                For more information about the APIs, check the function definitions               *
 ***************************************************************************************************/

#endif /* _STM32F429XX_SPI_DRIVER_H_ */
