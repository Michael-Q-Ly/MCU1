/**
 * @file 005_SPITxTesting.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Test if SPI API Driver header works or not
 * @version 0.1
 * @date 2022-06-13
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note SPI alternate functionality pin numbers:
 *          PB12     SPI2_NSS
 *          PB13     SPI2_SCK
 *          PB14     SPI2_MISO
 *          PB15     SPI2_MOSI
 *          Alt Function Mode: 5
 * 
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"
#include "stm32f429xx_spi_driver.h"

void SPI2_GPIOInits(void) ;
void SPI2_inits(void) ;

int main(void) {
    SPI2_GPIOInits() ;
    return 0 ;
}

/****************************************************************************************************
 * @fn                  SPI2_GPIOInits
 * 
 * @brief               Initialize the GPIO pins to behave as SPI2 pins
 * 
 * @return              none
 * 
 * @note                none
 * 
 */
void SPI2_GPIOInits(void) {
    /*
     * Configure the SPI pins
     */

    GPIO_Handle_t SPIPins ;
    SPIPins.pGPIOx = GPIOB ;
    SPIPins.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_ALT_FUN ;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST ;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl     = GPIO_NO_PUPD ;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP ;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode   = 5 ;


    /*
     * Configure the SPI2 pins
     */

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
    GPIO_Init(&SPIPins) ;
    // SCK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13 ;
    GPIO_Init(&SPIPins) ;
    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14 ;
    GPIO_Init(&SPIPins) ;
    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15 ;
    GPIO_Init(&SPIPins) ;
}

/**
 * @brief Initializes SPI2 peripheral
 * 
 */
void SPI2_inits(void) {
    SPI_Handle_t SPI2Handle ;

    SPI2Handle.pSPIx = SPI2 ;
    SPI2Handle.SPIConfig.SPI_DeviceMode     = SPI_DEVICE_MODE_CONTROLLER ;
    SPI2Handle.SPIConfig.SPI_BusConfig      = SPI_BUS_CONFIG_FD ;
    SPI2Handle.SPIConfig.SPI_SclkSpeed      = SPI_SCLK_SPEED_DIV2 ;                     /* Generates SCK of 8 MHz */
    SPI2Handle.SPIConfig.SPI_DFF            = SPI_DFF_8BITS ;
    SPI2Handle.SPIConfig.SPI_CPOL           = SPI_CPOL_LOW ;
    SPI2Handle.SPIConfig.SPI_CPHA           = SPI_CPHA_LOW ;
    SPI2Handle.SPIConfig.SPI_SSM            = SPI_SSM_EN ;                              /* Software slave management enabled for NSS pin */

    SPI_Init(&SPI2Handle) ;
}
