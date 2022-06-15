/**
 * @file 006_SPITxOnlyArduino.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Arduino as a SPI peripheral device with STM32F429xx controller
 * @version 0.1
 * @date 2022-06-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"
#include "stm32f429xx_spi_driver.h"
#include <string.h>

void SPI2_GPIOInits(void) ;
void SPI2_Inits(void) ;
void GPIO_ButtonInit(void) ;
void delay(void) ;

int main(void) {
    // Create a message to transmit
    char const *user_data = "Hello world" ;

    // Configure GPIO button
    GPIO_ButtonInit() ; 

    // Configure GPIO to behave as SPI2 pins
    SPI2_GPIOInits() ;

    // Initialize SPI2 peripheral parameters
    SPI2_Inits() ;

    /*
     * Setting SSOE does NSS output enable.
     * The NSS pin is automatically managed by the hardware.
     * I.e., when SPE = 1, NSS will be pulled LOW,
     * and NSS pin will be HIGH when SPE = 0.
     */
    SPI_SSOEConfig(SPI2, ENABLE) ;

    while (1) {
        // Wait until a button press
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)) ;

        // Debounce button press with software delay for clean read
        delay() ;

        // Enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE) ;

        // First send length information so that the peripheral knows how long the message is
        uint8_t dataLen = strlen(user_data) ;
        // to send 1 byte of data, which turns out to be 0x0B, the read opcode
        SPI_SendData(SPI2, &dataLen, 1) ;

        // Transmit data
        SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data)) ;

        // Wait for BSY bit to reset  -> This will indicate that SPI is not busy in communication
        while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) == FLAG_SET) ;

        // Clear the OVR flag by reading DR and SR
        uint8_t temp = SPI2->DR ;
        temp = SPI2->SR ;
        (void) temp ;                           /*!< temp is not always used, so typecast will avoid warning on compilation for ununsed variable >*/

        // Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, DISABLE) ;
    }

    // Program should never reach here!
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
    memset(&SPIPins, 0, sizeof(SPIPins)) ;

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
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9 ;
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
    GPIO_Init(&SPIPins) ;

    // SCK
    SPIPins.pGPIOx = GPIOB ;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10 ;
    GPIO_Init(&SPIPins) ;

    // MOSI
    SPIPins.pGPIOx = GPIOC ;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3 ;
    GPIO_Init(&SPIPins) ;

#ifdef later
    // SCK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13 ;
    GPIO_Init(&SPIPins) ;

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15 ;
    GPIO_Init(&SPIPins) ;
#endif /* later */

    /*
     * Disabled for this particular application since there is no peripheral device
     */

//     // MISO
//     SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14 ;
//     GPIO_Init(&SPIPins) ;
}

/****************************************************************************************************
 * @fn                  SPI2_Inits
 * 
 * @brief               Initializes SPI2 peripheral
 * 
 * @return              none
 * 
 * @note                none
 * 
 */
void SPI2_Inits(void) {
    SPI_Handle_t SPI2Handle ;
    memset(&SPI2Handle, 0, sizeof(SPI2Handle)) ;

    SPI2Handle.pSPIx = SPI2 ;
    SPI2Handle.SPIConfig.SPI_DeviceMode     = SPI_DEVICE_MODE_CONTROLLER ;
    SPI2Handle.SPIConfig.SPI_BusConfig      = SPI_BUS_CONFIG_FD ;
    SPI2Handle.SPIConfig.SPI_SclkSpeed      = SPI_SCLK_SPEED_DIV8 ;                     /* Generates SCK of 2 MHz */
    SPI2Handle.SPIConfig.SPI_DFF            = SPI_DFF_8BITS ;
    SPI2Handle.SPIConfig.SPI_CPOL           = SPI_CPOL_LOW ;
    SPI2Handle.SPIConfig.SPI_CPHA           = SPI_CPHA_LOW ;
    SPI2Handle.SPIConfig.SPI_SSM            = SPI_SSM_DI ;                              /* Hardware slave management enabled for NSS pin */

    SPI_Init(&SPI2Handle) ;
}

/****************************************************************************************************
 * @fn                  GPIO_buttonInit
 * 
 * @brief               Initializes button as a GPIO
 * 
 * @return              none
 * 
 * @note                none
 * 
 */
void GPIO_ButtonInit(void) {
    GPIO_Handle_t GPIOHandle ;
    memset(&GPIOHandle, 0, sizeof(GPIOHandle)) ;

    GPIOHandle.pGPIOx = GPIOA ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_0 ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_INPUT ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl  = GPIO_NO_PUPD ;

    GPIO_Init(&GPIOHandle) ;
}
/****************************************************************************************************
 * @fn          delay
 * 
 * @brief       Software delay; can be used for debouncing
 * 
 * @return      none
 * 
 * @note        none
 * 
 */
void delay(void) {
    for (uint32_t i = 0 ; i < 500000/2 ; i++) ;
}
