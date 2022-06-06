/**
 * @file stm32f429xx_gpio_driver.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stm32f429xx_gpio_driver.h"

/****************************************************************************************************
 *                                   APIs supported by this driver                                  *
 *                For more information about the APIs, check the function definitions               *
 ***************************************************************************************************/

/*
 * Peripheral clock setup
 */

/****************************************************************************************************
 * @fn                  GPIO_PeriClockControl
 * 
 * @brief               Enables or disables peripheral clock for the given GPIO port
 * 
 * @param[in] pGPIOx    Base address of the GPIO peripheral
 * @param[in] EnorDi    ENABLE or DISABLE macros
 * 
 * @return              none
 * 
 * @note                none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
    // If enabled,
    if (EnorDi == ENABLE) {
        // Check which GPIO and enable
        switch ((unsigned long int) pGPIOx) {
                case GPIOA_BASE_ADDR:       GPIOA_PCLK_EN() ;       break ;
                case GPIOB_BASE_ADDR:       GPIOB_PCLK_EN() ;       break ;
                case GPIOC_BASE_ADDR:       GPIOC_PCLK_EN() ;       break ;
                case GPIOD_BASE_ADDR:       GPIOD_PCLK_EN() ;       break ;
                case GPIOE_BASE_ADDR:       GPIOE_PCLK_EN() ;       break ;
                case GPIOF_BASE_ADDR:       GPIOF_PCLK_EN() ;       break ;
                case GPIOG_BASE_ADDR:       GPIOG_PCLK_EN() ;       break ;
                case GPIOH_BASE_ADDR:       GPIOH_PCLK_EN() ;       break ;
                case GPIOI_BASE_ADDR:       GPIOI_PCLK_EN() ;       break ;
                case GPIOJ_BASE_ADDR:       GPIOJ_PCLK_EN() ;       break ;
                case GPIOK_BASE_ADDR:       GPIOK_PCLK_EN() ;       break ;
                default:                    return ;                break ;         // TODO: return error or cause a user fault
        }
    }
    else {
        // Disable the GPIO otherwise
        switch ((unsigned long int) pGPIOx) {
                case GPIOA_BASE_ADDR:       GPIOA_PCLK_DI() ;       break ;
                case GPIOB_BASE_ADDR:       GPIOB_PCLK_DI() ;       break ;
                case GPIOC_BASE_ADDR:       GPIOC_PCLK_DI() ;       break ;
                case GPIOD_BASE_ADDR:       GPIOD_PCLK_DI() ;       break ;
                case GPIOE_BASE_ADDR:       GPIOE_PCLK_DI() ;       break ;
                case GPIOF_BASE_ADDR:       GPIOF_PCLK_DI() ;       break ;
                case GPIOG_BASE_ADDR:       GPIOG_PCLK_DI() ;       break ;
                case GPIOH_BASE_ADDR:       GPIOH_PCLK_DI() ;       break ;
                case GPIOI_BASE_ADDR:       GPIOI_PCLK_DI() ;       break ;
                case GPIOJ_BASE_ADDR:       GPIOJ_PCLK_DI() ;       break ;
                case GPIOK_BASE_ADDR:       GPIOK_PCLK_DI() ;       break ;
                default:                    return ;                break ;         // TODO: return error or cause a user fault
        }
    }
}

/*
 * Init and De-init
 */

/****************************************************************************************************
 * @fn                          GPIO_Init
 * 
 * @brief                       Initializes operation of a GPIO: Mode, Speed, PUPD, AltFn
 * 
 * @param[in] pGPIO_Handle      Struct containing GPIO number and GPIO config settings
 * 
 * @return                      none
 * 
 * @note                        In the implementation, we multiply by 2 for some configurations. This
 *                              is because some of the registers use 2 bit fields per pin.
 *                              The alt function uses 4 bit fields per pin, however, and is an array.
 *                              The reason we use the integer division and modulus is so we know which
 *                              index to address in the alt fun (high [1] or low [0] register).
 *                              Example: Pin 10 gives us 10/8 = 1 -> high reg
 *                                          10%8 = 2 -> We take this and multiply by 4 and then left shift
 *                                          by that amount. This gives us a left shift of 8, landing us
 *                                          on bit 8 of the high reg, which is the lsb of pin 10's four
 *                                          bit fields!
 *                                      
 */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) {
    // 1. Configure the mode of GPIO pin
    uint32_t temp = 0 ;                                                                                             // Temporary register
    if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // The non-interrupt mode
        temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)) ;
        pGPIO_Handle->pGPIOx->MODER &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) ;                      // Clear 2 bit fields
        pGPIO_Handle->pGPIOx->MODER |= temp ;                                                                       // Set
    }
    else {
        // TODO: later; this is for interrupt mode
    }

    // 2. Configure the speed
    temp = 0 ;
    temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) ;
    pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) ;                        // Clear
    pGPIO_Handle->pGPIOx->OSPEEDR |= temp ;                                                                         // Set

    // 3. Configure the PUPD settings
    temp = 0 ;
    temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) ;
    pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) ;                          // Clear
    pGPIO_Handle->pGPIOx->PUPDR |= temp ;                                                                           // Set

    // 4. Configure the op type
    temp = 0 ;
    temp = pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdCtrl << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber ;
    pGPIO_Handle->pGPIOx->OTYPER &= ~(0x3 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber) ;                         // Clear
    pGPIO_Handle->pGPIOx->OTYPER |= temp ;                                                                          // Set

    // 5. Configure the alt functionality
    if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FUN) {
        // Configure the alt function registers
        uint32_t temp1 ;
        uint32_t temp2 ;

        temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8 ;
        temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8 ;

        pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));                                                  // Clear 4 bit fields
        pGPIO_Handle->pGPIOx->AFR[temp1] |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));       // Set
    }
}

/****************************************************************************************************
 * @fn                          GPIO_DeInit
 * 
 * @brief                       Deinitializes operation of a GPIO pin
 * 
 * @param[in] pGPIOx            Base address of a GPIO
 * 
 * @return                      none
 * 
 * @note                        none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

} 

/*
 * Data read and write
 */

/****************************************************************************************************
 * @fn                          GPIOReadFromInputPin
 * 
 * @brief                       Reads the value from a GPIO port pin
 * 
 * @param[in] pGPIOx            Base address of the GPIO port
 * @param[in] pinNumber         GPIO port pin number
 * 
 * @return                      uint8_t
 * 
 * @note                        none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

    return 1 ;
}

/****************************************************************************************************
 * @fn                          GPIO_ReadFromInputPort
 * 
 * @brief                       Reads the value from a GPIO port
 * 
 * @param[in] pGPIOx            Base address of the GPIO port
 * 
 * @return                      uint16_t
 * 
 * @note                        none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {


    return 1 ;
}

/****************************************************************************************************
 * @fn                          GPIO_WritToOutputPin
 * 
 * @brief                       Writes a value to a GPIO port pin
 * 
 * @param[in] pGPIOx            Base address of GPIO port
 * @param[in] pinNumber         GPIO port pin number
 * @param[in] value             Value to be written to GPIO port pin
 * 
 * @return                      none
 * 
 * @note                        none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {

}

/****************************************************************************************************
 * @fn                          GPIO_WriteToOutputPort
 * 
 * @brief                       Write a value to a GPIO port
 * 
 * @param[in] pGPIOx            Base address of GPIO port
 * @param[in] value             Value to be written to GPIO port
 * 
 * @return                      none
 * 
 * @note                        none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

}

/****************************************************************************************************
 * @fn                          GPIO_ToggleOutputPin
 * 
 * @brief                       Toggles a GPIO port pin as ENABLED or DISABLED
 * 
 * @param[in] pGPIOx            Base address of GPIO port
 * @param[in] pinNumber         Pin number of GPIO port
 * 
 * @return                      none
 * 
 * @note                        none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

}

/*
 * IRQ configuration and ISR handling
 */

/****************************************************************************************************
 * @fn                          GPIO_IRQConfig
 * 
 * @brief                       Configures the IRQ of a GPIO
 * 
 * @param[in] IRQNumber         IRQ number from NVIC table of ARM Cortex M4
 * @param[in] IRQPriority       IRQ priority from NVIC table of ARM Cortex M4
 * @param[in] EnorDi            ENABLE or DISABLE macros
 * 
 * @return                      none
 * 
 * @note                        none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}

/****************************************************************************************************
 * @fn                          GPIO_IRQHandler
 * 
 * @brief                       ISR for GPIO
 * 
 * @param[in] pinNumber         Pin number of GPIO
 * 
 * @return                      none
 * 
 * @note                        none
 */
void GPIO_IRQHandler(uint8_t pinNumber) {

}
