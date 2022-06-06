/**
 * @file stm32f429xx_gpio_driver.h
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _STM32F429XX_H_
#define _STM32F429XX_H_

#include "stm32f429xx.h"

/*
 * This holds the configurable parts of GPIOs
 */
typedef struct {
    uint8_t GPIO_PinNumber ;
    uint8_t GPIO_PinMode ;
    uint8_t GPIO_PinSpeed ;
    uint8_t GPIO_PinPuPdCtrl ;
    uint8_t GPIO_PinOPType ;
    uint8_t GPIO_PinAltFunMode ;
} GPIO_PinConfig_t ;

/*
 * This is a handle for a GPIO pin
 */

typedef struct {
    // Pointer to hold the base address of the GPIO peripheral
    GPIO_RegDef_t *pGPIOx ;                                         /*!< This holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig ;                               /*!< This holds GPIO pin configuration settings */
} GPIO_Handle_t ;




/****************************************************************************************************
 *                                   APIs supported by this driver                                  *
 *                For more information about the APIs, check the function definitions               *
 ***************************************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(void) ;

/*
 * Init and De-init
 */

void GPIO_Init(void) ;
void GPIO_DeInit(void) ;

/*
 * Data read and write
 */
void GPIO_ReadFromInputPin(void) ;
void GPIO_ReadFromInputPort(void) ;
void GPIO_WriteToOutputPin(void) ;
void GPIO_WriteToOutputPort(void) ;
void GPIO_ToggleOutputPin(void) ;

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(void) ;
void GPIO_IRQHandler(void) ;

#endif /* _STM32F429XX_H_ */
