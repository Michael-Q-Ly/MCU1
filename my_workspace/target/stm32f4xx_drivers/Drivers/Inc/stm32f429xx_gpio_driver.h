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

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) ;

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) ;
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) ;

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) ;
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) ;
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t, pinNumber, uint8_t value) ;
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) ;
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx uint8_t pinNumber) ;

/*
 * IRQ configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) ;
void GPIO_IRQHandler(uint8_t pinNumber) ;

#endif /* _STM32F429XX_H_ */
