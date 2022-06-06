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

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {

}

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) {

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

} 

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx uint8_t pinNumber) {

}

/*
 * IRQ configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}

void GPIO_IRQHandler(uint8_t pinNumber) {

}
