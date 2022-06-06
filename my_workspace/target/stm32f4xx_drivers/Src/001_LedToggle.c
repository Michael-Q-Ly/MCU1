/**
 * @file 001_LedToggle.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Toggles the on-board LED with some delay between each blink
 *          Case 1: Use push-pull configuration for output pin
 *          Case 2: Use open drain configuration for the output pin
 * @version 0.1
 * @date 2022-06-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"

void delay(void) ;

int main(void) {
    GPIO_PeriClockControl(GPIOG, ENABLE) ;
    uint32_t volatile *pAhb1ClkCtrlReg = (uint32_t*)(0x40023830) ;
    *pAhb1ClkCtrlReg |= BIT6 ;
#ifdef later
    GPIO_Handle_t GpioLed ;

    GpioLed.pGPIOx = GPIOG ;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_13 ;
    GpioLed.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_OUTPUT ;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST ;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType   = GPIO_OP_TYPE_PP ;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD ;

    GPIO_PeriClockControl(GPIOG, ENABLE) ;
    GPIO_Init(&GpioLed) ;

    while (1) {
        GPIO_ToggleOutputPin(GPIOG, GPIO_PIN_NO_13) ;
        delay() ;
    }
#endif /* later */

    return 0 ;
}

void delay(void) {
    for (uint32_t i = 0 ; i < 500000 ; i++) ;
}
