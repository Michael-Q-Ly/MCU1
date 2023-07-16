/**
 * @file 002_LedButton.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Toggles the on-board LED wheenver the on-board button is pressed
 * @version 0.1
 * @date 2022-06-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"

#define HIGH                1
#define LOW                 0
#define BTN_PRESSED         HIGH

void delay(void) ;

int main(void) {
    GPIO_Handle_t GpioLed ;
    GPIO_Handle_t GpioBtn ;

    // Configure the LED to use PG13 with no PUPD with fast output speed
    GpioLed.pGPIOx = GPIOG ;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_13 ;
    GpioLed.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_OUTPUT ;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST ;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType   = GPIO_OP_TYPE_PP ;                 // Push-Pull
    // GpioLed.GPIO_PinConfig.GPIO_PinOPType   = GPIO_OP_TYPE_OD ;                 // Open Drain - Requires a 470 Ohm external resistor to pull up
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD ;

    GPIO_PeriClockControl(GPIOG, ENABLE) ;
    GPIO_Init(&GpioLed) ;

    // Configure the button to use PA0 with no PUPD with fast output speed
    GpioBtn.pGPIOx = GPIOA ;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_0 ;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_INPUT ;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST ;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD ;

    GPIO_PeriClockControl(GPIOA, ENABLE) ;
    GPIO_Init(&GpioBtn) ;
    
    while (1) {
        if (GPIO_ReadFromInputPin(GpioBtn.pGPIOx, GpioBtn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED) {
            // Debounce
            delay() ;
            GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber) ;
        }
    }

    return 0 ;
}

/****************************************************************************************************
 * @fn          delay
 * 
 * @brief       Software delay
 * 
 * @return      none
 * 
 * @note        none
 * 
 */
void delay(void) {
    for (uint32_t i = 0 ; i < 500000/2 ; i++) ;
}
