/**
 * @file 003_ExternalLedButton.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief  Uses an external button attach to PB12 to toggle an LED attached to PA14 when the button is pressed
 *         Be sure to use a pullup or pulldown resistor as well as a current-limiting resistor     
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
#define BTN_PRESSED         LOW

void delay(void) ;

int main(void) {
    GPIO_Handle_t GpioLed ;
    GPIO_Handle_t GpioBtn ;

    // Configure the LED to use PA8 with no PUPD with fast output speed
    GpioLed.pGPIOx = GPIOA ;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_8 ;
    GpioLed.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_OUTPUT ;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST ;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType   = GPIO_OP_TYPE_PP ;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD ;
    // Enable GPIO clock and initialize it
    GPIO_PeriClockControl(GPIOA, ENABLE) ;
    GPIO_Init(&GpioLed) ;

    /*
     * Note: The button uses a pull-up resistor and is active LOW
     */
    // Configure the button to use PB12 with external PUPD and fast output speed
    GpioBtn.pGPIOx = GPIOB ;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_12 ;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_INPUT ;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST ;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD ;
    // Enable GPIO clock and initialize it
    GPIO_PeriClockControl(GPIOB, ENABLE) ;
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
