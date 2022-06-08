/**
 * @file 004_ButtonInterrupt.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief External button connected to PD5 toggles an LED triggered by button press (interrupt)
 * @version 0.1
 * @date 2022-06-07
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
void EXTI9_5_IRQHandler(void) ;

int main(void) {
    GPIO_Handle_t GpioLed ;
    GPIO_Handle_t GpioBtn ;

    // Configure the LED to use PD12 with no PUPD with fast output speed
    GpioLed.pGPIOx = GPIOD ;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_12 ;
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
    // Configure the button to use PD5 with external PUPD and fast output speed
    GpioBtn.pGPIOx = GPIOD ;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_5 ;
    // Configure interrupt for falling edge
    GpioBtn.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_IT_TF ;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST ;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD ;
    // Enable GPIO clock and initialize it
    GPIO_PeriClockControl(GPIOB, ENABLE) ;
    GPIO_Init(&GpioBtn) ;

    // IRQ configurations
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15) ;
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE) ;
    
    while (1) ;

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

void EXTI9_5_IRQHandler(void) {
        GPIO_IRQHandler(GPIO_PIN_NO_5) ;
        GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12) ;
}
