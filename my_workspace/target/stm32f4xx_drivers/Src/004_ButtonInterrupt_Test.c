/**
 * @file 004_ButtonInterrupt_Test.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"
#include "string.h"

void config_led(GPIO_Handle_t *pGpioLed) ;
void config_btn(GPIO_Handle_t *pGpioBtn) ;
void init_led(GPIO_Handle_t *pGpioLed) ;
void init_btn(GPIO_Handle_t *pGpioBtn) ;

void delay(void) ;
void EXTI9_5_IRQHandler(void) ;

int main(void) {
    /*
     * Configure button and LED
     */
    // Define which peripherals to use
    GPIO_Handle_t GpioLed ;
    GPIO_Handle_t GpioBtn ;

    // Enable peripheral clock for GPIO
    GPIO_PeriClockControl(GPIOD, ENABLE) ;                  // GPIOD    - AHB1 - Used for button and LED

    // Initialize the GPIO peripherals to zero
    memset(&GpioLed, 0, sizeof(GpioLed)) ;
    memset(&GpioBtn, 0, sizeof(GpioBtn)) ;

    // Configure the LED settings
    GpioLed.pGPIOx = GPIOD ;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_12 ;
    GpioLed.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_OUTPUT ;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_LOW ;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD ;
    // Configure the button settings
    GpioLed.pGPIOx = GPIOD ;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_5 ;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_INPUT ;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_FAST ;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU ;

#ifdef later
    // Initialize the LED GPIO
    uint32_t temp ;
    temp = (GpioLed.GPIO_PinConfig.GPIO_PinMode << (2 * GpioLed.GPIO_PinConfig.GPIO_PinNumber)) ;
    GpioLed.pGPIOx->MODER &= ~(0x3 << 2 * (GpioLed.GPIO_PinConfig.GPIO_PinNumber)) ;                      // Clear 2 bit fields
    GpioLed.pGPIOx->MODER |= temp ;
    // TODO: Initialize the speed, pupd, and otyper registers for the LED!!! 
#endif /* later */

    // Initialize the LED GPIO
    init_led(&GpioLed) ;


    /***********************************************************
     * Configure interrupts
     **********************************************************/

    /*
     * Configure the GPIO port selection in SYSCFG_EXTICR
     */
    // Enable peripheral clock for SYSFG so we may use EXTI
    SYSCFG_PCLK_EN() ;

    // Set SYSCFG to connect the button EXTI line to GPIOD
    SYSCFG->EXTICR[GpioBtn.GPIO_PinConfig.GPIO_PinNumber / 4] &= ~(0xF << ((GpioBtn.GPIO_PinConfig.GPIO_PinNumber % 4) * 4)) ;
    SYSCFG->EXTICR[GpioBtn.GPIO_PinConfig.GPIO_PinNumber / 4] |= (0x3 << ((GpioBtn.GPIO_PinConfig.GPIO_PinNumber % 4) * 4)) ;   // Pin D is the 4th pin, thus 0x3

    // Set up the EXTI line as an interrupt
    EXTI->IMR |= (1 << GpioBtn.GPIO_PinConfig.GPIO_PinNumber) ;
    // Disable the rising edge trigger (button release)
    EXTI->RTSR &= ~(1 << GpioBtn.GPIO_PinConfig.GPIO_PinNumber) ;
    // Enable the falling edge trigger (button press)
    EXTI->FTSR |= (1 << GpioBtn.GPIO_PinConfig.GPIO_PinNumber) ;

    // Configure the priority of the button interrupt
    *(NVIC_PR_BASE_ADDR + (IRQ_NO_EXTI9_5 / 4)) &= ~(0xFF << ((8 * IRQ_NO_EXTI9_5 % 4) + (8 - NO_PRIORITY_BITS_IMPLEMENTED))) ;
    *(NVIC_PR_BASE_ADDR + (IRQ_NO_EXTI9_5 / 4)) |= (0xF << ((8 * IRQ_NO_EXTI9_5 % 4) + (8 - NO_PRIORITY_BITS_IMPLEMENTED))) ;

    // Configure the ISER for the IRQ number for pin 5
    *NVIC_ISER0 |= (1 << IRQ_NO_EXTI9_5) ;

    // Loop forever, waiting for interrupt
    while (1) ;

    return 0 ;
}


void init_led(GPIO_Handle_t *pGpioLed) {
    uint32_t temp ;
    temp = (pGpioLed->GPIO_PinConfig.GPIO_PinMode << (2 * pGpioLed->GPIO_PinConfig.GPIO_PinNumber)) ;
    pGpioLed->pGPIOx->MODER &= ~(0x3 << 2 * (pGpioLed->GPIO_PinConfig.GPIO_PinNumber)) ;                      // Clear 2 bit fields
    pGpioLed->pGPIOx->MODER |= temp ; 
}


void init_btn(GPIO_Handle_t *pGpioBtn) {
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
    // This will introduce ~200 ms delay when system clock is 16 MHz
    for (uint32_t i = 0 ; i < 500000/2 ; i++) ;
}

/****************************************************************************************************
 * @fn          EXTI9_5_IRQHandler
 * 
 * @brief       Handles interrupts for pins 5 through 9; toggles LED from falling edge button press
 * 
 * @return      none
 * 
 * @note        none
 * 
 */
void EXTI9_5_IRQHandler(void) {
    delay() ;
    GPIO_IRQHandler(GPIO_PIN_NO_5) ;
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12) ;
}
