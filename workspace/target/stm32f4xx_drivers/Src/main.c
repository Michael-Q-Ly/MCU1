/**
 * @file main.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"
#include <stdint.h>

void EXTI0_IRQHandler(void) ;

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {
    return 0 ;
}

/****************************************************************************************************
 * @fn              EXTI0_IRQHandler
 * @brief           Handles interrupt
 * 
 * @return          none
 * 
 * @note            Overrides the weak function from the startup file
 * 
 */
void EXTI0_IRQHandler(void) {
    // Handle the interrupt by sending the pin number to the GPIO IRQ handler function
    GPIO_IRQHandler(0) ;
}