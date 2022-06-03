/**
 * @file main.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-03
 * 
 * @copyright Copyright (c) 2022
 * @
 */
#include "main.h"
#include <stdio.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
    #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// Global shared variable between main code and ISR
button_t g_button ;

uint32_t volatile *pEXTIPendReg         = (uint32_t*) (EXTI_BASE_ADDRESS + EXTI_PEND_REG_OFFSET) ;
uint32_t volatile *pClkCtrlRegAhb1      = (uint32_t*) (RCC_BASE_ADDRESS + RCC_AHB1ENR_REG_OFFSET) ;
uint32_t volatile *pClkCtrlRegApb2      = (uint32_t*) (RCC_BASE_ADDRESS + RCC_APB2ENR_REG_OFFSET) ;
uint32_t volatile *pGPIOAModeReg        = (uint32_t*) (GPIOA_BASE_ADDR + GPIOA_PORT_MODE_REG_OFFSET) ;
uint32_t volatile *pEXTIMaskReg         = (uint32_t*) (EXTI_BASE_ADDRESS + EXTI_MASK_REG_OFFSET) ;
uint32_t volatile *pEXTIEdgeCtrlReg     = (uint32_t*) (EXTI_BASE_ADDRESS + EXTI_EDGE_CTRL_REG_OFFSET) ;
uint32_t volatile *pNVICIRQEnReg        = (uint32_t*) (NVIC_IRQ_EN_REG) ;

void button_init(void) ;
void EXTI0_IRQHandler(void) ;

int main(void) {
    button_init() ;
    g_button.pressed = 0 ;
    g_button.press_count = 0 ;

    while(1) {
        // Disable interrupt
        *pEXTIMaskReg &= ~(BIT_0) ;

        if (g_button.pressed) {
            // Some delay until button debouncing gets over
            for (uint32_t volatile i = 0 ; i < 500000 ; i++) ;
            g_button.press_count++ ;
            printf("Button is pressed: %lu\n", g_button.press_count) ;
            g_button.pressed = 0 ;
        }

        // Enable interrupt
        *pEXTIMaskReg |= BIT_0 ;
    }
}


/******************************************** Function Definitions ********************************************/

/**
 * @brief Configures the user wake-up button connected to PA0 as an interrupt-enabled pin
 */
void button_init(void) {
    /* GPIOA clock enable */
    *pClkCtrlRegAhb1 |= BIT_0 ;

    /* syscfg clock enable */
    *pClkCtrlRegApb2 |= BIT_14 ;

    /* Edge detection configuration */
    *pEXTIEdgeCtrlReg |= BIT_0 ;

    /* EXTI interrupt enable */
    *pEXTIMaskReg |= BIT_0 ;

    /* NVIC IRQ enable */
    *pNVICIRQEnReg |= BIT_6 ;
}

/**
 * @brief Interrupt handler for user wake-up button
 */
void EXTI0_IRQHandler(void) {
    // Make this flag SET if button is pressed
    g_button.pressed = 1 ;

    // Clearing the EXTI pending bit
    *pEXTIPendReg |= BIT_0 ;
}
