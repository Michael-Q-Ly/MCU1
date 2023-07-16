/**
 * @file main.h
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _MAIN_H_
#define _MAIN_H_
#include <stdint.h>

// External Interrupts
#define EXTI_BASE_ADDRESS                   0x40013C00UL
#define EXTI_PEND_REG_OFFSET                0x14
#define EXTI_MASK_REG_OFFSET                0x00
#define EXTI_EDGE_CTRL_REG_OFFSET           0X08
// NVIC
#define NVIC_IRQ_EN_REG                     0xE000E100          // Found in ARM Cortex M4 User Guide
// RCC
#define RCC_BASE_ADDRESS                    0x40023800UL
#define RCC_AHB1ENR_REG_OFFSET              0x30
#define RCC_APB2ENR_REG_OFFSET              0x44
// GPIO
#define GPIOA_BASE_ADDR                     0x40020000UL
#define GPIOA_PORT_MODE_REG_OFFSET          0X00UL
#define GPIOA_ALT_FUN_HIGH_REG_OFFSET       0x24UL
// Bit shifts
#define BIT_0                               (1UL << 0)
#define BIT_6                               (1UL << 6)
#define BIT_8                               (1UL << 8)
#define BIT_14                              (1UL << 14)
#define BIT_25                              (1UL << 25)
#define BIT_26                              (1UL << 26)

/**
 * Interrupt-enabled button
 * 
 */
typedef struct {
    uint8_t volatile pressed ;              /**<Determins if user pressed button; detects rising edges */
    uint32_t press_count ;                  /**<Determines number of times user has pressed button */
} button_t ;

#endif /* _MAIN_H_ */