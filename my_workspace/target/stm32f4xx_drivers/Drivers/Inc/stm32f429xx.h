/**
 * @file stm32f429xx.h
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _STM32F29XX_H_
#define _STM32F29XX_H_

/*
 * Base addresses of FLASH and SRAM memories
 */

#define ROM                     0x1FFF0000UL
#define FLASH_BASE_ADDR         0x80000000UL
#define SRAM1_BASE_ADDR         0x20000000UL
#define SRAM1_SIZE              0x1C000U                                /**<112 kB */
#define SRAM2_BASE_ADDR         (SRAM1_BASE_ADDR + SRAM1_SIZE)
#define SRAM2_SIZE              0x4000U                                 /* 16 kB */
#define SRAM3_BASE_ADDR         (SRAM2_BASE_ADDR + SRAM2_SIZE)
#define SRAM3_SIZE              0x10000U                                /* 64 kB */
#define SRAM                    SRAM1_BASE_ADDR

/*
 * AHBx and APBx bus peripheral base addresses
 */

#define PERIPH_BASE             0x40000000U
#define APB1_PERIPH_BASE_ADDR   PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR   0x40010000U
#define AHB1_PERIPH_BASE_ADDR   0x40020000U
#define AHB2_PERIPH_BASE_ADDR   0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 */

#define GPIOA_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x2000)
#define GPIOJ_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x2400)
#define GPIOK_BASE_ADDR         (AHB1_PERIPH_BASE_ADDR + 0x2800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus:
 *      I2C1, I2C2, I2C3, SPI2, SPI3, USART2, USART3M UART4, UART5
 * TODO: Complete for all other peripherals
 */

#define I2C1_BASE_ADDR          (APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR          (APB1_PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR          (APB1_PERIPH_BASE_ADDR + 0x5C00)

#define SPI2_BASE_ADDR          (APB1_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR          (APB1_PERIPH_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR        (APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR        (APB1_PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR         (APB1_PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR         (APB1_PERIPH_BASE_ADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus:
 *      SPI1, USART1, USART6, EXTI, SYSCFG
 * TODO: Complete for all other peripherals
 */

#define EXTI_BASE_ADDR          (APB2_PERIPH_BASE_ADDR + 0X3C00)
#define SPI1_BASE_ADDR          (APB2_PERIPH_BASE_ADDR + 0X3000)
#define SYSCFG_BASE_ADDR        (APB2_PERIPH_BASE_ADDR + 0X3800)
#define USART1_BASE_ADDR        (APB2_PERIPH_BASE_ADDR + 0X1000)
#define USART6_BASE_ADDR        (APB2_PERIPH_BASE_ADDR + 0X1400)

#endif /* _STM32F29XX_H_ */
