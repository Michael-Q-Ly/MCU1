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
#endif /* _STM32F29XX_H_ */
