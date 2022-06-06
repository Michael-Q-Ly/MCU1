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

#include <stdint.h>

/*
 * Generic macros
 */

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          DISABLE

/*
 * Bit shifts
 */

#define BIT0                    (0x1U << 0)
#define BIT1                    (0x1U << 1)
#define BIT2                    (0x1U << 2)
#define BIT3                    (0x1U << 3)
#define BIT4                    (0x1U << 4)
#define BIT5                    (0x1U << 5)
#define BIT6                    (0x1U << 6)
#define BIT7                    (0x1U << 7)
#define BIT8                    (0x1U << 8)
#define BIT9                    (0x1U << 9)
#define BIT10                   (0x1U << 10)
#define BIT11                   (0x1U << 11)
#define BIT12                   (0x1U << 12)
#define BIT13                   (0x1U << 13)
#define BIT14                   (0x1U << 14)
#define BIT15                   (0x1U << 15)
#define BIT16                   (0x1U << 16)
#define BIT17                   (0x1U << 17)
#define BIT18                   (0x1U << 18)
#define BIT19                   (0x1U << 19)
#define BIT20                   (0x1U << 20)
#define BIT21                   (0x1U << 21)
#define BIT22                   (0x1U << 22)
#define BIT23                   (0x1U << 23)
#define BIT24                   (0x1U << 24)
#define BIT25                   (0x1U << 25)
#define BIT26                   (0x1U << 26)
#define BIT27                   (0x1U << 27)
#define BIT28                   (0x1U << 28)
#define BIT29                   (0x1U << 29)
#define BIT30                   (0x1U << 30)
#define BIT31                   (0x1U << 31)

/*
 * Base addresses of FLASH and SRAM memories
 */

#define ROM                     0x1FFF0000UL
#define FLASH_BASE_ADDR         0x80000000UL
#define SRAM1_BASE_ADDR         0x20000000UL
#define SRAM1_SIZE              0x1C000U                                /*!< 112 kB */
#define SRAM2_BASE_ADDR         (SRAM1_BASE_ADDR + SRAM1_SIZE)
#define SRAM2_SIZE              0x4000U                                 /*!< 16 kB */
#define SRAM3_BASE_ADDR         (SRAM2_BASE_ADDR + SRAM2_SIZE)
#define SRAM3_SIZE              0x10000U                                /*!< 64 kB */
#define SRAM                    SRAM1_BASE_ADDR

 /*
  * AHBx and APBx bus peripheral base addresses
 */

#define PERIPH_BASE             0x40000000U                             /*!< GPIO peripheral base address */
#define APB1_PERIPH_BASE_ADDR   PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR   0x40010000U
#define AHB1_PERIPH_BASE_ADDR   0x40020000U
#define AHB2_PERIPH_BASE_ADDR   0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
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
 * Base address of RCC peripheral
 */

#define RCC_BASE_ADDR           (AHB1_PERIPH_BASE_ADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus:
 *      I2C1, I2C2, I2C3, SPI2, SPI3, USART2, USART3, UART4, UART5
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





/************************************ Peripheral Register Definition Structures ************************************/

/*
 * Note: Registers of a peripheral are specific to MCU
 * E.g.: The number of registers of SPI peripheral of STM32F4xxfamily of MCUs may be different (more or less)
 * compared to the number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your device RM
 */

typedef struct {
    volatile uint32_t  MODER ;                  /*!< GPIO port mode register,                                       Address offset: 0x00 */
    volatile uint32_t  OTYPER ;                 /*!< GPIO port output type register,                                Address offset: 0x04 */
    volatile uint32_t  OSPEEDR ;                /*!< GPIO port output speed register,                               Address offset: 0x08 */
    volatile uint32_t  PUPDR ;                  /*!< GPIO port pull-up/pull-down register,                          Address offset: 0x0C */
    volatile uint32_t  IDR ;                    /*!< GPIO port input data register,                                 Address offset: 0x10 */
    volatile uint32_t  ODR ;                    /*!< GPIO port output data register,                                Address offset: 0x14 */
    volatile uint32_t  BSRR ;                   /*!< GPIO port bit set/reset register,                              Address offset: 0x18 */
    volatile uint32_t  LCKR ;                   /*!< GPIO port configuration lock register,                         Address offset: 0x1C */
    volatile uint32_t  AFR[2] ;                 /*!< AFR[0]: GPIO alternate function low register, AF[1]: GPIO alternate function high register,                Address offset: 0x20 - 0x24 */
} GPIO_RegDef_t ;

typedef struct {
    volatile uint32_t CR ;                      /*!< RCC clock control register                                     Address offset: 0x00 */
    volatile uint32_t PLLCFGR ;                 /*!< RCC PLL configuration register                                 Address offset: 0x04 */
    volatile uint32_t CFGR ;                    /*!< RCC clock configuration register                               Address offset: 0x08 */
    volatile uint32_t CIR ;                     /*!< RCC clock interrupt register                                   Address offset: 0x0C */
    volatile uint32_t AHB1RSTR ;                /*!< RCC AHB1 peripheral reset register                             Address offset: 0x10 */
    volatile uint32_t AHB2RSTR ;                /*!< RCC AHB2 peripheral reset register                             Address offset: 0x14 */
    volatile uint32_t AHB3RSTR ;                /*!< RCC AHB3 peripheral reset register                             Address offset: 0x18 */
    uint32_t RESERVED0 ;                        /*!< Reserved byte */
    volatile uint32_t APB1RSTR ;                /*!< RCC APB1 peripheral reset register                             Address offset: 0x20 */
    volatile uint32_t APB2RSTR ;                /*!< RCC APB2 peripheral reset register                             Address offset: 0x24 */
    uint32_t RESERVED1[2] ;                     /*!< Reserved byte */
    volatile uint32_t AHB1ENR ;                 /*!< RCC AHB1 peripheral clock register                             Address offset: 0x30 */
    volatile uint32_t AHB2ENR ;                 /*!< RCC AHB2 peripheral clock enable register                      Address offset: 0x34 */
    volatile uint32_t AHB3ENR ;                 /*!< RCC AHB3 peripheral clock enable register                      Address offset: 0x38 */
    uint32_t RESERVED2 ;                        /*!< Reserved byte */
    volatile uint32_t APB1ENR ;                 /*!< RCC APB1 peripheral clock enable register                      Address offset: 0x40 */
    volatile uint32_t APB2ENR ;                 /*!< RCC APB2 peripheral clock enable register                      Address offset: 0x44 */
    uint32_t RESERVED3[2] ;                     /*!< Reserved byte */
    volatile uint32_t AHB1LPENR ;               /*!< RCC AHB1 peripheral clock enable in low power mode register    Address offset: 0x50 */
    volatile uint32_t AHB2LPENR ;               /*!< RCC AHB2 peripheral clock enable in low power mode register    Address offset: 0x54 */
    volatile uint32_t AHB3LPENR ;               /*!< RCC AHB3 peripheral clock enable in low power mode register    Address offset: 0x58 */
    uint32_t RESERVED4 ;                        /*!< Reserved byte */
    volatile uint32_t APB1LPENR ;               /*!< RCC APB1 peripheral clock enable in low power mode register    Address offset: 0x60 */
    volatile uint32_t APB2LPENR ;               /*!< RCC APB2 peripheral clock enabled in low power mode register   Address offset: 0x64 */
    uint32_t RESERVED5[2] ;                     /*!< Reserved byte */
    volatile uint32_t BDCR ;                    /*!< RCC Backup domain control register                             Address offset: 0x70 */
    volatile uint32_t CSR ;                     /*!< RCC clock control & status register                            Address offset: 0x74 */
    uint32_t RESERVED6[2] ;                     /*!< Reserved byte */
    volatile uint32_t SSCGR ;                   /*!< RCC spread spectrum clock generation register                  Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR ;              /*!< RCC PLLI2S configuration register                              Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR ;              /*!< RCC PLL configuration register                                 Address offset: 0x88 */
    volatile uint32_t DCKCFGR ;                 /*!< RCC Dedicated Clock Configuration Register                     Address offset: 0x8C */
#ifdef later
    volatile uint32_t CR ;                      /*!< RCC clock control register                                     Address offset: 0x00 */
    volatile uint32_t PLLCFGR ;                 /*!< RCC PLL configuration register                                 Address offset: 0x04 */
    volatile uint32_t CFGR ;                    /*!< RCC clock configuration register                               Address offset: 0x08 */
    volatile uint32_t CIR ;                     /*!< RCC clock interrupt register                                   Address offset: 0x0C */
    volatile uint32_t AHB1RSTR ;                /*!< RCC AHB1 peripheral reset register                             Address offset: 0x10 */
    volatile uint32_t AHB2RSTR ;                /*!< RCC AHB2 peripheral reset register                             Address offset: 0x14 */
    volatile uint32_t AHB3RSTR ;                /*!< RCC AHB3 peripheral reset register                             Address offset: 0x18 */
    uint32_t RESERVED0 ;                        /*!< Reserved byte */
    volatile uint32_t APB1RSTR ;                /*!< RCC APB1 peripheral reset register                             Address offset: 0x20 */
    volatile uint32_t APB2RSTR ;                /*!< RCC APB2 peripheral reset register                             Address offset: 0x24 */
    uint32_t RESERVED1 ;                        /*!< Reserved byte */
    uint32_t RESERVED2 ;                        /*!< Reserved byte */
    volatile uint32_t AHB1ENR ;                 /*!< RCC AHB1 peripheral clock register                             Address offset: 0x30 */
    volatile uint32_t AHB2ENR ;                 /*!< RCC AHB2 peripheral clock enable register                      Address offset: 0x34 */
    volatile uint32_t AHB3ENR ;                 /*!< RCC AHB3 peripheral clock enable register                      Address offset: 0x38 */
    uint32_t RESERVED3 ;                        /*!< Reserved byte */
    volatile uint32_t APB1ENR ;                 /*!< RCC APB1 peripheral clock enable register                      Address offset: 0x40 */
    volatile uint32_t APB2ENR ;                 /*!< RCC APB2 peripheral clock enable register                      Address offset: 0x44 */
    uint32_t RESERVED4 ;                        /*!< Reserved byte */
    uint32_t RESERVED5 ;                        /*!< Reserved byte */
    volatile uint32_t AHB1LPENR ;               /*!< RCC AHB1 peripheral clock enable in low power mode register    Address offset: 0x50 */
    volatile uint32_t AHB2LPENR ;               /*!< RCC AHB2 peripheral clock enable in low power mode register    Address offset: 0x54 */
    volatile uint32_t AHB3LPENR ;               /*!< RCC AHB3 peripheral clock enable in low power mode register    Address offset: 0x58 */
    uint32_t RESERVED6 ;                        /*!< Reserved byte */
    volatile uint32_t APB1LPENR ;               /*!< RCC APB1 peripheral clock enable in low power mode register    Address offset: 0x60 */
    volatile uint32_t APB2LPENR ;               /*!< RCC APB2 peripheral clock enabled in low power mode register   Address offset: 0x64 */
    uint32_t RESERVED7 ;                        /*!< Reserved byte */
    uint32_t RESERVED8 ;                        /*!< Reserved byte */
    volatile uint32_t BDCR ;                    /*!< RCC Backup domain control register                             Address offset: 0x70 */
    volatile uint32_t CSR ;                     /*!< RCC clock control & status register                            Address offset: 0x74 */
    uint32_t RESERVED9 ;                        /*!< Reserved byte */
    uint32_t RESERVED10 ;                       /*!< Reserved byte */
    volatile uint32_t SSCGR ;                   /*!< RCC spread spectrum clock generation register                  Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR ;              /*!< RCC PLLI2S configuration register                              Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR ;              /*!< RCC PLL configuration register                                 Address offset: 0x88 */
    volatile uint32_t DCKCFGR ;                 /*!< RCC Dedicated Clock Configuration Register                     Address offset: 0x8C */
#endif /* later */
} RCC_RegDef_t ;

/*
 * Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA                   ((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB                   ((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC                   ((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD                   ((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE                   ((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF                   ((GPIO_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG                   ((GPIO_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH                   ((GPIO_RegDef_t*) GPIOH_BASE_ADDR)
#define GPIOI                   ((GPIO_RegDef_t*) GPIOI_BASE_ADDR)
#define GPIOJ                   ((GPIO_RegDef_t*) GPIOJ_BASE_ADDR)
#define GPIOK                   ((GPIO_RegDef_t*) GPIOK_BASE_ADDR)

#define RCC                     ((RCC_RegDef_t*) RCC_BASE_ADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= BIT0)
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= BIT1)
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= BIT2)
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= BIT3)
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= BIT4)
#define GPIOF_PCLK_EN()         (RCC->AHB1ENR |= BIT5)
#define GPIOG_PCLK_EN()         (RCC->AHB1ENR |= BIT6)
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= BIT7)
#define GPIOI_PCLK_EN()         (RCC->AHB1ENR |= BIT8)
#define GPIOJ_PCLK_EN()         (RCC->AHB1ENR |= BIT9)
#define GPIOK_PCLK_EN()         (RCC->AHB1ENR |= BIT10)

/*
 * Clock enable macros for I2Cx peripherals
 */

// // TODO: Finish other I2C peripherals
#define I2C1_PCLK_EN()          (RCC->APB1ENR |= BIT21)
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= BIT22)
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= BIT23)

/*
 * Clock enable macros for SPIx peripherals
 */

// // TODO: Finish other SPI peripherals
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= BIT12)
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= BIT14)
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= BIT15)

/*
 *  Clock enable macros for USARTx peripherals
 */

// // TODO: Finish other USART peripherals
#define USART1_PCLK_EN()        (RCC->APB2ENR |= BIT4)
#define USART2_PCLK_EN()        (RCC->APB1ENR |= BIT17)
#define USART3_PCLK_EN()        (RCC->APB1ENR |= BIT18)
#define UART4_PCLK_EN()         (RCC->APB1ENR |= BIT19)
#define UART5_PCLK_EN()         (RCC->APB1ENR |= BIT20)
#define USART6_PCLK_EN()        (RCC->APB2ENR |= BIT5)

/*
 * Clock enable macros for SYSCFG peripheral
 */

// // TODO: SYSCFG macro
#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= BIT14)

/*
 * Clock Disable macros for GPIOx peripherals
 */

// // TODO: Finish GPIO macros for disable
#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~BIT0)
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~BIT1)
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~BIT2)
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~BIT3)
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~BIT4)
#define GPIOF_PCLK_DI()         (RCC->AHB1ENR &= ~BIT5)
#define GPIOG_PCLK_DI()         (RCC->AHB1ENR &= ~BIT6)
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~BIT7)
#define GPIOI_PCLK_DI()         (RCC->AHB1ENR &= ~BIT8)
#define GPIOJ_PCLK_DI()         (RCC->AHB1ENR &= ~BIT9)
#define GPIOK_PCLK_DI()         (RCC->AHB1ENR &= ~BIT10)

/*
 * Clock disable macros for I2Cx peripherals
 */

// // TODO: Finish other I2C peripherals for disable
#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~BIT21)
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~BIT22)
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~BIT23)

/*
 * Clock disable macros for SPIx peripherals
 */

// // TODO: Finish other SPI peripherals for disable
#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~BIT12)
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~BIT14)
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~BIT15)

/*
 *  Clock disable macros for USARTx peripherals
 */

// // TODO: Finish other USART peripherals for disable
#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~BIT4)
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~BIT17)
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~BIT18)
#define UART4_PCLK_DI()         (RCC->APB1ENR &= ~BIT19)
#define UART5_PCLK_DI()         (RCC->APB1ENR &= ~BIT20)
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~BIT5)

/*
 * Clock disable macros for SYSCFG peripheral
 */

// // TODO: SYSCFG macro
#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~BIT14)

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT0) ;   (RCC->AHB1RSTR &= ~BIT0) ;  } while(0)
#define GPIOB_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT1) ;   (RCC->AHB1RSTR &= ~BIT1) ;  } while(0)
#define GPIOC_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT2) ;   (RCC->AHB1RSTR &= ~BIT2) ;  } while(0)
#define GPIOD_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT3) ;   (RCC->AHB1RSTR &= ~BIT3) ;  } while(0)
#define GPIOE_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT4) ;   (RCC->AHB1RSTR &= ~BIT4) ;  } while(0)
#define GPIOF_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT5) ;   (RCC->AHB1RSTR &= ~BIT5) ;  } while(0)
#define GPIOG_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT6) ;   (RCC->AHB1RSTR &= ~BIT6) ;  } while(0)
#define GPIOH_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT7) ;   (RCC->AHB1RSTR &= ~BIT7) ;  } while(0)
#define GPIOI_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT8) ;   (RCC->AHB1RSTR &= ~BIT8) ;  } while(0)
#define GPIOJ_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT9) ;   (RCC->AHB1RSTR &= ~BIT9) ;  } while(0)
#define GPIOK_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT10) ;  (RCC->AHB1RSTR &= ~BIT10) ; } while(0)

#endif /* _STM32F29XX_H_ */
