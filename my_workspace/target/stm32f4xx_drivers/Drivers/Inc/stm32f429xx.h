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

/************************************ START: Processor-Specific Details ************************************/

/*
 * ARM Cortex Mx processor NVIC ISERx register addresses
 */

#define NVIC_ISER0              ((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1              ((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2              ((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3              ((volatile uint32_t*) 0xE000E10C)
#define NVIC_ISER4              ((volatile uint32_t*) 0xE000E110)
#define NVIC_ISER5              ((volatile uint32_t*) 0xE000E114)
#define NVIC_ISER6              ((volatile uint32_t*) 0xE000E118)
#define NVIC_ISER7              ((volatile uint32_t*) 0xE000E11C)

/*
 * ARM Cortex Mx processor NVIC ICERx register addresses
 */

#define NVIC_ICER0              ((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1              ((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2              ((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3              ((volatile uint32_t*) 0xE000E18C)
#define NVIC_ICER4              ((volatile uint32_t*) 0xE000E190)
#define NVIC_ICER5              ((volatile uint32_t*) 0xE000E194)
#define NVIC_ICER6              ((volatile uint32_t*) 0xE000E198)
#define NVIC_ICER7              ((volatile uint32_t*) 0xE000E19C)

/*
 * ARM Cortex Mx processor NVIC priority register address calculation
 */

#define NVIC_PR_BASE_ADDR       ((volatile uint32_t*) 0XE000E400)

#define NO_PRIORITY_BITS_IMPLEMENTED    4                               /*!< Upper 4 bits of interrupt priority register byte offset; lower 4 bytes are ignored */

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
 *
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
 *
 */

#define EXTI_BASE_ADDR          (APB2_PERIPH_BASE_ADDR + 0X3C00)

#define SPI1_BASE_ADDR          (APB2_PERIPH_BASE_ADDR + 0X3000)
#define SPI4_BASE_ADDR          (APB2_PERIPH_BASE_ADDR + 0x3400)
#define SPI5_BASE_ADDR          (APB2_PERIPH_BASE_ADDR + 0x5000)
#define SPI6_BASE_ADDR          (APB2_PERIPH_BASE_ADDR + 0x5400)

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

/**
 * @brief Peripheral register definition structure for GPIOs
 * 
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

/**
 * @brief Peripheral register definition structure for SPI communication
 * 
 */
typedef struct {
    volatile uint32_t CR1 ;                     /*!< SPI control register 1 (Not used in I2S Mode)                  Address offset: 0x00 */
    volatile uint32_t CR2 ;                     /*!< SPI control register 2                                         Address offset: 0x04 */
    volatile uint32_t SR ;                      /*!< SPI status register                                            Address offset: 0x08 */
    volatile uint32_t DR ;                      /*!< SPI data register                                              Address offset: 0x0C */
    volatile uint32_t CRCPR ;                   /*!< SPI CRC polynomial register                                    Address offset: 0x10 */
    volatile uint32_t RXCRCR ;                  /*!< SPI RX CRC register (Not used in I2S Mode)                     Address offset: 0x14 */
    volatile uint32_t TXCRCR ;                  /*!< SPI TX CRC register (Not used in I2S Mode)                     Address offset: 0x18 */
    volatile uint32_t I2SCFGR ;                 /*!< SPI_I2S configuration register                                 Address offset: 0x1C */
    volatile uint32_t I2SPR ;                   /*!< SPI_I2S prescaler register                                     Address offset: 0x20 */
} SPI_RegDef_t ;

/**
 * @brief Peripheral register definition structure for RCC
 * 
 */
typedef struct {
    volatile uint32_t CR ;                      /*!< RCC clock control register                                     Address offset: 0x00 */
    volatile uint32_t PLLCFGR ;                 /*!< RCC PLL configuration register                                 Address offset: 0x04 */
    volatile uint32_t CFGR ;                    /*!< RCC clock configuration register                               Address offset: 0x08 */
    volatile uint32_t CIR ;                     /*!< RCC clock interrupt register                                   Address offset: 0x0C */
    volatile uint32_t AHB1RSTR ;                /*!< RCC AHB1 peripheral reset register                             Address offset: 0x10 */
    volatile uint32_t AHB2RSTR ;                /*!< RCC AHB2 peripheral reset register                             Address offset: 0x14 */
    volatile uint32_t AHB3RSTR ;                /*!< RCC AHB3 peripheral reset register                             Address offset: 0x18 */
    uint32_t RESERVED0 ;                        /*!< Reserved byte                                                  Address offset: 0x1C */
    volatile uint32_t APB1RSTR ;                /*!< RCC APB1 peripheral reset register                             Address offset: 0x20 */
    volatile uint32_t APB2RSTR ;                /*!< RCC APB2 peripheral reset register                             Address offset: 0x24 */
    uint32_t RESERVED1[2] ;                     /*!< Reserved bytes                                                 Address offset: 0x28 - 0x2C*/
    volatile uint32_t AHB1ENR ;                 /*!< RCC AHB1 peripheral clock register                             Address offset: 0x30 */
    volatile uint32_t AHB2ENR ;                 /*!< RCC AHB2 peripheral clock enable register                      Address offset: 0x34 */
    volatile uint32_t AHB3ENR ;                 /*!< RCC AHB3 peripheral clock enable register                      Address offset: 0x38 */
    uint32_t RESERVED2 ;                        /*!< Reserved byte                                                  Address offset: 0x3C */
    volatile uint32_t APB1ENR ;                 /*!< RCC APB1 peripheral clock enable register                      Address offset: 0x40 */
    volatile uint32_t APB2ENR ;                 /*!< RCC APB2 peripheral clock enable register                      Address offset: 0x44 */
    uint32_t RESERVED3[2] ;                     /*!< Reserved bytes                                                 Address offset: 0x48 - 0x4C */
    volatile uint32_t AHB1LPENR ;               /*!< RCC AHB1 peripheral clock enable in low power mode register    Address offset: 0x50 */
    volatile uint32_t AHB2LPENR ;               /*!< RCC AHB2 peripheral clock enable in low power mode register    Address offset: 0x54 */
    volatile uint32_t AHB3LPENR ;               /*!< RCC AHB3 peripheral clock enable in low power mode register    Address offset: 0x58 */
    uint32_t RESERVED4 ;                        /*!< Reserved byte                                                  Address offset: 0x5C */
    volatile uint32_t APB1LPENR ;               /*!< RCC APB1 peripheral clock enable in low power mode register    Address offset: 0x60 */
    volatile uint32_t APB2LPENR ;               /*!< RCC APB2 peripheral clock enabled in low power mode register   Address offset: 0x64 */
    uint32_t RESERVED5[2] ;                     /*!< Reserved bytes                                                 Address offset: 0x68 - 0x6C */
    volatile uint32_t BDCR ;                    /*!< RCC Backup domain control register                             Address offset: 0x70 */
    volatile uint32_t CSR ;                     /*!< RCC clock control & status register                            Address offset: 0x74 */
    uint32_t RESERVED6[2] ;                     /*!< Reserved bytes                                                 Address offset: 0x78 - 0x7C */
    volatile uint32_t SSCGR ;                   /*!< RCC spread spectrum clock generation register                  Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR ;              /*!< RCC PLLI2S configuration register                              Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR ;              /*!< RCC PLL configuration register                                 Address offset: 0x88 */
    volatile uint32_t DCKCFGR ;                 /*!< RCC Dedicated Clock Configuration Register                     Address offset: 0x8C */
} RCC_RegDef_t ;

/**
 * @brief Peripheral register definition structure for EXTI
 * 
 */
typedef struct {
    volatile uint32_t IMR ;                     /*!< Interrupt mask register                                        Address offset: 0x00 */
    volatile uint32_t EMR ;                     /*!< Event mask register                                            Address offset: 0x04 */
    volatile uint32_t RTSR ;                    /*!< Rising trigger selection register                              Address offset: 0x08 */
    volatile uint32_t FTSR ;                    /*!< Falling trigger selection register                             Address offset: 0x0C */
    volatile uint32_t SWIER ;                   /*!< Software interrupt event register                              Address offset: 0x10 */
    volatile uint32_t PR ;                      /*!< Pending register                                               Address offset: 0x14 */
} EXTI_RegDef_t ;

/**
 * @brief Peripheral register definition structure for SYSCFG
 * 
 */
typedef struct {
    volatile uint32_t MEMRMP ;                  /*!< Memory remap register                                          Address offset: 0x00 */
    volatile uint32_t PMC ;                     /*!< Peripheral mode configuration register                         Address offset: 0x04 */
    volatile uint32_t EXTICR[4] ;               /*!< External interrupt configuration registers 1-4                 Address offset: 0x08 - 0x14 */
    uint32_t RESERVED0[2] ;                     /* Reserved bytes                                                   Address offset: 0x18 - 0x1C */
    volatile uint32_t CMPCR ;                   /*!< Compensation cell control register                             Address offset: 0x20 */
} SYSCFG_RegDef_t ;

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

#define SPI1                    ((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2                    ((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3                    ((SPI_RegDef_t*) SPI3_BASE_ADDR)
#define SPI4                    ((SPI_RegDef_t*) SPI4_BASE_ADDR)
#define SPI5                    ((SPI_RegDef_t*) SPI5_BASE_ADDR)
#define SPI6                    ((SPI_RegDef_t*) SPI6_BASE_ADDR)

#define RCC                     ((RCC_RegDef_t*) RCC_BASE_ADDR)

#define EXTI                    ((EXTI_RegDef_t*) EXTI_BASE_ADDR)

#define SYSCFG                  ((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

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

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= BIT21)
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= BIT22)
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= BIT23)

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= BIT12)
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= BIT13)
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= BIT14)
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= BIT15)
#define SPI5_PCLK_EN()          (RCC->APB2ENR |= BIT20)
#define SPI6_PCLK_EN()          (RCC->APB2ENR |= BIT21)

/*
 *  Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()        (RCC->APB2ENR |= BIT4)
#define USART2_PCLK_EN()        (RCC->APB1ENR |= BIT17)
#define USART3_PCLK_EN()        (RCC->APB1ENR |= BIT18)
#define UART4_PCLK_EN()         (RCC->APB1ENR |= BIT19)
#define UART5_PCLK_EN()         (RCC->APB1ENR |= BIT20)
#define USART6_PCLK_EN()        (RCC->APB2ENR |= BIT5)

/*
 * Clock enable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= BIT14)

/*
 * Clock Disable macros for GPIOx peripherals
 */

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

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~BIT21)
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~BIT22)
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~BIT23)

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~BIT12)
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~BIT13)
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~BIT14)
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~BIT15)
#define SPI5_PCLK_DI()          (RCC->APB2ENR &= ~BIT20)
#define SPI6_PCLK_DI()          (RCC->APB2ENR &= ~BIT21)

/*
 *  Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~BIT4)
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~BIT17)
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~BIT18)
#define UART4_PCLK_DI()         (RCC->APB1ENR &= ~BIT19)
#define UART5_PCLK_DI()         (RCC->APB1ENR &= ~BIT20)
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~BIT5)

/*
 * Clock disable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~BIT14)

/*
 * Generic macros
 */

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          DISABLE
#define FLAG_SET                SET
#define FLAG_RESET              RESET

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
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT0) ;   (RCC->AHB1RSTR &= ~BIT0) ;      } while (0)
#define GPIOB_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT1) ;   (RCC->AHB1RSTR &= ~BIT1) ;      } while (0)
#define GPIOC_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT2) ;   (RCC->AHB1RSTR &= ~BIT2) ;      } while (0)
#define GPIOD_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT3) ;   (RCC->AHB1RSTR &= ~BIT3) ;      } while (0)
#define GPIOE_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT4) ;   (RCC->AHB1RSTR &= ~BIT4) ;      } while (0)
#define GPIOF_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT5) ;   (RCC->AHB1RSTR &= ~BIT5) ;      } while (0)
#define GPIOG_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT6) ;   (RCC->AHB1RSTR &= ~BIT6) ;      } while (0)
#define GPIOH_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT7) ;   (RCC->AHB1RSTR &= ~BIT7) ;      } while (0)
#define GPIOI_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT8) ;   (RCC->AHB1RSTR &= ~BIT8) ;      } while (0)
#define GPIOJ_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT9) ;   (RCC->AHB1RSTR &= ~BIT9) ;      } while (0)
#define GPIOK_REG_RESET()       do {    (RCC->AHB1RSTR |= BIT10) ;  (RCC->AHB1RSTR &= ~BIT10) ;     } while (0)

/*
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASE_ADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
                                    (x == GPIOB) ? 1 :\
                                    (x == GPIOC) ? 2 :\
                                    (x == GPIOD) ? 3 :\
                                    (x == GPIOE) ? 4 :\
                                    (x == GPIOF) ? 5 :\
                                    (x == GPIOG) ? 6 :\
                                    (x == GPIOH) ? 7 :\
                                    (x == GPIOI) ? 8 :\
                                    (x == GPIOJ) ? 9 :\
                                    (x == GPIOK) ? 10 : 0)      /* PK[15:12] Not used for EXTICR3 and EXTICR4 */

/*
 * Macros to reset SPIx peripherals
 */

#define SPI1_REG_RESET()        do {    (RCC->APB2RSTR |= BIT12) ;  (RCC->APB2RSTR &= ~BIT12) ;     } while (0)
#define SPI2_REG_RESET()        do {    (RCC->APB1RSTR |= BIT14) ;  (RCC->APB1RSTR &= ~BIT14) ;     } while (0)
#define SPI3_REG_RESET()        do {    (RCC->APB1RSTR |= BIT15) ;  (RCC->APB1RSTR &= ~BIT15) ;     } while (0)
#define SPI4_REG_RESET()        do {    (RCC->APB2RSTR |= BIT13) ;  (RCC->APB2RSTR &= ~BIT13) ;     } while (0)
#define SPI5_REG_RESET()        do {    (RCC->APB2RSTR |= BIT20) ;  (RCC->APB2RSTR &= ~BIT20) ;     } while (0)
#define SPI6_REG_RESET()        do {    (RCC->APB2RSTR |= BIT20) ;  (RCC->APB2RSTR &= ~BIT20) ;     } while (0)

/*
 * IRQ (Interrupt Request) Numbers of STM32F429xx MCU
 * NOTE: Update these macros with valid values according to your MCU
 */

// TODO: You may complete this list for other peripherals

#define IRQ_NO_EXTI0                6
#define IRQ_NO_EXTI1                7
#define IRQ_NO_EXTI2                8
#define IRQ_NO_EXTI3                9
#define IRQ_NO_EXTI4                10
#define IRQ_NO_EXTI9_5              23
#define IRQ_NO_EXTI5_10             40

/*
 * Macros for IRQ priority levels
 */

#define NVIC_IRQ_PRI0               0
#define NVIC_IRQ_PRI15              15

/*
 * ISER0-ISER7 bit boundaries
 */

#define SIZE_OF_ISERx_REG           32
#define START_BIT_OF_ISER           0

#define START_BIT_OF_ISER0          START_BIT_OF_ISER
#define END_BIT_OF_ISER0            (START_BIT_OF_ISER0 + SIZE_OF_ISERx_REG - 1)
#define START_BIT_OF_ISER1          (START_BIT_OF_ISER + (1 * SIZE_OF_ISERx_REG))
#define END_BIT_OF_ISER1            (START_BIT_OF_ISER1 + SIZE_OF_ISERx_REG - 1)
#define START_BIT_OF_ISER2          (START_BIT_OF_ISER + (3 * SIZE_OF_ISERx_REG))
#define END_BIT_OF_ISER2            (START_BIT_OF_ISER2 + SIZE_OF_ISERx_REG - 1)
#define START_BIT_OF_ISER3          (START_BIT_OF_ISER + (4 * SIZE_OF_ISERx_REG))
#define END_BIT_OF_ISER3            (START_BIT_OF_ISER3 + SIZE_OF_ISERx_REG - 1)
#define START_BIT_OF_ISER4          (START_BIT_OF_ISER + (5 * SIZE_OF_ISERx_REG))
#define END_BIT_OF_ISER4            (START_BIT_OF_ISER4 + SIZE_OF_ISERx_REG - 1)
#define START_BIT_OF_ISER5          (START_BIT_OF_ISER + (6 * SIZE_OF_ISERx_REG))
#define END_BIT_OF_ISER5            (START_BIT_OF_ISER5 + SIZE_OF_ISERx_REG - 1)
#define START_BIT_OF_ISER6          (START_BIT_OF_ISER + (7 * SIZE_OF_ISERx_REG))
#define END_BIT_OF_ISER6            (START_BIT_OF_ISER6 + SIZE_OF_ISERx_REG - 1)
#define START_BIT_OF_ISER7          (START_BIT_OF_ISER + (8 * SIZE_OF_ISERx_REG))
#define END_BIT_OF_ISER7            (START_BIT_OF_ISER7 + SIZE_OF_ISERx_REG - 1)

/*
 * ICER0-ICER7 bit boundaries
 */

#define SIZE_OF_ICERx_REG           32
#define START_BIT_OF_ICER           0

#define START_BIT_OF_ICER0          START_BIT_OF_ICER
#define END_BIT_OF_ICER0            (START_BIT_OF_ICER0 + SIZE_OF_ICERx_REG - 1)
#define START_BIT_OF_ICER1          (START_BIT_OF_ICER + (1 * SIZE_OF_ICERx_REG))
#define END_BIT_OF_ICER1            (START_BIT_OF_ICER1 + SIZE_OF_ICERx_REG - 1)
#define START_BIT_OF_ICER2          (START_BIT_OF_ICER + (3 * SIZE_OF_ICERx_REG))
#define END_BIT_OF_ICER2            (START_BIT_OF_ICER2 + SIZE_OF_ICERx_REG - 1)
#define START_BIT_OF_ICER3          (START_BIT_OF_ICER + (4 * SIZE_OF_ICERx_REG))
#define END_BIT_OF_ICER3            (START_BIT_OF_ICER3 + SIZE_OF_ICERx_REG - 1)
#define START_BIT_OF_ICER4          (START_BIT_OF_ICER + (5 * SIZE_OF_ICERx_REG))
#define END_BIT_OF_ICER4            (START_BIT_OF_ICER4 + SIZE_OF_ICERx_REG - 1)
#define START_BIT_OF_ICER5          (START_BIT_OF_ICER + (6 * SIZE_OF_ICERx_REG))
#define END_BIT_OF_ICER5            (START_BIT_OF_ICER5 + SIZE_OF_ICERx_REG - 1)
#define START_BIT_OF_ICER6          (START_BIT_OF_ICER + (7 * SIZE_OF_ICERx_REG))
#define END_BIT_OF_ICER6            (START_BIT_OF_ICER6 + SIZE_OF_ICERx_REG - 1)
#define START_BIT_OF_ICER7          (START_BIT_OF_ICER + (8 * SIZE_OF_ICERx_REG))
#define END_BIT_OF_ICER7            (START_BIT_OF_ICER7 + SIZE_OF_ICERx_REG - 1)


/****************************************************************************************************
 *                            Bit position definitions of SPI peripheral                            *
 ***************************************************************************************************/

/*
 * Bit position definitions of SPI_CR1
 */

#define SPI_CR1_CPHA                0           /*!< Clock phase >*/
#define SPI_CR1_CPOL                1           /*!< Clock polarity >*/
#define SPI_CR1_MSTR                2           /*!< Controller (master) selection >*/
#define SPI_CR1_BR                  3           /*!< Baud rate control >*/
#define SPI_CR1_SPE                 6           /*!< SPI enable >*/
#define SPI_CR1_LSBFIRST            7           /*!< Frame Format >*/
#define SPI_CR1_SSI                 8           /*!< Internal peripheral (slave) select >*/
#define SPI_CR1_SSM                 9           /*!< Software peripheral (slave) management >*/
#define SPI_CR1_RXONLY              10          /*!< Receive only >*/
#define SPI_CR1_DFF                 11          /*!< Data frame format >*/
#define SPI_CR1_CRCNEXT             12          /*!< CRC transfer next >*/
#define SPI_CR1_CRCEN               13          /*!< Hardware CRC calculation enable >*/
#define SPI_CR1_BIDIOE              14          /*!< Output enable in bidirectional mode >*/
#define SPI_CR1_BIDIMODE            15          /*!< Bidirectional data mode enable >*/

/*
 * Bit position definitions of SPI_CR2
 */

#define SPI_CR2_RXDMAEN             0           /*!< Rx buffer DMA enable >*/
#define SPI_CR2_TXDMAEN             1           /*!< Tx buffer DMA enable >*/
#define SPI_CR2_SSOE                2           /*!< SS output enable >*/
#define SPI_CR2_FRF                 4           /*!< Frame format >*/
#define SPI_CR2_ERRIE               5           /*!< Error interrupt enable >*/
#define SPI_CR2_RXNEIE              6           /*!< RX buffer not empty interrupt enable >*/
#define SPI_CR2_TXEIE               7           /*!< Tx buffer empty interrupt enable >*/

/*
 * Bit position definitions of SPI_SR
 */

#define SPI_SR_RXNE                 0           /*!< Receive buffer not empty >*/
#define SPI_SR_TXE                  1           /*!< Transmit buffer empty >*/
#define SPI_SR_CHSIDE               2           /*!< Channel side >*/
#define SPI_SR_UDR                  3           /*!< Underrun flag >*/
#define SPI_SR_CRCERR               4           /*!< CRC error flag >*/
#define SPI_SR_MODF                 5           /*!< Mode fault >*/
#define SPI_SR_OVR                  6           /*!< Overrun flag >*/
#define SPI_SR_BSY                  7           /*!< Busy flag >*/
#define SPI_SR_FRE                  8           /*!< Frame format error >*/

#endif /* _STM32F29XX_H_ */
