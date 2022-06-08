/**
 * @file stm32f429xx_gpio_driver.h
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief 
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _STM32F429XX_H_
#define _STM32F429XX_H_

#include "stm32f429xx.h"

/*
 * This structure holds the configurable parts of GPIOs
 */
typedef struct {
    uint8_t GPIO_PinNumber ;                                        /*!< Possible values from @GPIO_PIN_NUMBERS >*/
    uint8_t GPIO_PinMode ;                                          /*!< Possible values from @GPIO_PIN_MODES >*/
    uint8_t GPIO_PinSpeed ;                                         /*!< Possible values from @GPIO_SPEED >*/
    uint8_t GPIO_PinPuPdCtrl ;                                      /*!< Possible values from @GPIO_PUPD >*/
    uint8_t GPIO_PinOPType ;                                        /*!< Possible values from @GPIO_OP_TYPE >*/
    uint8_t GPIO_PinAltFunMode ;
} GPIO_PinConfig_t ;

/*
 * This is a handle structure for a GPIO pin
 */
typedef struct {
    // Pointer to hold the base address of the GPIO peripheral
    GPIO_RegDef_t *pGPIOx ;                                         /*!< This holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig ;                               /*!< This holds GPIO pin configuration settings */
} GPIO_Handle_t ;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

/*
 * @GPIO_PIN_MODES
 * GPIO pin configuration: Port Mode
 * 0 through 2 are non-interrupt, 3-6 are interrupt
 */

#define GPIO_MODE_INPUT         0                                   /*!< GPIO input mode */
#define GPIO_MODE_OUTPUT        1                                   /*!< GPIO output mode */
#define GPIO_MODE_ALT_FUN       2                                   /*!< GPIO alternate function mode */
#define GPIO_MODE_ANALOG        3                                   /*!< GPIO analog mode */
#define GPIO_MODE_IT_TF         4                                   /*!< GPIO input falling edge trigger */
#define GPIO_MODE_IT_TR         5                                   /*!< GPIO input rising edge trigger */
#define GPIO_MODE_IT_TRF        6                                   /*!< GPIO input rising and falling edge trigger */

/*
 * @GPIO_OP_TYPE
 * GPIO pin configuration: Output Type
 */

#define GPIO_OP_TYPE_PP         0                                   /*!< Push-pull output data type */
#define GPIO_OP_TYPE_OD         1                                   /*!< Open drain output data type */

/*
 * @GPIO_SPEED
 * GPIO pin configuration: Output Speed
 */

#define GPIO_SPEED_LOW          0                                   /*!< GPIO low output speed */
#define GPIO_SPEED_MEDIUM       1                                   /*!< GPIO medium output speed */
#define GPIO_SPEED_FAST         2                                   /*!< GPIO high output speed */
#define GPIO_SPEED_HIGH         3                                   /*!< GPIO very high output speed */

/*
 * @GPIO_PUPD
 * GPIO pin configuration: Pull-up/Pull-down
 */

#define GPIO_NO_PUPD            0                                   /*!< GPIO no pull-up or pull-down for pin */
#define GPIO_PIN_PU             1                                   /*!< GPIO pull-up pin */
#define GPIO_PIN_PD             2                                   /*!< GPIO pull-down pin */




/****************************************************************************************************
 *                                   APIs supported by this driver                                  *
 *                For more information about the APIs, check the function definitions               *
 ***************************************************************************************************/

/*
 * Peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) ;

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) ;
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) ;

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) ;
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) ;
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) ;
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) ;
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) ;

/*
 * IRQ configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) ;
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) ;
void GPIO_IRQHandler(uint8_t pinNumber) ;

#endif /* _STM32F429XX_H_ */
