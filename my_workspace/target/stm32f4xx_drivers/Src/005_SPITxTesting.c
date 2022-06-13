/**
 * @file 005_SPITxTesting.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Test if SPI API Driver header works or not
 * @version 0.1
 * @date 2022-06-13
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note SPI alternate functionality pin numbers:
 *          PB12     SPI2_NSS
 *          PB13     SPI2_SCK
 *          PB14     SPI2_MISO
 *          PB15     SPI2_MOSI
 *          Alt Function Mode: 5
 * 
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"
#include "stm32f429xx_spi_driver.h"

int main(void) {
    return 0 ;
}
