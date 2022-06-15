/**
 * @file main.cpp
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Arduino peripheral for STM32F429xx controller receives string from button press
 * @version 0.1
 * @date 2022-06-14
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note SPI pin numbers:
 *       SCK    13      // Serial Clock
 *       CIPO   12      // Controller In Peripheral Out
 *       COPI   11      // Controller Out Peripheral In
 *       PS     10      // Peripheral Select
 */

#include <Arduino.h>
#include <SPI.h>

#define SPI_SCK             13
#define SPI_CIPO            12
#define SPI_COPI            11
#define SPI_PS              10

char dataBuff[500] ;

void SPI_PeripheralInit(void) ;
uint8_t SPI_PeripheralReceive(void) ;
void SPI_PeripheralTransmit(char data) ;

void setup() {
    // Begin transferring data at a rate of 9600 bits/second
    Serial.begin(9600) ;
    // Initialize SPI peripheral
    SPI_PeripheralInit() ;
    Serial.println("Peripheral initialized") ;
}

void loop() {
    uint32_t i ;
    uint16_t dataLen = 0 ;
    Serial.println("Peripheral waiting for SS/PS to go LOW") ;
    while (digitalRead(SS)) ;

    i = 0 ;
    dataLen = SPI_PeripheralReceive() ;
    for (i = 0 ; i < dataLen ; i++) {
        dataBuff[i] = SPI_PeripheralReceive() ;
    }

    // This prints out the length of the message
    // Serial.println(String(i, HEX)) ;

    dataBuff[i] = '\0' ;

    Serial.print("RCVD: ") ;
    Serial.println(dataBuff) ;
    Serial.print("Length: ") ;
    Serial.println(dataLen) ;
}

/****************************************************************************************************
 * @fn                  SPI_PeripheralInit
 * 
 * @brief               Initialize SPI pins as INPUT or OUTPUT and enable SPI as peripheral
 * 
 * @return              none
 * 
 * @note                none
 * 
 */
void SPI_PeripheralInit(void) {
    // Initialize SPI pins
    pinMode(SCK, INPUT) ;
    pinMode(MOSI, INPUT) ;
    pinMode(MISO, OUTPUT) ;
    pinMode(SS, INPUT) ;

    // Enable SPI as peripheral device
    SPCR = (1 << SPE) ;
}

/****************************************************************************************************
 * @fn                  SPI_PeripheralReceive
 * 
 * @brief               Returns/reads contents of SPDR (SPI Data Register)
 * 
 * @return uint8_t 
 * 
 * @note                none
 * 
 */
uint8_t SPI_PeripheralReceive(void) {
    // Wait for reception complete
    while (!(SPSR & (1 << SPIF))) ;
    // Return data register
    return SPDR ;
}

/****************************************************************************************************
 * @fn                  SPI_PeripheralTransmit
 * 
 * @brief               Sends one byte of data
 * 
 * @param data          Data to be sent via SPI communication
 * 
 * @note                none
 * 
 */
void SPI_PeripheralTransmit(char data) {
    // Start transmission
    SPDR = data ;
    // Wait for transmission complete
    while (!(SPSR & (1 << SPIF))) ;
}
