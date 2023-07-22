#include <Arduino.h>
/*

 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * CIPO  12  // Controller In Peripheral Out.
 * COPI  11  // Controller Out Peripheral In.
 * PS    10  // Peripheral Select
 *

 */

#include <SPI.h>

/* Migrating to use of controller/peripheral or main/secondary instead of master/slave*/
#define COPI    MOSI
#define CIPO    MISO
#define PS      SS

const byte led = 9;           // Peripheral LED digital I/O pin.

boolean ledState = HIGH;      // LED state flag.

uint8_t dataBuff[255];

// uint8_t board_id[10] = "ARDUINOUNO";
char const board_id[11] = "ARDUINOUNO";

#define NACK    0xA5
#define ACK     0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54

#define LED_ON     1
#define LED_OFF    0

// Arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

// Initialize SPI peripheral.
void SPI_PeripheralInit(void)  { 
    // Initialize SPI pins.
    pinMode(SCK, INPUT);
    pinMode(COPI, INPUT);
    pinMode(CIPO, OUTPUT);
    pinMode(PS, INPUT);
    // Make SPI as peripheral

    // Enable SPI as peripheral.
    SPCR = (1 << SPE);
}

// This function returns SPDR Contents 
uint8_t SPI_PeripheralReceive(void) {
    /* Wait for reception complete */
    while(!(SPSR & (1<<SPIF)));
    /* Return Data Register */
    return SPDR;
}


// Sends one byte of data 
void SPI_PeripheralTransmit(uint8_t data) {
    /* Start transmission */
    SPDR = data;

    /* Wait for transmission complete */
    while(!(SPSR & (1<<SPIF)));
}
  

// The setup() function runs after reset.
void setup() {
    // Initialize serial for troubleshooting.
    Serial.begin(9600);

    // Initialize peripheral LED pin.
    pinMode(led, OUTPUT);

    digitalWrite(led,LOW);

    // Initialize SPI peripheral.
    SPI_PeripheralInit();

    Serial.println("Peripheral Initialized");
}


byte checkData(byte commnad) {
    //todo
    return ACK;
}

// The loop function runs continuously after setup().
void loop() {
    byte data,command,len,ackornack=NACK;

    //1. fist make sure that ss is low . so lets wait until ss is low 
    Serial.println("Peripheral waiting for ss to go low");
    while(digitalRead(PS) );

    //2. now lets wait until rx buffer has a byte
    command = SPI_PeripheralReceive();
    ackornack = checkData(command);

    SPI_PeripheralTransmit(ackornack);

    len = SPI_PeripheralReceive(); //dummy byte

    if(command == COMMAND_LED_CTRL) {
        //read 2 more bytes pin number and value 
        uint8_t pin = SPI_PeripheralReceive(); 
        uint8_t value = SPI_PeripheralReceive(); 
        Serial.println("RCVD:COMMAND_LED_CTRL");
        if(value == (uint8_t)LED_ON) {
            digitalWrite(pin,HIGH);
        }
        else if (value == (uint8_t) LED_OFF) {
            digitalWrite(pin,LOW);
        }
    }

    else if (command == COMMAND_SENSOR_READ) {
        // Read analog pin number 
        uint16_t aread;
        uint8_t pin = SPI_PeripheralReceive(); 
        // PinMode(pin+14, INPUT_PULLUP);
        uint8_t val;
        aread = analogRead(pin+14);
        val = map(aread, 0, 1023, 0, 255);

        SPI_PeripheralTransmit(val);

        val = SPI_PeripheralReceive(); //dummy read

        Serial.println("RCVD:COMMAND_SENSOR_READ");
    }

    else if (command == COMMAND_LED_READ) {
        uint8_t pin = SPI_PeripheralReceive(); 
        uint8_t val = digitalRead(pin);
        SPI_PeripheralTransmit(val);
        val = SPI_PeripheralReceive(); //dummy read
        Serial.println("RCVD:COMMAND_LED_READ");
    }

    else if (command == COMMAND_PRINT) {
        uint8_t len = SPI_PeripheralReceive(); 
        for(int i=0 ; i < len ; i++) {
            dataBuff[i] = SPI_PeripheralReceive();
        }
        Serial.println((char*)dataBuff);

        Serial.println("RCVD:COMMAND_PRINT");
    }

    else if (command == (unsigned int)COMMAND_ID_READ) {
        for(unsigned int i=0 ; i < strlen(board_id) ; i++)
        {
            SPI_PeripheralTransmit(board_id[i]);
        }
        SPI_PeripheralReceive();
        Serial.println("RCVD:COMMAND_ID_READ");
    }
}