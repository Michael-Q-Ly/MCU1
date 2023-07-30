/**
 * @file 007_SPICmdHandling.c
 * @author Michael Ly (github.com/Michael-Q-Ly)
 * @brief Arduino as a SPI peripheral device with STM32F429xx controller
 * @version 0.1
 * @date 2023-07-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "stm32f429xx.h"
#include "stm32f429xx_gpio_driver.h"
#include "stm32f429xx_spi_driver.h"
#include <string.h>

/* Command Codes */
// STM32F4xx commands
#define COMMAND_LED_CTRL 		0x50	/*<< STM32F4xx LED control command >>*/
#define COMMAND_SENSOR_READ		0x51	/*<< STM32F4xx sensor read command  >>*/
#define COMMAND_LED_READ		0x52	/*<< STM32F4xx LED read command >>*/
#define COMMAND_PRINT			0x53	/*<< STM32F4xx print command >>*/
#define COMMAND_ID_READ			0x54	/*<< STM32F4xx ID read command >>*/

// STM32F4xx LED
#define LED_ON 					1		/*<< STM32F4xx LED Active HIGH >>*/
#define LED_OFF 				0		/*<< STM32F4xx LED Active HIGH >>*/

// Arduino Uno R3 analog pins
#define UNOR3_ANALOG_PIN_NO_0 	0
#define UNOR3_ANALOG_PIN_NO_1	1
#define UNOR3_ANALOG_PIN_NO_2	2
#define UNOR3_ANALOG_PIN_NO_3	3
#define UNOR3_ANALOG_PIN_NO_4	4

// Arduino LED
#define UNOR3_LED_PIN 			9

extern void initialise_monitor_handles() ;

static void SPI2_GPIOInits(void) ;
static void SPI2_Inits(void) ;
static void GPIO_ButtonInit(void) ;
static void delay(void) ;
static void wait_for_button_press(void) ;
static uint8_t SPI_VerifyResponse(uint8_t ackByte) ;
static void SPI_ResetBusyFlag(void) ;

// Helper functions
static void dummy_read_write(uint8_t dummyRead, uint8_t dummyWrite) ;

// SPI commands
static void send_CMD_LED_CTRL(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) ;
static void send_CMD_SENSOR_READ(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) ;
static void send_CMD_LED_READ(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) ;
static void send_CMD_PRINT(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) ;

int main(void) {
	initialise_monitor_handles();
	printf("Application is running\n") ;

	/*
	 * PC2  --> SPI2_MISO
	 * PC3  --> SPI2_MOSI
	 * PB10 --> SPI2_SCLK
	 * PB9  --> SPI2_NSS
	 * ALT function mode: 5
	 */
	uint8_t dummyWrite = 0xFF ;
	uint8_t dummyRead  = 0xFF ;

    // Configure GPIO button
    GPIO_ButtonInit() ;

    // Configure GPIO to behave as SPI2 pins
    SPI2_GPIOInits() ;

    // Initialize SPI2 peripheral parameters
    SPI2_Inits() ;
	printf("SPI initialization done\n") ;

    /*
     * Setting SSOE does NSS output enable.
     * The NSS pin is automatically managed by the hardware.
     * I.e., when SPE = 1, NSS will be pulled LOW,
     * and NSS pin will be HIGH when SPE = 0.
     */
    SPI_SSOEConfig(SPI2, ENABLE) ;

    while (1) {
    	wait_for_button_press() ;
        // Enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE) ;

        uint8_t commandCode = COMMAND_LED_CTRL ;
        uint8_t ackByte ;
        uint8_t args[2] ;

        /**************************************************************/
        /*            1. CMD_LED_CTRL <pin_no_1>  	<value_1>         */
        /**************************************************************/

        send_CMD_LED_CTRL(dummyRead, dummyWrite, &commandCode, &ackByte, args) ;

        /**************************************************************/
        /*           2. CMD_SENSOR_READ <analog_pin_no_1>			  */
        /**************************************************************/

        wait_for_button_press() ;
        commandCode = COMMAND_SENSOR_READ ;
        send_CMD_SENSOR_READ(dummyRead, dummyWrite, &commandCode, &ackByte, args) ;

        /**************************************************************/
        /*                   3. CMD_LED_READ <pin_no_1>               */
        /**************************************************************/
        wait_for_button_press() ;
        commandCode = COMMAND_LED_READ ;
        send_CMD_LED_READ(dummyRead, dummyWrite, &commandCode, &ackByte, args) ;

        /**************************************************************/
        /*              4. CMD_PRINT <len(2)> <message(len)>          */
        /**************************************************************/

        wait_for_button_press() ;
        commandCode = COMMAND_PRINT ;
        send_CMD_PRINT(dummyRead, dummyWrite, &commandCode, &ackByte, args) ;

#ifdef later
        /**************************************************************/
        /*               *5. CMD_ID_READ <analog_pin_no_1>            */
        /**************************************************************/

        wait_for_button_press() ;
        commandCode = COMMAND_ID_READ ;
        send_CMD_ID_READ(dummyRead, dummyWrite, &commandCode, &ackByte, args);
        /*TODO: move this function prototype and make definition */
void send_CMD_ID_READ(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) ;
#endif
    } // End while(1)

    // Program should never reach here!
    return 0 ;
}

/****************************************************************************************************
 * @fn 					void SPI2_GPIOInits(void)
 * @brief 				Initialize the GPIO pins to behave as SPI2 pins
 *
 * @pre 				GPIO off
 * @post				Turn on GPIO B9, B10, C3, C2
 * @note 				B9 = CSZ/NPS | B10 = SCK | C3 = COPI | C2 = CIPO
 */
void SPI2_GPIOInits(void) {
    /*
     * Configure the SPI pins
     */

    GPIO_Handle_t SPIPins ;
    memset(&SPIPins, 0, sizeof(SPIPins)) ;

    SPIPins.pGPIOx = GPIOB ;
    SPIPins.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_ALT_FUN ;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST ;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl     = GPIO_NO_PUPD ;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP ;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode   = 5 ;


    /*
     * Configure the SPI2 pins
     */

    // NSS / NPS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9 ;
    GPIO_Init(&SPIPins) ;

    // SCK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10 ; /* PB13 did not work */
    GPIO_Init(&SPIPins) ;

    // MOSI / COPI
    SPIPins.pGPIOx = GPIOC ;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3 ; /* PB15 did not work */
    GPIO_Init(&SPIPins) ;

    // MISO / CIPO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2 ; /* PB15 did not work */
    GPIO_Init(&SPIPins) ;
}

/****************************************************************************************************
 * @fn 					void SPI2_Inits(void)
 * @brief 				Initialize the SPI port 2 peripheral
 *
 * @pre 				SPI2 off
 * @post 				SPI 2 on as device mode ctrl, full duplex, clk div8,
 * 						DFF 8-bit, CPOL_LOW, CPHA_LOW, and hardware periph mgmt
 * 						enabled for CSZ pin
 */
void SPI2_Inits(void) {
    SPI_Handle_t SPI2Handle ;
    memset(&SPI2Handle, 0, sizeof(SPI2Handle)) ;

    SPI2Handle.pSPIx = SPI2 ;
    SPI2Handle.SPIConfig.SPI_DeviceMode     = SPI_DEVICE_MODE_CONTROLLER ;
    SPI2Handle.SPIConfig.SPI_BusConfig      = SPI_BUS_CONFIG_FD ;
    SPI2Handle.SPIConfig.SPI_SclkSpeed      = SPI_SCLK_SPEED_DIV8 ;                     /* Generates SCK of 2 MHz */
    SPI2Handle.SPIConfig.SPI_DFF            = SPI_DFF_8BITS ;
    SPI2Handle.SPIConfig.SPI_CPOL           = SPI_CPOL_LOW ;
    SPI2Handle.SPIConfig.SPI_CPHA           = SPI_CPHA_LOW ;
    SPI2Handle.SPIConfig.SPI_SSM            = SPI_SSM_DI ;                              /* Hardware slave management enabled for NSS pin */

    SPI_Init(&SPI2Handle) ;
}

/****************************************************************************************************
 * @fn 					void GPIO_ButtonInit(void)
 * @brief 				Initialize STM32 A0 (user button) pin as input GPIO
 *
 * @pre 				GPIO A0 default
 * @post 				GPIOA0 input button
 */
void GPIO_ButtonInit(void) {
    GPIO_Handle_t GPIOHandle ;
    memset(&GPIOHandle, 0, sizeof(GPIOHandle)) ;

    GPIOHandle.pGPIOx = GPIOA ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_0 ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_INPUT ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST ;
    GPIOHandle.GPIO_PinConfig.GPIO_PinPuPdCtrl  = GPIO_NO_PUPD ;

    GPIO_Init(&GPIOHandle) ;
}
/****************************************************************************************************
 * @fn 					void delay(void)
 * @brief       		Software delay; can be used for debouncing
 *
 * @pre					none
 * @post 				Software delay (blocking); continue program
 */
void delay(void) {
    for (uint32_t i = 0 ; i < 500000/2 ; i++) ;
}

/****************************************************************************************************
 * @fn 					void wait_for_button_press(void)
 * @brief 				Waits for GPIO A0 to be pressed and then debounces
 *
 * @pre 				User button not pressed
 * @post				User button has been pressed then released
 */
void wait_for_button_press(void) {
        // Wait until a button press
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)) ;

        // Debounce button press with software delay for clean read
        delay() ;
}

/****************************************************************************************************
 * @fn  				SPI_VerifyResponse(uint8_t)
 *
 * @brief 				Verifies what acknowledge byte is returned from peripheral device
 *
 * @pre 				Controller sends message to peripheral
 * @post				Controller Receives message and gets either ack or nack
 * @param[in] ackByte	ACK or NACK
 * @return 				ack = 1, nack = 0
 */
uint8_t SPI_VerifyResponse(uint8_t ackByte) {
	if (ackByte == 0xF5) {
		// ack
		return 1 ;
	}
	// nack
	return 0 ;
}

/****************************************************************************************************
 * @fn 					void dummy_read_write(uint8_t, uint8_t)
 * @brief 				Performs a dummy read to clear RXNE and then a dummy transmit to fetch response
 *
 * @pre 				Enable SPI2 and send first byte
 * @post				RXNE cleared and byte fetched
 * @param dummyRead 	Clears RXNE
 * @param dummyWrite 	Fetches response from peripheral device
 */
void dummy_read_write(uint8_t dummyRead, uint8_t dummyWrite) {
	// Perform dummy read to clear RXNE
	SPI_ReceiveData(SPI2, &dummyRead, 1) ;

	// Send some dummy bits (1 byte) to fetch response from the peripheral
	SPI_SendData(SPI2, &dummyWrite, 1) ;
}

/****************************************************************************************************
 * @fn 					void SPI_ResetBusyFlag()
 * @brief 				Waits for the SPI_BSY flag to reset, which indicates SPI communication is no longer busy
 *
 * @pre 				SPI must be asking to receive data
 * @post 				SPI_BSY flag is reset
 */
void SPI_ResetBusyFlag() {
		// Wait for BSY bit to reset  -> This will indicate that SPI is not busy in communication
		while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) == FLAG_SET) ;
		// Clear the OVR flag by reading DR and SR
		uint8_t temp __attribute__((unused)) = SPI2->DR ; /* temp is declared, but not referenced */
		temp = SPI2->SR ;
}

/****************************************************************************************************
 * @fn  				void send_CMD_LED_CTRL(uint8_t, uint8_t, uint8_t* const, uint8_t*, uint8_t*)
 * @brief 				Send first command to peripheral - Turn LED ON
 *
 * @pre 				Wait for user GPIO to be pressed and receive ack
 * @post				Turns LED on from peripheral device
 *
 * @param dummyRead 	Clears RXNE
 * @param dummyWrite	Fetches response from peripheral device
 * @param commandCode	Code to send to peripheral device
 * @param ackByte 		Checks for ack or nack
 * @param args 			Arguments to pick peripheral device and pin for LED
 */
void send_CMD_LED_CTRL(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) {
	// Send command
	SPI_SendData(SPI2, commandCode, 1) ;

	dummy_read_write(dummyRead, dummyWrite) ;

	// Receive the ack byte received
	SPI_ReceiveData(SPI2, ackByte, 1) ;

	// Verify ack or nack
	if (SPI_VerifyResponse(*ackByte)) {
		args[0] = UNOR3_LED_PIN ;
		args[1] = LED_ON ;

		// Send arguments
		SPI_SendData(SPI2, args, 2) ; /* 2 bytes sent */

		SPI_ResetBusyFlag();

		printf("CMD_LED_CTRL executed\n") ;
	}
}

/****************************************************************************************************
 * @fn 					void send_CMD_SENSOR_READ(uint8_t, uint8_t, uint8_t* const, uint8_t*, uint8_t*)
 * @brief 				Send second command to peripheral device - Get mapped analog value of LED
 *
 * @pre 				Wait for user GPIO to be pressed and receive ack
 * @post 				Receives analog value back from peripheral device
 *
 * @param dummyRead 	Clears RXNE
 * @param dummyWrite	Fetches response from peripheral device
 * @param commandCode	Code to send to peripheral device
 * @param ackByte 		Checks for ack or nack
 * @param args 			Arguments to pick peripheral device and pin for LED
 */
void send_CMD_SENSOR_READ(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) {
	// Send command
	SPI_SendData(SPI2, commandCode, 1) ;

	dummy_read_write(dummyRead, dummyWrite) ;

	// Receive the ack byte received
	SPI_ReceiveData(SPI2, ackByte, 1) ;

	// Verify ack or nack
	if (SPI_VerifyResponse(*ackByte)) {
		// Send arguments
		args[0] = UNOR3_ANALOG_PIN_NO_0 ;

		// Send arguments
		SPI_SendData(SPI2, args, 1) ; /* 1 byte sent */

		// Do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1) ;

		// Insert delay so peripheral can have data ready
		delay();

		// Send some dummy bits (1 byte) to fetch response from the peripheral
		SPI_SendData(SPI2, &dummyWrite, 1) ;

		uint8_t analogRead ;
		SPI_ReceiveData(SPI2, &analogRead, 1) ;

		// Wait for BSY bit to reset  -> This will indicate that SPI is not busy in communication
		SPI_ResetBusyFlag() ;

		printf("CMD_SENSOR_READ %d\n", analogRead) ;
	}
}

/****************************************************************************************************
 * @fn 					void send_CMD_LED_READ(uint8_t, uint8_t, uint8_t* const, uint8_t*, uint8_t*)
 * @brief 				Send third command to peripheral device - Get LED logic level
 *
 * @pre 				Wait for user GPIO to be pressed and receive ack
 * @post 				Receives analog value back from peripheral device
 * @param dummyRead 	Clears RXNE
 * @param dummyWrite	Fetches response from peripheral device
 * @param commandCode	Code to send to peripheral device
 * @param ackByte 		Checks for ack or nack
 * @param args 			Arguments to pick peripheral device and pin for LED
 */
void send_CMD_LED_READ(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) {
	// Send command
	SPI_SendData(SPI2, commandCode, 1) ;

	dummy_read_write(dummyRead, dummyWrite) ;

	// Receive the ack byte received
	SPI_ReceiveData(SPI2, ackByte, 1) ;

	// Verify ack or nack
	if (SPI_VerifyResponse(*ackByte)) {
		// Send arguments
		args[0] = UNOR3_ANALOG_PIN_NO_0 ;

		// Send arguments
		SPI_SendData(SPI2, args, 1) ; /* 1 byte sent */

		// Do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1) ;

		// Insert delay so peripheral can have data ready
		delay();

		// Send some dummy bits (1 byte) to fetch response from the peripheral
		SPI_SendData(SPI2, &dummyWrite, 1) ;

		uint8_t ledStatus ;
		SPI_ReceiveData(SPI2, &ledStatus, 1) ;

		// Wait for BSY bit to reset  -> This will indicate that SPI is not busy in communication
		SPI_ResetBusyFlag() ;

		printf("CMD_LED_READ %s\n", ledStatus == LED_ON ? "ON" : "OFF") ;
	}
}

/****************************************************************************************************
 * @fn 					void send_CMD_PRINT(uint8_t, uint8_t, uint8_t* const, uint8_t*, uint8_t*)
 * @brief 				Send fourth command to peripheral device - Send string to peripheral and have it print that string
 *
 * @pre 				Wait for user GPIO to be pressed and receive ack
 * @post 				Receives analog value back from peripheral device
 *
 * @param dummyRead 	Clears RXNE
 * @param dummyWrite	Fetches response from peripheral device
 * @param commandCode	Code to send to peripheral device
 * @param ackByte 		Checks for ack or nack
 * @param args 			Arguments to pick peripheral device and pin for LED
 */
void send_CMD_PRINT(uint8_t dummyRead, uint8_t dummyWrite, uint8_t *const commandCode, uint8_t *ackByte, uint8_t *args) {
	// Send command
	SPI_SendData(SPI2, commandCode, 1) ;

	dummy_read_write(dummyRead, dummyWrite) ;

	// Receive the ack byte received
	SPI_ReceiveData(SPI2, ackByte, 1) ;

	uint8_t message[] = "Hello! How are you?" ;
	// Verify ack or nack
	if (SPI_VerifyResponse(*ackByte)) {
		// Send argument to be length of message
		args[0] = strlen( (char *)message ) ;

		// Send arguments
		SPI_SendData(SPI2, args, 1) ; /* 1 byte sent */

		// Do a dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1) ;

		// Insert delay so peripheral can have data ready
		delay();

		// Send message
		int index = 0 ;
		while (index < args[0]) {
			SPI_SendData(SPI2, &message[index], 1) ;
			SPI_ReceiveData(SPI2, &dummyRead, 1) ;
			index++ ;
		}
//		SPI_SendData(SPI2, message, args[0]) ;

		// Wait for BSY bit to reset  -> This will indicate that SPI is not busy in communication
		SPI_ResetBusyFlag() ;

		printf("CMD_PRINT executed\n") ;
	}
}
