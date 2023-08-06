# Handling SPI TXE Interrupts in the Code

|						| Handle TXE interrupt	|						|
|-----------------------------------------------|-----------------------|-----------------------------------------------|
|                      				| 8-bit or 16-bit mode?	|						|
| 8-bit						|			| 16-bit					|
| Write one byte to SPI Data Register (DR)	|			| Write 2 bytes to SPI Data Register (DR)	| 
| len--						|			| len -= 2					|
| YES - Transmission Over			| len = 0?		| NO - Transmission Not Over			|
| Close the SPI TX				|			| Wait until another TXE interrupt		|
