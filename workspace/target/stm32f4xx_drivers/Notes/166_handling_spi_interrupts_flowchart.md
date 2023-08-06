# Handling SPI Interrupts in the Code

| Enter ISR							|
|---------------------------------------------------------------|
|Understand which event caused interrupt to trigger (check SR)  |

| Interrupt is Due to Setting of RXNE flag	|  Handle RXNE event	|
|-----------------------------------------------|-----------------------|
| Interrupt is due to setting of TXE flag	| Handle TXE flag	|
| Interrupt is due to setting of ERROR flag	| Handle error		|
