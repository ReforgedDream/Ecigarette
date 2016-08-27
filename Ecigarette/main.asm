; Created: 25.05.2016 14:12:47

;.include "tn13adef.inc"
.include "m64adef.inc"

;.device attiny13a
.device atmega64a

//Constants definition
.equ segind0 = 0xC0	//7-segment LED indicator's digits definitions
.equ segind1 = 0xF9	//		 0
.equ segind2 = 0xA4	//		 _
.equ segind3 = 0xB0	//	5	|_|	1  <--6
.equ segind4 = 0x99	//	4	|_|.7	2 
.equ segind5 = 0x92	//
.equ segind6 = 0x82	//		 3
.equ segind7 = 0xF8	//
.equ segind8 = 0x80	//
.equ segind9 = 0x90	//
.equ segindDash = 0b_1011_1111		//"-" sign
.equ segindA = 0b_1000_1000		//"A"
.equ segindB = 0b_1000_0011		//"b"
.equ segindC = 0b_1100_0110		//"C"
.equ segindD = 0b_1010_0001		//"d"
.equ segindE = 0b_1000_0110		//"E"
.equ segindF = 0b_1000_1110		//"F"

.equ segindR = 0b_1010_1111		//"r"

.equ presetLow = 0x39	//Here we can set a desired period of time for a precise delay (2 bytes long)
.equ presetHigh = 0x01	//on 16 MHz and with no prescaler, there is approx. 62500 (0xF424) overflows per second (for 8-bit timer)
						//and 0x7A12 on 8 MHz
//150 Hz (0x01A1) works well for the red LED indicator
//200 Hz (0x0139) for the green 4-digit indicator
.equ owfPerSecond8BitLow = 0x24
.equ owfPerSecond8BitHigh = 0xF4 //supra

//Symbolic custom registers names
.def currentPower				 =	R17
;.def overflowsCounterBlink		 =	R18
.def overflowsCounterBlockAdj	 =	R18


.def digitToDisp1 = R21			//1st digit to be displayed on the LED. Only hexadecimal digits are defined!
.def digitToDisp2 = R22			//2nd digit to be displayed on the LED
.def digitToDisp3 = R23			//3rd digit to be displayed on the LED
.def digitToDisp4 = R20			//4th digit to be displayed on the LED
.def overflowsCounterLow = R24		//Incrementing every time when overflow of the timer0 occurs
.def overflowsCounterHigh = R25		//my my, there's too many overflows for a humble 8-bit register


.def flagStorage = R19			//Custom flags storage
								//And custom symbolic bit names, why not
.equ blockAdj	=	0
.equ ledState	=	1
.equ achtung =						2	//USART: new data received
.equ transmit =						3	//USART: a command sequence is recognized
.equ timeToRefresh =				4	//LED digits should be refreshed
.equ uartTXBufferOverflow =			5

.equ control1 = 0x61			//"a"
.equ control2 = 0x77			//"w"
.equ control3 = 0x6B			//"k" (ASCII)

.equ blink = 0x04				//0x04 = 55 ms on 8 MHz, prescaler=64, system divider=8 
.equ buttonCutoff = 0x0B		//0x07 = 100 ms on 8 MHz, prescaler=64, system divider=8

.equ uartRAMStorageRXLength = 8	//well, a length of storage in RAM, dedicated for saving UART's received bytes
.equ uartRAMStorageTXLength = 8	//same for bytes to transmit (255 bytes maximum!)

;.equ owfPerSecond8BitHigh = 0x00
;.equ owfPerSecond8BitLow = 0x49		//8 MHz, prescaler=64, system divider=8 -> ~73 (0x0049) 8-bit overflows/sec

;--------------------------------------------------------------------------------------------
;Macro definitions

.macro UOUT        ;Universal OUT command. Works with either memory mapped and usual I/O registers.
.if @0 < 0x40
	OUT @0,@1         
.else
	STS @0,@1
.endif
.endm

.macro UIN        ;Universal IN command. Works with either memory mapped and usual I/O registers.
.if @1 < 0x40
	IN @0,@1         
.else
	LDS @0,@1
.endif
.endm

.macro PUSHSREG
PUSH R16		//Stores the value of R16 in stack
IN R16, SREG	//Stores SREG in R16...
PUSH R16		//...and then stores the value of SREG in stack
.endm

.macro POPSREG
POP R16			//Extract SREG value from stack...
OUT SREG, R16	//...and apply it to SREG
POP R16			//Extract R16 value from stack
.endm

;--------------------------------------------------------------------------------------------
.DSEG			//SRAM memory segment
.ORG SRAM_START //start from the beginning

uartRX:				.BYTE uartRAMStorageRXLength	//allocate space for read buffer...
uartErrorsCounter:	.BYTE 1		//...and for errors counter

uartTX:				.BYTE uartRAMStorageTXLength	//allocate space for write buffer
uartTXRead:			.BYTE 1
uartTXWrite:		.BYTE 1
;--------------------------------------------------------------------------------------------
.CSEG
//Reset and Interrupt Vectors table

	.ORG 0x0000	;(RESET) 
	RJMP Reset
/*
	.ORG INT0addr	;External Interrupt 0 (IRQ0 Handler)
	RETI

	.ORG PCI0addr	;External Interrupt Request 0 (PCINT0 Handler) (pin change)
	RJMP PinToggle
	
	.ORG OVF0addr	;Timer/Counter0 Overflow
	RJMP Timer0Over
		
	.ORG ERDYaddr	;EEPROM Ready
	RETI
	.ORG ACIaddr	;Analog Comparator
	RETI
	.ORG OC0Aaddr	;Timer/Counter0 Compare Match A
	RETI
	.ORG OC0Baddr	;Timer/Counter0 Compare Match B
	RETI
	.ORG WDTaddr	;Watchdog Time-out
	RETI
	.ORG ADCCaddr	;ADC Conversion Complete
	RETI
*/

	.ORG INT0addr	;(INT0) External Interrupt Request 0
	RETI
	.ORG INT1addr	;(INT1) External Interrupt Request 1
	RETI
	.ORG INT2addr	; External Interrupt Request 2
	RETI
	.ORG INT3addr	; External Interrupt Request 3
	RETI

	.ORG INT4addr	; External Interrupt Request 4
	RJMP PinToggle

	.ORG INT5addr	; External Interrupt Request 5
	RETI
	.ORG INT6addr	; External Interrupt Request 6
	RETI
	.ORG INT7addr	; External Interrupt Request 7
	RETI
	.ORG OC2addr	;(TIMER2 COMP) Timer/Counter2 Compare Match
	RETI
	.ORG OVF2addr	;(TIMER2 OVF) Timer/Counter2 Overflow
	RETI
	.ORG ICP1addr	;(TIMER1 CAPT) Timer/Counter1 Capture Event
	RETI
	.ORG OC1Aaddr	;(TIMER1 COMPA) Timer/Counter1 Compare Match A
	RETI
	.ORG OC1Baddr	;(TIMER1 COMPB) Timer/Counter1 Compare Match B
	RETI
	.ORG OVF1addr	;(TIMER1 OVF) Timer/Counter1 Overflow
	RETI
	.ORG OC0addr	;(TIMER0 COMP) Timer/Counter0 Compare Match
	RETI

	.ORG OVF0addr	;(TIMER0 OVF) Timer/Counter0 Overflow
	RJMP Timer0Over

	.ORG SPIaddr	;(SPI,STC) Serial Transfer Complete
	RETI
	.ORG URXC0addr	;(USART0,RXC) USART0, Rx Complete
	RETI
	.ORG UDRE0addr	;(USART0,UDRE) USART0 Data Register Empty
	RETI
	.ORG UTXC0addr	;(USART0,TXC) USART0, Tx Complete
	RETI
	.ORG ADCCaddr	;(ADC) ADC Conversion Complete
	RETI
	.ORG ERDYaddr	;(EE_RDY) EEPROM Ready
	RETI
	.ORG ACIaddr	;(ANA_COMP) Analog Comparator
	RETI
	.ORG OC1Caddr	; Timer/Counter1 Compare Match C
	RETI
	.ORG ICP3addr	; Timer/Counter3 Capture Event
	RETI
	.ORG OC3Aaddr	; Timer/Counter3 Compare Match A
	RETI
	.ORG OC3Baddr	; Timer/Counter3 Compare Match B
	RETI
	.ORG OC3Caddr	; Timer/Counter3 Compare Match C
	RETI
	.ORG OVF3addr	; Timer/Counter3 Overflow
	RETI

	.ORG URXC1addr	;(USART1,RXC) USART1, Rx Complete
	RJMP U1_RXcomplete

	.ORG UDRE1addr	;(USART1,UDRE) USART1 Data Register Empty
	RJMP U1_DREmpty

	.ORG UTXC1addr	;(USART1,TXC) USART1, Tx Complete
	RETI
	.ORG TWIaddr	;(TWI) 2-wire Serial Interface
	RETI
	.ORG SPMRaddr	;(SPM_RDY) Store Program Memory Ready
	RETI

.ORG INT_VECTORS_SIZE	;end of table

;--------------------------------------------------------------------------------------------
//Interrupts Handler//
;-------------------

Timer0Over:

PUSHSREG
 PUSH YL
  PUSH YH
/*   
INC overflowsCounterBlink

SBRC flagStorage, ledState		//Skip next instruction if the LED is off
RJMP ledIsOn	

	UIN R16, OCR0A
	COM R16
	LSR R16
	ANDI R16, ~(1<<7)

	CP overflowsCounterBlink, R16
	BRLO endBlink

		CLR overflowsCounterBlink
		ORI flagStorage, (1<<ledState)
		CBI PORTB, 2
		RJMP endBlink
	
ledIsOn:

	CPI overflowsCounterBlink, blink
	BRLO endBlink
		CLR overflowsCounterBlink
		ANDI flagStorage, ~(1<<ledState)
		SBI PORTB, 2

endBlink:
*/

ADIW overflowsCounterHigh:overflowsCounterLow, 1	//incrementing the whole word

CPI overflowsCounterHigh, presetHigh	//if the higher register contains half of preset...
BRLO notAFastPreset						//...AND...

CPI overflowsCounterLow, presetLow		//...the lower register contains another half, then go to executing our payload
BRLO notAFastPreset						//else - exit

//this string executes once in a certain period
	ORI flagStorage, (1<<timeToRefresh)	//Set timeToRefresh flag
	LDI overflowsCounterLow, 0x00	//zeroing the overflows counter
	LDI overflowsCounterHigh, 0x00
	INC R17							//Circling from 0 to 3 periodically (fast), to determine which digit should be lit
	CPI R17, 4						//compare with 4
	BRNE notAFastPreset				//if not 4, then exit, else reset R17 to 0
	LDI R17, 0						//reset to 0

notAFastPreset:

SBRS flagStorage, blockAdj			//If blockAdj==0 then skip
RJMP endOfTimerInt

	INC overflowsCounterBlockAdj

	CPI overflowsCounterBlockAdj, buttonCutoff
	BRLO endOfTimerInt	

		CLR overflowsCounterBlockAdj
		ANDI flagStorage, ~(1<<blockAdj)

endOfTimerInt:

  POP YH
 POP YL
POPSREG

RETI

;-------------------

PinToggle:

PUSHSREG

UIN R16, PINB					//Detecting if any of the adjusting buttons are pressed
COM R16							//Inversion (buttons are active low)
ANDI R16, (1 << 3 | 1 << 4)		//If no buttons is pressed, then the int is generated by button release
BREQ buttonReleased				//Jump to setting the flag 'blockAdj' and then exit

SBRC flagStorage, blockAdj		//If blockAdj==1 then skip
RJMP endPowerAdj				//Exit from the interrupt

	UIN R16, PINB
	ANDI R16, (1<<4)			//Detecting the SB2 button's state
	BRNE notSB2					//If R16 is zero now, then SB2 is pressed

		//SB2 IS PRESSED
		ORI flagStorage, (1<<blockAdj)		//Set the flag that forbids further interrupt algorithm executions
		INC currentPower					//Increment the 'currentPower' pointer
		CPI currentPower, 0x05				//Compare it with 5 that is over limit
		BRLO endPowerAdj					//If the pointer equals 5 then proceed to the next instructions, else jump

			LDI currentPower, 0x04			//Load the pointer with the maximum possible value
			RJMP endPowerAdj				//Exit from the interrupt

	notSB2:

	UIN R16, PINB
	ANDI R16, (1<<3)						//Detecting the SB1 button's state
	BRNE endPowerAdj						//If R16 is zero now, then SB1 is pressed

		//SB1 IS PRESSED

		ORI flagStorage, (1<<blockAdj)		//Set the flag that forbids further interrupt algorithm executions
		SUBI currentPower, 0x01				//Decrement the 'currentPower' pointer
		BRCS minReached						//If the pointer is below zero, then jump
		RJMP endPowerAdj					//...else exit from the interrupt

			minReached:
			LDI currentPower, 0x00			//Load the pointer with the minimum possible value
			
endPowerAdj:

POPSREG

RETI

buttonReleased:
	ORI flagStorage, (1<<blockAdj)			//Forbid further button processing in order to cut off contact bouncing
	RJMP endPowerAdj

;-------------------

U1_DREmpty:	//USART 1 Data Register Empty Interrupt

PUSH R18
 PUSH YH
  PUSH YL
   PUSHSREG	//Store both R16 and SREG in stack

LDS R16, uartTXRead		//Get a read and write pointers
LDS R18, uartTXWrite

SBRC flagStorage, uartTXBufferOverflow	//If an overflow has occured...
RJMP unreadData							//..then go to data transfer

CP R16, R18			//If the pointers are equal, then all data has been read and sent
BRNE unreadData

	//Buffer is empty
	UIN R18, UCSR1B
	ANDI R18, ~(1<<UDRIE1)	//Forbid this interrupt
	UOUT UCSR1B, R18
	RJMP exitTXIntTrue		//Exit

unreadData:

LDI YL, low(uartTX)		//Get a start address
LDI YH, high(uartTX)
ADD YL, R16
CLR R18
ADC YH, R18				//Add with carry the read pointer

LD R18, Y				//Extract a value...
UOUT UDR1, R18			//...and send it through UART

ANDI flagStorage, ~(1<<uartTXBufferOverflow)	//Clear TX Overflow flag
INC R16					//Increment the read pointer
CPI R16, uartRAMStorageTXLength	//Compare it with buffer length
BRLO exitTXInt
	CLR R16				//Clear if needed
exitTXInt:

STS uartTXRead, R16		//Store the read pointer

exitTXIntTrue:

   POPSREG		//Extract SREG and R16 from stack
  POP YL
 POP YH
POP R18

RETI

;-------------------

U1_RXcomplete:	//USART 1 Receive Complete interrupt

PUSH R18
 PUSH YH			//Saving in stack registers to be used
  PUSH YL
   PUSHSREG	//Store both R16 and SREG in stack

CLR R18
UIN R16, UCSR1A		//Read error flags
SBRC R16, 2			//Skip if no error
	SUBI R18, (-1)	//Else increment R18
SBRC R16, 3			//Skip...
	SUBI R18, (-1)	//...Increment
SBRC R16, 4			//Skip...
	SUBI R18, (-1)	//...Increment

LDI YL, low(uartErrorsCounter)	//Load Y pair with address of the Errors SRAM storage
LDI YH, high(uartErrorsCounter)
LD R16, Y						//Read error count from SRAM
ADD R16, R18					//Apply new errors to it...
ST Y, R16						//And store back

UIN R16, UDR1	//Read received data
CPI R18, 0x00	//If there are read errors...
BREQ errorFF
	LDI R16, 0xFF	//Then write 0xFF instead of data
errorFF:
ST X+, R16		//Store data in SRAM and post-increment

LDI YL, low(uartRX)	//Load Y pair with start address of the SRAM storage
LDI YH, high(uartRX)
ADIW YH:YL, (uartRAMStorageRXLength-1)	//add (storage_size - 1) to Y
CP YL, XL
CPC YH, XH			//If X pointer reached last allocated cell in the storage...
BRGE exitInt2		//...then reset it to the beginning
	LDI XL, low(uartRX)	//load X pointer with address of SRAM storage
	LDI XH, high(uartRX)

exitInt2:

ORI flagStorage, (1<<achtung)	//Achtung! Some new data received!

   POPSREG		//Extract SREG and R16 from stack
  POP YL		//Extracting saved values from stack
 POP YH
POP R18

RETI

;-------------------

//End of Interrupts Handler//

;--------------------------------------------------------------------------------------------
//Storage of static data in flash

fixedLevels:	.db 0x4D, 0x7F, 0xB3, 0xDA, 0xFF, 0x00

decAddrTable: .dw disp0, disp1, disp2, disp3, disp4, \
		disp5, disp6, disp7, disp8, disp9, \
		dispA, dispB, dispC, dispD, dispE, dispF, \
		dispR, dispDash	//Adresses of the labels, stored in a certain place (decAddrTable) in program memory
;--------------------------------------------------------------------------------------------
Reset:

//SRAM flush
			LDI	ZL, Low(SRAM_START)	; Load Z with SRAM start address
			LDI	ZH, High(SRAM_START)
			CLR	R16					; R16 <- 0x00
Flush:		ST 	Z+, R16				; Flush byte and increment
			CPI	ZH, High(RAMEND+1)	; Is ZH == high half of the RAMEND address?
			BRNE Flush				; Loop if no

			CPI	ZL,Low(RAMEND+1)	; Same for low half of the address
			BRNE Flush

		CLR	ZL
		CLR	ZH

//R0-R31 flush
	LDI	ZL, 0x1E	; Address of R30 (in SRAM address space)
	CLR	ZH
	DEC	ZL			; Decrement address (flushing begins from R29 since we use R30:R31 as an address pointer)
	ST Z, ZH		; Load register with zero
	BRNE PC-2		; If Zero flag is cleared step back 2 times

//Thanks for code to DI HALT, Testicq and all fellow comrades from easyelectronics.ru

//Stack initialization
;LDI R16, RAMEND
;UOUT SPL, R16
LDI R16, Low(RAMEND)
OUT SPL, R16
LDI R16, High(RAMEND)
OUT SPH, R16

//------------------------------------

//Timer initialization

	//Timer2
	UIN R16, DDRB
	ANDI R16, ~(1 << PB7)	//PWM pin disabled by default
	UOUT DDRB, R16

	LDI R16, 0b_1000_0011	//Fast PWM mode
	UOUT TCCR2, R16

	LDI R16, 0b_0000_0011	//clock/64
	UOUT TCCR0B, R16


//Timer0 Initialization
LDI R16, 0b_0000_0001	//set CS00 bit in TCCR0 register
OUT TCCR0, R16			//now using system clock for Timer0 without prescaler
CLR R16
ANDI R16, (1 << TOIE0 | 1 << TOIE2)	//set CS00 bit in TCCR0 register
OUT TIMSK, R16			//set TOIE0 in TIMSK register
//now we have the overflow interrupt enabled for timer0

//------------------------------------

//USART1 Initialization

// Set baud rate (f osc = 16 MHz)
// 2400 baud -> 0x01A0
// 9600 baud -> 0x0067
// 1Mbaud -> 0x0000
// For 8 MHz, to achieve the same speed grades, U2X bit should be enabled
LDI R16, 0x00
UOUT UBRR1H, R16
LDI R16, 0x00
UOUT UBRR1L, R16

// 7 - (RXC1) USART Receive Complete					(r/o)
// 6 - (TXC1) USART Transmit Complete (clearing by writing 1)
// 5 - (UDRE1) USART Data Register Empty				(r/o)
// 4 - (FE1) Frame Error (must be set to 0)			(r/o)
// 3 - (DOR1) Data Overrun (must be set to 0)		(r/o)
// 2 - (UPE1) USART Parity Error (must be set to 0)	(r/o)
// 1 - (U2X1) Double the USART Transmission Speed
// 0 - (MPCM1) Multi-Processor Communication Mode
LDI R16, 0b_0100_0000
UOUT UCSR1A, R16

// 7 - (RXCIE1) RX Complete Interrupt Enable
// 6 - (TXCIE1) TX Complete Interrupt Enable
// 5 - (UDRIE1) USART Data Register Empty Interrupt Enable
// 4 - (RXEN1) Receiver Enable
// 3 - (TXEN1) Transmitter Enable
// 2 - (UCSZ12) Character Size (combined with the UCSZn1:0 bit in UCSRC)
// 1 - (RXB81) Receive Data Bit 8 (for nine data bits only)	(r/o)
// 0 - (TXB81) Transmit Data Bit 8 (for nine data bits only)
// TX Complete' and 'UDR Empty' interrupts enable, enable receiver and transmitter, no ninth bit:
LDI R16, 0b_1011_1000
UOUT UCSR1B, R16

// Set frame format: asynchronous operation, no parity, 8 data, 1 stop bit
LDI R16, 0b_0000_0110
UOUT UCSR1C, R16

//------------------------------------
//external interrupt init

LDI R16, 0b_0010_0000	//enable toggle int
UOUT GIMSK, R16

LDI R16, 0b_0001_1000	//pin 3 and 4 toggling contribute to the int. activation
UOUT PCMSK, R16

//------------------------------------
//gpio init
/*
LDI R16, 0b_0000_0100		//tri-stated inputs, low level on outputs
UOUT PORTB, R16
*/
LDI R16, 0b_0000_0100		//tri-stated inputs, low level on outputs
UOUT PORTF, R16

LDI R16, 0xFF
OUT DDRC, R16			//write 1-s into each port C...
OUT DDRA, R16			//...and port A direction registers

//------------------------------------

CLR R16		//clear R16 for the order's sake

;LDI currentPower, 0x01

SEI			//interrupts enabled globally

;--------------------------------------------------------------------------------------------

//Main Routine//
Start:

UIN R16, PINF
ANDI R16, (1<<1)
BRNE label1

//PRESSED
	LDI R16, 0b_0000_0101	//PWM pin enabled
	UOUT DDRB, R16
	
RJMP label2
label1:

//NOT PRESSED
	LDI R16, 0b_0000_0100	//PWM pin disabled
	UOUT DDRB, R16

label2:

//---

LDI ZL, low(fixedLevels)
LDI ZH, high(fixedLevels)
MOV R16, currentPower
ADD ZL, R16
CLR R16
ADC ZH, R16
LPM R16, Z

UOUT OCR0A, R16

;-------------------------------------

//Preparing data for 7-segment LED and displaying it

SBRS flagStorage, timeToRefresh		//Data refreshing occurs only once in a certain period set by timer
RJMP notATimeToRefresh				//If the flag isn't set then skip

	ANDI flagStorage, ~(1<<timeToRefresh)	//CBR wont work or I am stupid -_- clear the flag

	//First two digits of the LED

	LDI YL, low(spiMISO)		//Get the address of the buffer that we want to be displayed
	LDI YH, high(spiMISO)
	MOV R16, R13				//Get a special pointer that increments by timer
	ADD YL, R16
	CLR R16
	ADC YH, R16					//Add the pointer to the address with carry

	LD R16, Y					//Load a content of the ongoing buffer cell

	MOV YL, R16					//Digits of the byte should be separated 
	MOV YH, R16					//(a byte in hexadecimal form consists of two digits maximum)
	ANDI YL, 0b_0000_1111		//Mask high...
	ANDI YH, 0b_1111_0000		//...and low digit
	LSR YH
	LSR YH
	LSR YH
	LSR YH						//Shift high masked digit right 4 times
	MOV digitToDisp1, YH
	MOV digitToDisp2, YL		//Display both high and low digit separately on the LED

	//Last two digits of the LED

	MOV R16, R13				//Load the ordinal number of the cell, that displayed now (supra)

	MOV YL, R16
	MOV YH, R16
	ANDI YL, 0b_0000_1111		//Mask high and low digits
	ANDI YH, 0b_1111_0000
	LSR YH
	LSR YH
	LSR YH
	LSR YH						//Shift high masked digit right 4 times
	MOV digitToDisp3, YH
	MOV digitToDisp4, YL		//Display both high and low digit separately on the LED

	//Any possible errors check

	CPI digitToDisp1, 0x10	//if the 1st register...
	BRLO HH1				//...is more than F...
	SET						//...then set the T flag (which means "incorrect number")

	HH1:

		CPI digitToDisp2, 0x10	//if the 2nd one...
		BRLO HH2				//...is more than F...
		SET						//...then set the T flag (which means "incorrect number")

	HH2:

			CPI digitToDisp3, 0x10	//if the 3rd one...
			BRLO HH3				//...is more than F...
			SET						//...then set the T flag (which means "incorrect number")

	HH3:

				CPI digitToDisp4, 0x10	//if the 4th one...
				BRLO HH4				//...is more than F...
				SET						//...then set the T flag (which means "incorrect number")

	HH4:

	CPI digitToDisp1, 0x00	//same for less than 0
	BRPL LL1				//BRanch if PLus (if the N flag in SREG is cleared)
	SET

	LL1:

		CPI digitToDisp2, 0x00
		BRPL LL2			//BRanch if PLus (if the N flag in SREG is cleared)
		SET

	LL2:

			CPI digitToDisp3, 0x00
			BRPL LL3		//BRanch if PLus (if the N flag in SREG is cleared)
			SET

	LL3:

				CPI digitToDisp4, 0x00
				BRPL LL4			//BRanch if PLus (if the N flag in SREG is cleared)
				SET

	LL4:

	//Overflows check
	SBRC flagStorage, spiMOSIBufferOverflow
	SET

	SBRC flagStorage, spiMISOBufferOverflow
	SET

	SBRC flagStorage, uartTXBufferOverflow
	SET

	//Note that the error state (T flag) won't be resetted

	CPI R17, 0			//Is it the time to display 1st digit of LED?
	BREQ firstDigTeleport
	
	CPI R17, 1			//Is it the time to display 2nd digit of LED?
	BREQ secondDigTeleport

	CPI R17, 2			//Is it the time to display 3rd digit of LED?
	BREQ thirdDigTeleport

	CPI R17, 3			//Is it the time to display 4th digit of LED?
	BREQ fourthDigTeleport

notATimeToRefresh:

RJMP Start		//Go to start
//End of Main Routine//

;--------------------------------------------------------------------------------------------

//Decoding the value of R12//
Decode:		//if one of 4 digits is chosen, then select a sign to be displayed

LSL R12							//Logical Shift Left: a number gets multiplied by 2 (e.g. 0011<<1 == 0110, 3*2=6)
LDI ZL, Low(decAddrTable*2)		//Put the low part of the table of addresses' address into Z
LDI ZH, High(decAddrTable*2)	//Same for the high one
//Note that the preprocessing of the assembler interpretes addresses as words (for using in program counter)
//And, in order to appeal to specific bytes (not the whole word), we should multiply an address by 2

CLR R16			//CLeaRing the R16
ADD ZL, R12		//Adding the "offset" to the address of the table of addresses
ADC ZH, R16		//If there was an overflow one string upper ^, "C" flag appears...
				//...So we should handle this flag by ADding zero with Carry
//Now Z points to the beginning of the table PLUS number of cells defined by R12
//After all, Z points exactly to desired address in the table

LPM YL, Z+	//Load (from Program Memory) a content of the cell Z points to. And increment Z.
LPM YH, Z	//Next part of final destination address
//LPM command works with bytes, not with words, remember?

MOVW ZH:ZL, YH:YL	//now a desired address goes into Z

IJMP		//go to address of a desired subsequence
//http://easyelectronics.ru/avr-uchebnyj-kurs-vetvleniya.html

RJMP Start	//Go to start of the Main Routine <--- probably, now with index jumping, this string is useless
;--------------------------------------------------------------------------------------------

firstDig:
LDI R16, 0b_0000_0001	//Turn on PC0 (1st digit)
OUT PORTC, R16
	BRTS dispE				//If the number is incorrect, display the "E" letter ("Err-")
MOV R12, digitToDisp1	//Just put an appropriate number (that should be lit) in R12
RJMP decode				//Go to specific digit displaying

secondDig:
LDI R16, 0b_0000_0010	//Turn on PC1 (2nd digit)
OUT PORTC, R16
	BRTS dispR				//If the number is incorrect, display the "r" letter ("Err-")
MOV R12, digitToDisp2	//Just put an appropriate number (that should be lit) in R12
RJMP decode				//Go to specific digit displaying

thirdDig:
LDI R16, 0b_0000_0100	//Turn on PC2 (3rd digit)
OUT PORTC, R16
	BRTS dispR				//If the number is incorrect, display the "r" letter ("Err-")
MOV R12, digitToDisp3	//Just put an appropriate number (that should be lit) in R12
RJMP decode				//Go to specific digit displaying

fourthDig:
LDI R16, 0b_0000_1000	//Turn on PC3 (4th digit)
OUT PORTC, R16
	BRTS dispDash			//If the number is incorrect, display dash ("Err-")
MOV R12, digitToDisp4	//Just put an appropriate number (that should be lit) in R12
RJMP decode				//Go to specific digit displaying
;--------------------------------------------------------------------------------------------

disp0:
LDI R16, segind0	//displays 0...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp1:
LDI R16, segind1	//displays 1...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp2:
LDI R16, segind2	//displays 2...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp3:
LDI R16, segind3	//displays 3...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp4:
LDI R16, segind4	//displays 4...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp5:
LDI R16, segind5	//displays 5...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp6:
LDI R16, segind6	//displays 6...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp7:
LDI R16, segind7	//displays 7...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp8:
LDI R16, segind8	//displays 8...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

disp9:
LDI R16, segind9	//displays 9...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

dispA:
LDI R16, segindA	//displays A...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

dispB:
LDI R16, segindB	//displays B...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

dispC:
LDI R16, segindC	//displays C...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

dispD:
LDI R16, segindD	//displays D...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

dispE:
LDI R16, segindE	//displays E...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

dispF:
LDI R16, segindF	//displays F...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

;---

dispR:
LDI R16, segindR	//displays R...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start

;---

dispDash:
LDI R16, segindDash	//displays "-"...
OUT PORTA, R16		//...on the LED indicator
RJMP Start			//Get back to the start