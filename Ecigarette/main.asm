; Created: 25.05.2016 14:12:47

.include "tn13adef.inc"

.device attiny13a

//Symbolic custom registers names

.def overflowsCounterOffMode	 =	R22		//counter of time; continuously compared with a specific preset - time of both buttons held down
.def currentPower				 =	R23		//current PWM duty cycle; value from 0 to 4
.def overflowsCounterBlink		 =	R24		//counter of time; continuously compared with a specific preset - LED blinking
.def overflowsCounterBlockAdj	 =	R25		//counter of time; continuously compared with a specific preset - contact bouncing cutoff

.def flagStorage = R19			//Custom flags storage
								//And custom symbolic bit names
.equ blockAdj	=	0			//State of buttons (1 if blocked) - contact bouncing cutoff
.equ ledState	=	1			//State of the LED
.equ offMode	=	2			//1 if 'off mode' is enabled

//Constants definition

//Buttons' pins
.equ sb1	=	3	//SB1 is attached to the Port B, pin 3
.equ sb2	=	4	//SB2 is attached to the Port B, pin 4
.equ sb3	=	1	//SB3 is attached to the Port B, pin 1
.equ ledPin	=	2	//LED is attached to the Port B, pin 2 (SCK)
.equ pwmPin	=	0	//Transistor is attached to the Port B, pin 0 (MOSI)

//Timings
.equ blink			=	0x04		//0x04 = 54.4 ms	on 9.6 MHz, prescaler=64, system divider=8 
.equ buttonCutoff	=	0x0F		//0x0B = 149.6 ms	on 9.6 MHz, prescaler=64, system divider=8
.equ offModeTime	=	0xFF		//0x0F = 204 ms		on 9.6 MHz, prescaler=64, system divider=8
									//0xFF = 3.468 sec	on 9.6 MHz, prescaler=64, system divider=8

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

;--------------------------------------------------------------------------------------------
.CSEG
//Reset and Interrupt Vectors table

	.ORG 0x0000	;(RESET) 
	RJMP Reset

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

.ORG INT_VECTORS_SIZE	;end of table

;--------------------------------------------------------------------------------------------
//Interrupts Handler//
;-------------------

//Timer/Counter0 Overflow

Timer0Over:		

PUSHSREG
 PUSH YL
  PUSH YH
   PUSH R17						//Save SREG and all used registers in stack

//LED blinking algorithm:

INC overflowsCounterBlink		//Increment the counter

SBRC flagStorage, ledState		//Skip next instruction if the LED is off
RJMP ledIsOn	

	UIN R16, OCR0A				//Read the value of PWM duty cycle...
	COM R16						//...invert it...
	LSR R16						//and divide by 2

	CP overflowsCounterBlink, R16	//the result of previous operations is the duration of switched off state of the LED
	BRLO endBlink					//The timeout is not over yet; exit the algorithm

		CLR overflowsCounterBlink		//When the timeout is over, clear the counter...
		ORI flagStorage, (1<<ledState)	//...clear the flag of the LED's state...
		CBI PORTB, 2					//...turn the LED off...
		RJMP endBlink					//...and exit the algorithm
	
ledIsOn:		//if the LED is on:

	CPI overflowsCounterBlink, blink		//compare the counter with the turned-on-state time preset
	BRLO endBlink							//if not reached yet, exit the algorithm
		CLR overflowsCounterBlink			//When the timeout is over, clear the counter...
		ANDI flagStorage, ~(1<<ledState)	//...set the flag of the LED's state...
		SBI PORTB, 2						//...and turn the LED off

endBlink:

//End of the LED blinking algorithm.

//Contact bouncing cutoff algorithm:

SBRS flagStorage, blockAdj			//If the corresponding flag is set, enter the algorithm
RJMP endOfBlockAdj

	INC overflowsCounterBlockAdj	//Increment the counter

	CPI overflowsCounterBlockAdj, buttonCutoff	//If it's the time to clear the flag that blocks the buttons?
	BRLO endOfBlockAdj							//if not, exit the algorithm

		CLR overflowsCounterBlockAdj			//Clear the counter
		ANDI flagStorage, ~(1<<blockAdj)		//Clear the flag

endOfBlockAdj:

//End of the contact bouncing cutoff algorithm.

//Algorithm of entering the 'off mode':

UIN R16, PINB					//Detecting if any of the adjusting buttons are pressed
COM R16							//Inversion (buttons are active low)
ANDI R16, (1 << sb1 | 1 << sb2)		//Clear all the bits except the adjusting buttons
CPI R16, (1 << sb1 | 1 << sb2)		//If both buttons are pressed...
BRNE endOfTimerInt

	INC overflowsCounterOffMode	//...then increment the 'off mode' counter

	CPI overflowsCounterOffMode, offModeTime	//Compare the counter with the preset
	BRLO endOfTimerInt							//If it's not the time, then exit

		CLR overflowsCounterOffMode		//Or else, clear the counter
		SBRS flagStorage, offMode		//Skip next instruction if 'off mode' is active
		RJMP offModeDisabled

		ANDI flagStorage, ~(1 << offMode)	//Disable 'off mode'
		RJMP endOfTimerInt

offModeDisabled:		//If 'off mode' disabled...
		
		ORI flagStorage, (1 << offMode)	//...then enable 'off mode'

endOfTimerInt:

//End of entering the 'off mode' algorithm.

   POP R17	//Extract registers from the stack
  POP YH
 POP YL
POPSREG

RETI

;-------------------

PinToggle:

PUSHSREG

UIN R16, PINB						//Detecting if any of the adjusting buttons are pressed
COM R16								//Inversion (buttons are active low)
ANDI R16, (1 << sb1 | 1 << sb2)		//If no buttons is pressed, then the int is generated by button release
BREQ blockAndExit					//Jump to setting the flag 'blockAdj' and then exit

SBRC flagStorage, blockAdj		//If contact bouncing cutoff disabled, then enter the algorithm
RJMP endPowerAdj				//else exit from the interrupt

	UIN R16, PINB
	ANDI R16, (1 << sb2)		//Detecting the SB2 button state
	BRNE notSB2					//If R16 is zero now, then SB2 is pressed

		//SB2 IS PRESSED

		ORI flagStorage, (1 << blockAdj)	//Set the flag that forbids further interrupt algorithm executions
		INC currentPower					//Increment the 'currentPower' pointer
		CPI currentPower, 0x05				//Compare it with 5 that is over limit
		BRLO blockAndExit					//If the pointer equals 5 then proceed to the next instructions, else jump

			LDI currentPower, 0x04			//Load the pointer with the maximum possible value
			RJMP blockAndExit				//Exit from the interrupt

	notSB2:

	UIN R16, PINB
	ANDI R16, (1 << sb1)		//Detecting the SB1 button's state
	BRNE endPowerAdj			//If R16 is zero now, then SB1 is pressed

		//SB1 IS PRESSED

		ORI flagStorage, (1 << blockAdj)	//Set the flag that forbids further interrupt algorithm executions
		SUBI currentPower, 0x01				//Decrement the 'currentPower' pointer
		BRCS minReached						//If the pointer is below zero, then jump
		RJMP blockAndExit					//...else exit from the interrupt

			minReached:
			LDI currentPower, 0x00			//Load the pointer with the minimum possible value
			RJMP blockAndExit

endPowerAdj:

POPSREG		//Extract SREG and R16 from stack

RETI		//Exit from the interrupt

blockAndExit:
	ORI flagStorage, (1 << blockAdj)		//Forbid further button processing in order to cut off contact bouncing
	RJMP endPowerAdj

;-------------------

//End of Interrupts Handler//

;--------------------------------------------------------------------------------------------
//Storage of static data in flash

//Values of the desired PWM duty cycle

fixedLevels:	.db 0x4D, 0x7F, 0xB3, 0xDA, 0xFF, 0x00

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
LDI R16, RAMEND
UOUT SPL, R16

//------------------------------------

//Timer initialization
LDI R16, (1 << TOIE0)		//set TOIE0 in TIMSK0 register (Timer/Counter0 Overflow Interrupt Enable)
UOUT TIMSK0, R16			//interrupts enabling register

	//Timer0
	
	//Bits 3 and 2 are reserved
	LDI R16,	(1 << COM0A1 | 0 << COM0A0 | 0 << COM0B1 | 0 << COM0B0 | 0 << 3 | 0 << 2 | 1 << WGM01 | 1 << WGM00)
	UOUT TCCR0A, R16	//Fast PWM mode

	//Bits 5 and 4 are reserved
	LDI R16,	(0 << FOC0A | 0 << FOC0B | 0 << 5 | 0 << 4 | 0 << WGM02 | 0 << CS02 | 1 << CS01 | 1 << CS00)
	UOUT TCCR0B, R16	//clock/64, with system prescaler = 8 equals 13.6 ms

//------------------------------------
//external interrupt init

//PCIE: Pin Change Interrupt Enable
//Bit 7 and bits 4-0 are reserved
LDI R16,	(0 << 7 | 0 << INT0 | 1 << PCIE | 0 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0)
UOUT GIMSK, R16		//General Interrupt Mask Register - enable toggle int

//Bits 7 and 6 are reserved
LDI R16,	(0 << 7 | 0 << 6 | 0 << PCINT5 | 1 << PCINT4 | 1 << PCINT3 | 0 << PCINT2 | 0 << PCINT1 | 0 << PCINT0)
UOUT PCMSK, R16		//pin 3 and 4 toggling contribute to the int. activation

//------------------------------------
//gpio init

LDI R16, (1 << ledPin | 0 << pwmPin)	//Configure LED pin as output (PWM pin disabled by default)
UOUT DDRB, R16

LDI R16, (1 << ledPin)		//tri-stated inputs, low level on outputs. LED is off by default
UOUT PORTB, R16

//------------------------------------

CLR R16		//clear R16 for the order's sake

SEI			//interrupts enabled globally

;--------------------------------------------------------------------------------------------

//Main Routine//
Start:

SBRC flagStorage, offMode	//If 'off mode' is active, then keep PWM pin disabled
RJMP disablePWM

UIN R16, PINB			//Read the button state
ANDI R16, (1 << sb3)	//Clear all the bits except the SB3 button state
BRNE disablePWM			//If not a zero, then SB3 is released; so go to PWM pin disabling

	//SB3 is pressed
	UIN R16, DDRB			//Read the direction register value
	ORI R16, (1 << pwmPin)	//Enable PWM pin
	UOUT DDRB, R16			//Write the direction register
	
RJMP exitPWMEnablingAlgorithm	//Exit the algorithm

disablePWM:

	//SB3 is not pressed
	UIN R16, DDRB				//Read the direction register value
	ANDI R16, ~(1 << pwmPin)	//Disable PWM pin
	UOUT DDRB, R16				//Write the direction register

exitPWMEnablingAlgorithm:

//---

LDI ZL, low(fixedLevels * 2)	//Get the location of the fixed PWM levels values
LDI ZH, high(fixedLevels * 2)	//Assembler refers to flash in 2-byte words, so the multiplication by 2 is required!
MOV R16, currentPower			//Get the current PWM duty cycle value
ADD ZL, R16						//Use it as an offset in memory
CLR R17
ADC ZH, R17						//Addition with carry
LPM R16, Z						//Read a value from flash memory

UOUT OCR0A, R16					//Set this value as PWM duty cycle

//---

RJMP Start			//Go to start
//End of Main Routine//
