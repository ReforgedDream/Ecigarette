; Created: 25.05.2016 14:12:47

.include "tn13adef.inc"

.device attiny13a

//Constants definition

//Symbolic custom registers names
.def currentPower				 =	R23
.def overflowsCounterBlink		 =	R24
.def overflowsCounterBlockAdj	 =	R25

.def flagStorage = R19			//Custom flags storage
								//And custom symbolic bit names, why not
.equ blockAdj	=	0
.equ ledState	=	1

.equ blink = 0x04				//0x04 = 55 ms on 8 MHz, prescaler=64, system divider=8 
.equ buttonCutoff = 0x0B		//0x07 = 100 ms on 8 MHz, prescaler=64, system divider=8

.equ owfPerSecond8BitHigh = 0x00
.equ owfPerSecond8BitLow = 0x49		//8 MHz, prescaler=64, system divider=8 -> ~73 (0x0049) 8-bit overflows/sec

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
	RETI //RJMP PinToggle
	
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

Timer0Over:

PUSHSREG
 PUSH YL
  PUSH YH
   PUSH R17
   
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

SBRS flagStorage, blockAdj			//If blockAdj==0 then skip
RJMP endOfTimerInt

	INC overflowsCounterBlockAdj

	CPI overflowsCounterBlockAdj, buttonCutoff
	BRLO endOfTimerInt	

		CLR overflowsCounterBlockAdj
		ANDI flagStorage, ~(1<<blockAdj)

endOfTimerInt:

   POP R17
  POP YH
 POP YL
POPSREG

RETI

;-------------------
/*
PinToggle:

PUSHSREG

	UIN R16, PINB
	ANDI R16, (1<<4)
	BRNE label3
	//SB2 PRESSED
		UIN R16, OCR0A
		CPI R16, 0x0E
		BRCC label4
			SUBI R16, 0x0D
			UOUT OCR0A, R16
			RJMP label4
	label3:
		UIN R16, OCR0A
		//CPI R16, 0xF1
		//BRGE label4
			SUBI R16, -0x0D
		UOUT OCR0A, R16
	//SB2 IS NOT PRESSED

	label4:

POPSREG

RETI
*/


//End of Interrupts Handler//

;--------------------------------------------------------------------------------------------
//Storage of static data in flash
testcrap1:		.db 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C
fixedLevels:	.db 0x4D, 0x7F, 0xB3, 0xDA, 0xFF, 0x8C
testcrap2:		.db 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C

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
LDI R16, 0b_0000_0010	//interrupts
UOUT TIMSK0, R16			//set TOIE0 in TIMSK0 register (overflow enabled)

	//Timer2
	LDI R16, 0b_0000_0100	//PWM pin disabled by default
	UOUT DDRB, R16

	LDI R16, 0b_1000_0011	//Fast PWM mode
	UOUT TCCR0A, R16

	LDI R16, 0b_0000_0011	//clock/64
	UOUT TCCR0B, R16

	;LDI R16, 0x7F
	;UOUT OCR0A, R16
//------------------------------------
//external interrupt init
/*
LDI R16, 0b_0010_0000	//enable toggle int
UOUT GIMSK, R16

LDI R16, 0b_0001_1000	//pin 3 and 4 toggling contribute to the int. activation
UOUT PCMSK, R16
*/
//------------------------------------
//gpio init

LDI R16, 0b_0000_0100		//tri-stated inputs, low level on outputs
UOUT PORTB, R16

//------------------------------------

CLR R16		//clear R16 for the order's sake

;LDI currentPower, 0x01

SEI			//interrupts enabled globally

;--------------------------------------------------------------------------------------------

//Main Routine//
Start:

UIN R16, PINB
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


//LDI R16, 0b_0000_0101	//PWM pin enabled
//	UOUT DDRB, R16			///TEST!!!
//---

SBRC flagStorage, blockAdj			//If blockAdj==1 then skip
RJMP endPowerAdj

	UIN R16, PINB
	ANDI R16, (1<<4)
	BRNE notSB2

		//SB2 IS PRESSED
		ORI flagStorage, (1<<blockAdj)
		INC currentPower
		CPI currentPower, 0x05
		BRGE maxReached
		RJMP endPowerAdj

			maxReached:
			LDI currentPower, 0x04
			RJMP endPowerAdj

	notSB2:

	UIN R16, PINB
	ANDI R16, (1<<3)
	BRNE endPowerAdj

		//SB1 IS PRESSED
		ORI flagStorage, (1<<blockAdj)
		//CLR R17
		//CPSE currentPower, R17
		DEC currentPower
		BRCS minReached
		RJMP endPowerAdj

			minReached:
			LDI currentPower, 0x00
			
endPowerAdj:

//---

LDI ZL, low(fixedLevels)
LDI ZH, high(fixedLevels)
MOV R16, currentPower
ADD ZL, R16
CLR R17
ADC ZH, R17
LPM R16, Z

UOUT OCR0A, R16

RJMP Start		//Go to start
//End of Main Routine//
