; SimpleRobotProgram.asm
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.

ORG 0  ; Begin program at x000
;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)

WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

;***************************************************************
;* Main code
;***************************************************************
Main: 
	OUT		RESETPOS

Start:
	IN 		THETA
	OUT		LCD
	
	CALL	Move2Feet
	CALL 	TurnRight
	CALL    Wait1
	CALL 	TurnRight
	CALL    Wait1
	CALL 	TurnRight
	CALL    Wait1
	CALL 	TurnRight
	CALL    Wait1
	
	CALL	Back2Feet
	CALL    Wait1
	CALL 	TurnLeft
	CALL    Wait1
	CALL 	TurnLeft
	CALL    Wait1
	CALL 	TurnLeft
	CALL    Wait1
	CALL 	TurnLeft
	
		
;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

	
Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero         ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD         ; An indication that we are dead
	OUT    SSEG2
Forever:
	JUMP   Forever      ; Do this forever.
	DEAD:  DW &HDEAD    ; Example of a "local" variable
	
;***************************************************************
;* Our Subroutines
;***************************************************************
; Subroutine to move forward
Move2Feet:
		IN		LPOS
		ADDI 	TwoFeet
		STORE	target
		JUMP    CheckStop1
		
CheckStop1:
		LOAD    FMid
		OUT 	LVELCMD
		OUT		RVELCMD

		IN		LPOS	
		OUT		SSEG1
		IN		RPOS
		OUT		SSEG2
		IN 		THETA
		OUT		LCD
		
		IN		LPOS
		SUB		target
		JNEG	CheckStop1
		JUMP	Stop1
Stop1:
		LOAD 	Zero
		STORE	target
		OUT		LVELCMD
		OUT		RVELCMD
		RETURN
		
; Subroutine to move back two feet	
Back2Feet:
		IN		LPOS
		SUB 	TwoFeet
		STORE	target
		JUMP    CheckStop2
CheckStop2:
		LOAD    RMid
		OUT 	LVELCMD
		OUT		RVELCMD

		IN		LPOS	
		OUT		SSEG1
		IN		RPOS
		OUT		SSEG2
		IN 		THETA
		OUT		LCD
		
		IN		LPOS
		SUB		target
		JPOS	CheckStop2
		JUMP	Stop2
Stop2:
		LOAD 	Zero
		STORE	target
		OUT		LVELCMD
		OUT		RVELCMD
		RETURN

; Subroutine to turn left
TurnLeft:
        IN      THETA
        ADDI    -270
        OUT		LCD
        JNEG    Ln1
        JZERO	Ln1
        JUMP    Lr1

Ln1:
        IN      THETA
        ADDI    90
        STORE   target
Ln2:      
        LOAD 	RTurn
        OUT		LVELCMD
       	LOAD 	FTurn
		OUT 	RVELCMD	
		
        IN      THETA
        SUB     target
        JNEG    Ln2
        JZERO   EndTurnL
        JUMP    EndTurnL

Lr1:
        IN      THETA
        ADDI    -270
        STORE   target
Lr2:
        LOAD 	RTurn
        OUT		LVELCMD
       	LOAD 	FTurn
		OUT 	RVELCMD	
		
        IN      THETA
        ADDI    -270
        
        JPOS	FixLeft
        IN		THETA
        JUMP	Lr3
FixLeft:
		IN		THETA
		ADDI	-360
Lr3:		  
        SUB     target
        JNEG    Lr2
        JZERO   EndTurnL
        JUMP    EndTurnL

EndTurnL:
		LOAD	FStop
		OUT		LVELCMD
		LOAD	RStop
		OUT		RVELCMD
		LOAD	Zero
		STORE	target
        RETURN
        
; Subroutine to turn right
TurnRight:
		IN      THETA
        ADDI	-90
        JPOS    Rn1
        JZERO   Rr1
        JUMP    Rr1

Rn1:
        IN      THETA
        ADDI    -90
        STORE   target
Rn2:
        LOAD	FTurn
        OUT		LVELCMD
        LOAD 	RTurn
        OUT		RVELCMD
        
        IN      THETA
        SUB     target
        JPOS    Rn2
        JZERO   EndTurnR
        JUMP    EndTurnR

Rr1:
        IN      THETA
        ADDI    270
        STORE   target
Rr2:       
        LOAD	FTurn
        OUT		LVELCMD
        LOAD 	RTurn
        OUT		RVELCMD
        
        IN      THETA
        ADDI	-90
        
        JNEG	FixRight
        IN		THETA
        JUMP	Rr3		
FixRight:
		IN		THETA        
        ADDI    360
Rr3:
        SUB     target
        JPOS    Rr2
        JZERO   EndTurnR
        JUMP    EndTurnR

EndTurnR:
		LOAD	RStop
		OUT		LVELCMD
		LOAD	FStop
		OUT		RVELCMD
		LOAD	Zero
		STORE	target
        RETURN

;***************************************************************
;* Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     LIN
	OUT    SSEG2
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

;***************************************************************
;* Variables
;***************************************************************
Temp:     	DW 	0 ; "Temp" is not a great name, but can be useful
target: 	DW  0 ; Target theta abgle to reach while turning 

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500
FTurn:	  DW 150	   ; Turning speed
RTurn: 	  DW -150
FStop:    DW 30		   ; Stopping speed (locks the rotors)
RStop:	  DW -30

MinBatt:  DW 110       ; 11.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Current Y-position (read only)
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9
