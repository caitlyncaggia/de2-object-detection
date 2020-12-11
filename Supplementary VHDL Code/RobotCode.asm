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
	OUT    BEEP        ; Stop any beeping

	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display batt voltage on LCD

	CALL   UARTNL      ; send a couple newlines to make a break in the log
	CALL   UARTNL

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
Start:
        LOAD Zero			;reset position
        OUT RESETPOS
        ADDI &B00100001		;enable sonar 0 and 5
        OUT	SONAREN
        
        ADDI	1
		OUT 	BEEP
		LOAD	Zero
		OUT		BEEP
        
		CALL MoveForward	;enter the arena
		CALL Wait1
		JUMP Main
		
Main:

		CALL MainSonar
		CALL MoveForward	
		CALL Wait1
		
		CALL MainSonar
		CALL MoveForward	
		CALL Wait1
		
		CALL MainSonar
		CALL MoveForward	
		CALL Wait1
		
		CALL MainSonar
		
		CALL	Die


MainLoop:
		LOAD LoopCount
		ADDI -1				;loop runs once per row, stops after 4th row (set to 1 for debugging now)
		JZERO Done
		CALL MainSonar
        LOAD RFlag			;if the robot sees the right wall, RFlag = 1
        CALL BranchRight
        LOAD LFlag			;if the robot sees the left wall, LFlag = 1
		CALL BranchLeft
		CALL MoveForward
		LOAD LoopCount
		ADDI 1				;increment loop counter
		STORE LoopCount
		JUMP MainLoop		;repeat

BranchRight:
		JZERO EarlyReturn	;if an object was found on that row, return
		LOAD RightCount		;check how many times robot has gone right
        ADDI -2				;checking twice per side is enough
        JZERO EarlyReturn
		CALL TurnRight		;advance to the right tile
		CALL MoveForward	;move to tile closest to wall
		CALL BranchSonarRR	;check for objects
		CALL BranchSonarRL
		CALL MoveBackward	;return to the center aisle
		CALL ReverseRight
		LOAD RightCount		;mark that we looked right once
		ADDI 1
		STORE RightCount
		RETURN

BranchLeft:
		JZERO EarlyReturn	;if an object was found on that row, return
		LOAD LeftCount		;check how many times robot has gone left
        ADDI -2				;checking twice per side is enough
        JZERO EarlyReturn
		CALL TurnLeft		;advance to the left tile
		CALL MoveForward	;move to tile closest to wall
		CALL BranchSonarLR	;check for objects
		CALL BranchSonarLL
		CALL MoveBackward	;return to the center aisle
		CALL ReverseLeft
		LOAD LeftCount		;mark that we looked left once
		ADDI 1
		STORE LeftCount
		RETURN

EarlyReturn:
	RETURN

Done:
	CALL SendObjCount

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

;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
;MAIN SONAR SUBROUTINE

MainSonar:
	;make sure everything is set back to 0
	LOAD Zero
	STORE ObjDist
	STORE OBJX
	STORE OBJY
	STORE RFlag
	STORE LFlag
	
	ADDI	1
	OUT 	BEEP

	CALL MainCheckRight
	CALL	Wait1
	CALL MainCheckLeft
	RETURN

MainCheckRight:
	;Check the right side of the aisle
	LOAD Zero		;reset object distance to 0
	STORE ObjDist
	IN DIST5
	STORE ObjDist
	ADDI -610		;check if 2 feet away (1 tile)
	JZERO MainOneTileR
	JNEG MainOneTileR
	ADDI -610		;check if 4 feet away (2 tiles)
	JZERO MainTwoTileR
	JNEG MainTwoTileR
	ADDI -610		;check if 6 feet away (wall)
	JZERO MainWallR
	JNEG MainWallR

MainOneTileR:
	LOAD Zero
	ADDI 1
	STORE ObjDist
	
	LOAD	Zero
	ADDI	1
	OUT 	BEEP
	
	JUMP MainObjectPosition

MainTwoTileR:
	LOAD Zero
	ADDI 2
	STORE ObjDist
	JUMP MainObjectPosition

MainWallR:
	LOAD RFlag
	ADDI 1
	STORE RFlag
	RETURN

MainCheckLeft:
	;Check the left side of the aisle
	LOAD Zero		;reset object distance to 0
	STORE ObjDist
	IN DIST0
	ADDI -610		;check if 2 feet away (1 tile)
	JZERO MainOneTileL
	JNEG MainOneTileL
	ADDI -610		;check if 4 feet away (2 tiles)
	JZERO MainTwoTileL
	JNEG MainTwoTileL
	ADDI -610		;check if 6 feet away (wall)
	JZERO MainWallL
	JNEG MainWallL

MainOneTileL:
	LOAD Zero
	ADDI -1
	STORE ObjDist
	
	LOAD	Zero
	ADDI	1
	OUT 	BEEP
	
	JUMP MainObjectPosition

MainTwoTileL:
	LOAD Zero
	ADDI -2
	STORE ObjDist
	JUMP MainObjectPosition

MainWallL:
	LOAD LFlag
	ADDI 1
	STORE LFlag
	RETURN

MainObjectPosition:
	;if object's distance is 0, there is no object
	LOAD ObjDist
	JZERO EarlyReturn

	;object's y-value is same as robot's y value
	LOAD RealY ;robot's x and y are 'flipped'
	STORE OBJY
	
	;object's x-value differs by object distance
	LOAD RealX
	ADD ObjDist
	STORE OBJX
	
	
	CALL RecordObject
	
	LOAD	Zero
	OUT 	BEEP
	
	RETURN


;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
;BRANCH SONAR SUBROUTINES

BranchSonarRR:
	;after making a right turn, check for objects on the right
	LOAD Zero 	;reset object distance
	STORE ObjDist
	IN DIST5	;look at right sonar (below)
	CALL CheckBelow
	RETURN

BranchSonarRL:
	;after making a right turn, check for objects on the left
	LOAD Zero	;reset object distance
	STORE ObjDist
	IN DIST0	;look at left sonar (above)
	CALL CheckAbove
	RETURN

BranchSonarLR:
	;after making a left turn, check for objects on the right
	LOAD Zero	;reset object distance
	STORE ObjDist
	IN DIST5	;look at right sonar (above)
	CALL CheckAbove
	RETURN

BranchSonarLL:
	;after making a left turn, check for objects on the left
	LOAD Zero	;reset object distance
	STORE ObjDist
	IN DIST0	;look at left sonar (below)
	CALL CheckBelow
	RETURN

CheckAbove:
	;Check for objects above robot
	ADDI -610		;check if 2 feet away (1 tile)
	JZERO OneTileA
	JNEG OneTileA
	ADDI -610		;check if 4 feet away (2 tiles)
	JZERO TwoTileA
	JNEG TwoTileA
	ADDI -610		;check if 6 feet away (3 tiles)
	JZERO ThreeTileA
	JNEG ThreeTileA
	RETURN

OneTileA:
	LOAD YVAR ;check if the robot is against top wall (Y = 4)
	ADDI -4
	JZERO EarlyReturn
	LOAD ObjDist
	ADDI 1
	STORE ObjDist
	JUMP ObjectPosition

TwoTileA:
	LOAD YVAR ;check if the robot is 1 away from top wall (Y = 3)
	ADDI -3
	JZERO EarlyReturn
	LOAD ObjDist
	ADDI 2
	STORE ObjDist
	JUMP ObjectPosition

ThreeTileA:
	LOAD YVAR ;check if the robot is 2 away from top wall (Y = 2)
	ADDI -2
	JZERO EarlyReturn
	LOAD ObjDist
	ADDI 3
	STORE ObjDist
	JUMP ObjectPosition

CheckBelow:
	;Check for objects above robot
	CALL FindRobot	;update XVAR and YVAR
	ADDI -610		;check if 2 feet away (1 tile)
	JZERO OneTileB
	JNEG OneTileB
	ADDI -610		;check if 4 feet away (2 tiles)
	JZERO TwoTileB
	JNEG TwoTileB
	ADDI -610		;check if 6 feet away (3 tiles)
	JZERO ThreeTileB
	JNEG ThreeTileB
	RETURN

OneTileB:
	LOAD YVAR ;check if the robot is against bottom wall (Y = 1)
	ADDI -1
	JZERO EarlyReturn
	LOAD ObjDist
	ADDI -1
	STORE ObjDist
	JUMP ObjectPosition

TwoTileB:
	LOAD YVAR ;check if the robot is one away from bottom wall (Y = 2)
	ADDI -2
	JZERO EarlyReturn
	LOAD ObjDist
	ADDI -2
	STORE ObjDist
	JUMP ObjectPosition

ThreeTileB:
	LOAD YVAR	;check if robot is two away from bottom wall (Y = 3)
	ADDI -3
	JZERO EarlyReturn
	LOAD ObjDist
	ADDI -3
	STORE ObjDist
	JUMP ObjectPosition

ObjectPosition:
	;if object's distance is 0, there is no object
	LOAD ObjDist
	JZERO EarlyReturn

	;otherwise, object's x-value is same as robot's x value
	CALL FindRobot
	LOAD RealX
	STORE OBJX
	;object's y-value differs by object's distance
	LOAD RealY
	ADD ObjDist
	STORE OBJY
	CALL RecordObject
	RETURN

FindRobot:
	CALL ConvertFeet ;get current position in feet
	LOAD XVAR
	SHIFT -1	;divide by 2 to get tile position
	ADDI	1
	STORE XVAR
	STORE	RealY
	;STORE	OBJY
	
	LOAD YVAR
	SHIFT -1	;divide by 2 to get tile position
	STORE YVAR
	LOAD	Zero
	ADDI	3
	SUB		YVAR
	STORE	RealX
	;STORE	OBJX
	LOAD	Zero
	
	;CALL 	SendObjCoord
	
	RETURN

ConvertFeet:
	; Send robot coordinates in feet.  The convertion results in
	; the truncation of the division (i.e. remainder discarded).
	IN     XPOS
	STORE  d16sN       ; numerator of division subroutine
	LOAD   OneFoot
	STORE  d16sD       ; denominator
	CALL   Div16s      ; 16-bit signed division subroutine
	LOAD   dres16sQ    ; quotient result
	STORE  XVAR        ; this will be saved as robot's X
	
	IN     YPOS        ; note that this could easily be negative
	STORE  d16sN       ; numerator of division subroutine
	LOAD   OneFoot
	STORE  d16sD       ; denominator
	CALL   Div16s      ; 16-bit signed division subroutine
	LOAD   dres16sQ    ; quotient result
	STORE  YVAR        ; this will be saved as robot's Y
	RETURN


;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
;MOVEMENT SUBROUTINES

MoveForward:
	IN		THETA
	STORE	target
	CALL	decideAngle
	ADDI	8
	STORE  	DesTheta    ; desired heading current
	LOAD    FMid
	STORE  	DesVelL      ; desired velocity (medium forward)
	STORE  	DesVelR      ; desired velocity (medium forward)
	IN		LPOS
	ADD		Str2Feet
	STORE	TargetL
MoveF2:
	CALL	SetMovement
CheckDistF:
	IN      LPOS
	SUB     TargetL
	JNEG    MoveF2      ; not there yet; keep moving

	; Stop at this point
	LOAD    Zero
	STORE   DesTheta    ; desired heading (0 degrees)
	OUT 	LVELCMD
	OUT 	RVELCMD
	CALL	FindRobot
	RETURN

; Subroutine to Move Backward 2 feet
MoveBackward:
	IN		THETA
	STORE	target
	CALL	decideAngle
	STORE  	DesTheta    ; desired heading current
	LOAD    RMid
	STORE  	DesVelL      ; desired velocity (medium forward)
	STORE  	DesVelR      ; desired velocity (medium forward)
	IN		LPOS
	SUB		Str2Feet
	STORE	TargetL
MoveB2:
	CALL	SetMovement
CheckDistB:
	IN      LPOS
	SUB     TargetL
	JPOS    MoveB2      ; not there yet; keep moving

	; Stop at this point
	LOAD    Zero
	STORE   DesTheta    ; desired heading (0 degrees)
	OUT 	LVELCMD
	OUT 	RVELCMD
	RETURN

; Subroutine to turn left 90 degrees
TurnLeft:
        IN      THETA
        ADDI    -284
        OUT		LCD
        JNEG    Ln1
        JZERO	Ln1
        JUMP    Lr1

Ln1:
        IN      THETA
        ADDI    90
        CALL	decideAngle
        JPOS	Cont
        ADDI	360
Cont:
        ADDI	-14
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
        CALL	decideAngle
        ADDI	-14
        STORE   target
Lr2:
        LOAD 	RTurn
        OUT		LVELCMD
       	LOAD 	FTurn
		OUT 	RVELCMD

        IN      THETA
        ADDI    -286

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
		LOAD	Zero
		OUT		LVELCMD
		OUT		RVELCMD
		STORE	target

        RETURN


; Subroutine to turn right 90 degrees
TurnRight:
		IN      THETA
        ADDI	-76
        JPOS    Rn1
        JZERO   Rn1
        JUMP    Rr1

Rn1:
        IN      THETA
        ADDI    -90
        CALL	decideAngle
        ADDI	14
        STORE	target
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
        CALL	decideAngle
        ADDI	14
        STORE   target

Rr2:
        LOAD	FTurn
        OUT		LVELCMD
        LOAD 	RTurn
        OUT		RVELCMD

        IN      THETA
        ADDI	-76

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
		LOAD	Zero
		OUT		LVELCMD
		OUT		RVELCMD
		STORE	target

        RETURN

decideAngle:
    SUB     Deg45     ; if less than 45 degrees

    JZERO   faceNorth
    JNEG    faceNorth   ; decided to face north

    SUB     Deg90     ; if less than 135 degrees

    JZERO   faceWest
    JNEG    faceWest    ; decided to face west

    SUB     Deg90     ; if less than 225 degrees

    JZERO   faceSouth
    JNEG    faceSouth   ; decided to face south

    SUB     Deg90     ; if less than 315 degrees

    JZERO   faceEast
    JNEG    faceEast    ; decided to face east

    JUMP    faceNorth   ; if greater than 315 degrees, will turn north

faceNorth:
    LOAD    Zero        ; load the target angle of 0
    RETURN
faceWest:
    LOAD    Deg90     ; load the target angle of 90
    RETURN
faceSouth:
    LOAD    Deg180    ; load the target angle of 180
    RETURN
faceEast:
    LOAD    Deg270     ; load the target angle of 270
    RETURN              ; returns to the caller with the ...
                        ; ... correct target angle loaded
                      
	
; This is pretty good at correcting small motor control errors, but
; it is entirely based on the odometry, and thus can NOT correct
; for odometry error.
; It's designed to be used when already facing the correct direction.
; It doesn't work very well to GET to the right heading, though it
; will eventually.  A modified version could probably work better.
; This routine works by adding and subtracting the angular error
; from the desired wheel velocities to gently correct the robot
; back to the desired heading.
SetMovement:
	CALL   GetAngleError
	STORE  SMAE         ; save for later
	ADD    DesVelL
	CALL   CapVel       ; ensure not beyond max/min speed
	OUT    LVELCMD      ; left velocity
	LOAD   SMAE
	CALL   Inv          ; negate angle error
	ADD    DesVelR
	CALL   CapVel       ; ensure not beyond max/min speed
	OUT    RVELCMD      ; right velocity
	RETURN
GetAngleError:
	IN     Theta
	SUB    DesTheta     ; current - desired angle
GetAngleErrorAC:
	CALL   FoldAngle    ; convert to magnitude+direction
	RETURN
FoldAngle: ; fold +/-360 to +/-180
	ADDI   -180
	JNEG   FAL180
	ADDI   -180         ; If >180, subtract 360
	RETURN
FAL180:
	ADDI   360
	JPOS   FAOK
	ADDI   180          ; If <-180, add 360
	RETURN
FAOK:
	ADDI   -180         ; revert to original number
	RETURN
CapVel:
	STORE  SMV
	; velocities must not exceed +/-511
	JPOS   CVPos
	ADDI   511
	JNEG   CVMin
	JUMP   CVSafe
CVPos:
	ADDI   -511
	JPOS   CVMax
	JUMP   CVSafe
CVMax:
	LOADI  511
	RETURN
CVMin:
	LOADI  -511
	RETURN
CVSafe:
	LOAD   SMV
	RETURN

SMAE: DW 0 ; temporary storage for angle error
SMV: DW 0  ; temporary storage for  velocity

ReverseRight:
	CALL MoveBackward
	CALL TurnLeft
	RETURN

ReverseLeft:
	CALL MoveBackward
	CALL TurnRight
	RETURN


;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
;OBJECT DATA SUBROUTINES

;Use variables to store current object's x and y locations (OBJX and OBJY).
;Since we can send locations at any time, send locations each time an object is found.

RecordObject:
	;first check if already found... if yes, return
	;we know there are at most six possible objects...
CheckXList:
CheckX1:
	LOAD OBJX
	SUB OBJX1
	JZERO CheckY1
CheckX2:
	LOAD OBJX
	SUB OBJX2
	JZERO CheckY2
CheckX3:
	LOAD OBJX
	SUB OBJX3
	JZERO CheckY3
CheckX4:	
	LOAD OBJX
	SUB OBJX4
	JZERO CheckY4
CheckX5:	
	LOAD OBJX
	SUB OBJX5
	JZERO CheckY5
CheckX6:	
	LOAD OBJX
	SUB OBJX6
	JZERO CheckY6
	
	JUMP NotACopy
	
CheckY1:
	LOAD OBJY
	SUB OBJY1
	JZERO AlreadyFound
	JUMP CheckX2
CheckY2:
	LOAD OBJY
	SUB OBJY2
	JZERO AlreadyFound
	JUMP CheckX3
CheckY3:
	LOAD OBJY
	SUB OBJY3
	JZERO AlreadyFound
	JUMP CheckX4
CheckY4:
	LOAD OBJY
	SUB OBJY4
	JZERO AlreadyFound
	JUMP CheckX5
CheckY5:
	LOAD OBJY
	SUB OBJY5
	JZERO AlreadyFound
	JUMP CheckX6
CheckY6:
	LOAD OBJY
	SUB OBJY6
	JZERO AlreadyFound

	;if it doesn't match any saved coordinates...
	JUMP NotACopy
	;...send this object's coordinates
NotACopy:
	CALL SendObjCoord
	LOAD ObjCount
	ADDI 1
	STORE ObjCount
	CALL UpdateList

AlreadyFound:
	RETURN

UpdateList:
	;An object should never store a location of 0.
	;If any list has 0 as an entry, no object is stored there.
	;Fill the first empty slot with the new object's info...
	LOAD OBJX1
	JZERO EmptySlot1
	LOAD OBJX2
	JZERO EmptySlot2
	LOAD OBJX3
	JZERO EmptySlot3
	LOAD OBJX4
	JZERO EmptySlot4
	LOAD OBJX5
	JZERO EmptySlot5
	LOAD OBJX6
	JZERO EmptySlot6
	RETURN

EmptySlot1:
	LOAD OBJX
	STORE OBJX1
	LOAD OBJY
	STORE OBJY1
	RETURN

EmptySlot2:
	LOAD OBJX
	STORE OBJX2
	LOAD OBJY
	STORE OBJY2
	RETURN

EmptySlot3:
	LOAD OBJX
	STORE OBJX3
	LOAD OBJY
	STORE OBJY3
	RETURN

EmptySlot4:
	LOAD OBJX
	STORE OBJX4
	LOAD OBJY
	STORE OBJY4
	RETURN

EmptySlot5:
	LOAD OBJX
	STORE OBJX5
	LOAD OBJY
	STORE OBJY5
	RETURN

EmptySlot6:
	LOAD OBJX
	STORE OBJX6
	LOAD OBJY
	STORE OBJY6
	RETURN

;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
;COORDINATE TRANSMISSION

;Note: Sending object count stops the run.

; Subroutine to send object coordinates to base station.
; Format is '\nX', one byte for X, 'Y', and one byte for Y

SendObjCoord:
	CALL   UARTNL      ; send a newline
	LOAD   AsciiX      ; send ascii 'X'
	CALL   UARTRAW
	LOAD   OBJX        ; send X coordinate
	CALL   UARTSEND1
	LOAD   AsciiY      ; send ascii 'Y'
	CALL   UARTRAW
	LOAD   OBJY        ; send Y coordinate
	CALL   UARTSEND1
	RETURN
	AsciiX: DW &H581B  ; escaped 'X'
	AsciiY: DW &H591B  ; escaped 'Y'

; Subroutine to send object count to base station.
; Format is '\nN' followed by one byte containing the count.
; Before calling, store count in this variable:
SendObjCount:
	CALL   UARTNL      ; send a newline
	LOAD   AsciiN      ; send ascii 'N'
	CALL   UARTRAW
	LOAD   ObjCount    ; send count
	CALL   UARTSEND1
	RETURN
	AsciiN: DW &H4E1B  ; escaped 'N'

; Subroutines to send AC value through the UART,
; Calling UARTSend2 will send both bytes of AC
; formatted for default base station code:
; [ AC(15..8) | AC(7..0)]
; Calling UARTSend1 will only send the low byte.
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend2:
	STORE  UARTTemp
	SHIFT  -8
	ADDI   -27   ; escape character
	JZERO  UEsc1
	ADDI   27
	OUT    UART_DAT
	JUMP   USend2
UEsc1:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
USend2:
	LOAD   UARTTemp
UARTSend1:
	AND    LowByte
	ADDI   -27   ; escape character
	JZERO  UEsc2
	ADDI   27
	OUT    UART_DAT
	RETURN
UEsc2:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
	RETURN
	UARTTemp: DW 0

; Subroutine to send a newline to the computer log
UARTNL:
	LOAD   NL
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NL: DW &H0A1B

; Subroutine to send a space to the computer log
UARTNBSP:
	LOAD   NBSP
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NBSP: DW &H201B

; Subroutine to clear the internal UART receive FIFO.
UARTClear:
	IN     UART_DAT
	JNEG   UARTClear
	RETURN

; Helper subroutine for sending raw values
UARTRAW:
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN


;***************************************************************
;* Provided Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
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

;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).

Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry,
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine

Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Inv: 2's complement inversion
; Returns -AC in AC
; Written by Kevin Johnson.  No licence or copyright applied.

Abs:
	JPOS   Abs_r
Inv:
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN


;***************************************************************
;* Variables
;***************************************************************
Temp:     DW 	 0 	; "Temp" is not a great name, but can be useful

LoopCount: DW	 0	;counts number of rows checked
RFlag: 	  DW     0	;1 if wall is deteced on the right
LFlag: 	  DW     0	;1 if wall is detected on the left
RightCount: DW	 0	;tracks how many times right side of aisle was checked
LeftCount:	DW	 0	;tracks how many times left side of aisle was checked

CDX: DW 0      ; current desired X
CDY: DW 0      ; current desired Y
CDT: DW 0      ; current desired angle
CX:  DW 0      ; sampled X
CY:  DW 0      ; sampled Y
CT:  DW 0      ; sampled theta

TargetL:	DW	0	; Left wheel position it saves when moving forward or back
target:		DW	0 	; Target angle to turn

ObjCount: DW 	 0	;total count of objects
ObjDist:  DW     0	;current object's distance from robot
OBJX:     DW      &H0000	;current object's x-value
OBJY:     DW      &H0000	;current object's y-value
XVAR:     DW      &H0000	;robot's converted x-position
YVAR:     DW      &H0000	;robot's converted y-position
RealX:	  DW		0
RealY:    DW		0

OBJX1:    DW      &H0000	;saved object positions
OBJY1:    DW      &H0000
OBJX2:    DW      &H0000
OBJY2:    DW      &H0000
OBJX3:    DW      &H0000
OBJY3:    DW      &H0000
OBJX4:    DW      &H0000
OBJY4:    DW      &H0000
OBJX5:    DW      &H0000
OBJY5:    DW      &H0000
OBJX6:    DW      &H0000
OBJY6:    DW      &H0000
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
OneMeter: DW 952       ; ~1m in 1.05mm units
HalfMeter: DW 476      ; ~0.5m in 1.05mm units
OneFoot:  DW 275       ; ~1ft in 1.05mm robot units (used to be 290)
TwoFeet:  DW 581       ; ~2ft in 1.05mm units

myAngle:  DW 0
Deg45:    DW 45
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 330       ; 350 is a medium speed
RMid:     DW -330
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

FTurn:	  DW 150	   ; Turning speed
RTurn: 	  DW -150

DesTheta: DW 0
DesVelL:  DW 0
DesVelR:  DW 0

Str2Feet: DW 430

MinBatt:  DW 100       ; 10.0V - minimum safe battery voltage
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
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
