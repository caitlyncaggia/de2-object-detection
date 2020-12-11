; Add this subroutine to the turn functions in order to translate the target
; angle to either 0, 90, 180, 270. Will make it more accurate and the error
; of turn angle will not carry.

decideAngle:
    IN      THETA
    STORE   myAngle
    SUB     angle45     ; if less than 45 degrees

    JZERO   faceNorth
    JNEG    faceNorth   ; decided to face north

    SUB     angle90     ; if less than 135 degrees

    JZERO   faceWest
    JNEG    faceWest    ; decided to face west

    SUB     angle90     ; if less than 225 degrees

    JZERO   faceSouth
    JNEG    faceSouth   ; decided to face south

    SUB     angle90     ; if less than 315 degrees

    JZERO   faceWest
    JNEG    faceWest    ; decided to face east

    JUMP    faceNorth   ; if greater than 315 degrees, will turn north

faceNorth:
    LOAD    Zero        ; load the target angle of 0
    RETURN
faceWest:
    LOAD    angle90     ; load the target angle of 90
    RETURN
faceSouth:
    LOAD    angle180    ; load the target angle of 180
    RETURN
faceEast:
    LOAD    angle270    ; load the target angle of 270
    RETURN              ; returns to the caller with the ...
                        ; ... correct target angle loaded


myAngle:    DW  0
angle45:    DW  45
angle90:    DW  90
angle180:   DW  180
angle270:   DW  270
