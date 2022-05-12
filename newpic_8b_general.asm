LIST P=PIC18F4321	F=INHX32
#include <p18f4321.inc>

CONFIG  OSC=HSPLL ; L?oscil.lador a HighSpeed
CONFIG  PBADEN=DIG ; Volem que el PORTB sigui DIGital
CONFIG  WDT=OFF ; Desactivem el WatchDog Timer
CONFIG LVP=OFF

ORG 0x0000
GOTO    MAIN
ORG 0x0008
GOTO	HIGH_RSI
ORG 0x0018
RETFIE  FAST

    
PART_LOW    EQU 0x00  ; Variable
PART_HIGH   EQU 0x01  ;	Variable 
TECLA	    EQU 0x02 ; Tecla java
POS_SERVO_X EQU 0x03
COUNT_TIME   EQU 0x04
DONE_COUNT  EQU 0x05 
POS_SERVO_Y EQU 0x06
PART_LOW_X  EQU 0x07
PART_HIGH_X EQU 0x08
PART_LOW_Y  EQU 0x09
PART_HIGH_Y EQU 0x0A
AD_VALUE    EQU 0x0B ;Valor convertido del ADC
T_WAIT	    EQU 0x0C
    
TAULA7S	    EQU 0x20
TAULA_SERVO_X EQU 0x30

 
ORG TAULA7S
    ;Segments de C, segments de D
    DB 0x4D, 0x3E
    ;Segments de E, segments de F
    DB 0x4F, 0x47
    ;Segments de G, segments de A
    DB 0x7B, 0x77
    ;Segments de B, segments de c
    DB 0x1F, 0x0E
    
ORG TAULA_SERVO_X
    ;Pos de C, Pos de D
    DB .214, .220
    ;Pos de E, Pos de F
    DB .223, .226
    ;Pos de G, Pos de A
    DB .230, .234
    ;Pos de B, Pos de c
    DB .238, .242
        
;------------------- MOVIMIENTO SERVO ---------------------------
    
SERVO_X		;Carreguem el temps per als rebots.
   MOVLW   .148          
   MOVWF   PART_LOW_X,0
   MOVFF   POS_SERVO_X,PART_HIGH_X
ESPERA_X		;Rutina espera servo
   INCF    PART_LOW_X,1
   BTFSS   STATUS,C,0
   GOTO    ESPERA_X
   MOVLW   .148 
   MOVWF   PART_LOW_X,0
   INCF    PART_HIGH_X,1 
   BTFSS   STATUS,C,0
   GOTO    ESPERA_X
   RETURN  
   
SERVO_Y		;Carreguem el temps per als rebots.
   MOVLW   .148          
   MOVWF   PART_LOW_Y,0
   MOVFF   POS_SERVO_Y,PART_HIGH_Y
ESPERA_Y	;Rutina espera servo
   INCF    PART_LOW_Y,1
   BTFSS   STATUS,C,0
   GOTO    ESPERA_Y
   MOVLW   .148 
   MOVWF   PART_LOW_Y,0
   INCF    PART_HIGH_Y,1 
   BTFSS   STATUS,C,0
   GOTO    ESPERA_Y
   RETURN 
   
;------------------- CONFIGURACION TIMER0 ------------------------
   
CARREGA_TIMER
    BCF	INTCON,TMR0IF,0  ; Netejem el bit de causa d'interrupció
    MOVLW   b'00111011'
    MOVWF   TMR0H,0
    MOVLW   b'10000100'	 ; Carreguem el valor al TIMER0
    MOVWF   TMR0L,0
    RETURN
   
HIGH_RSI
    BTFSS   INTCON, TMR0IF,0 ; Mirem si la interrupció és de TIMER0
    RETFIE  FAST
    INCF    COUNT_TIME,1,0
    MOVF    T_WAIT,0,0
    CPFSEQ COUNT_TIME,0
    GOTO NEXT
    MOVLW 0xFF
    MOVWF DONE_COUNT,0
    NEXT	 
	BSF LATC,4,0
	CALL SERVO_X
	BCF LATC,4,0
	BSF LATC,5,0
	CALL SERVO_Y
	BCF LATC,5,0
	CALL CARREGA_TIMER
	RETFIE  FAST
   
CONFIG_TIMER
    ; CONFIGURACIÓ DEL TIMER0
    BCF	    RCON,IPEN,0	 ; Desactivem les prioritats
    MOVLW   b'10000001'	 ; Configurem el TIMER0
    MOVWF   T0CON,0      ; Timer0 Controller, no TOCÓN!!!
    CALL   CARREGA_TIMER ; Carreguem el TIMER0
    MOVLW   b'11100000'
    MOVWF   INTCON,0	 ; Habilitem totes les interrupcions i la del TMR0
    RETURN

;------------------- CONTROL REBOTES BOTONES ---------------------
    
ESPERA_16ms		;Carreguem el temps per als rebots.
   MOVLW   .158          
   MOVWF   PART_LOW,0
   MOVLW   .250
   MOVWF   PART_HIGH,0
ESPERA_REBOTE		;Rutina espera rebots
   INCF    PART_LOW,1
   BTFSS   STATUS,C,0
   GOTO    ESPERA_REBOTE
   MOVLW   .158 
   MOVWF   PART_LOW,0
   INCF    PART_HIGH,1 
   BTFSS   STATUS,C,0
   GOTO    ESPERA_REBOTE
   RETURN 
   
;------------------- MODOS DE JUEGO -----------------------------
PLAY_MANUAL
    BCF LATC,2,0 ;Y
    BCF LATC,1,0 ;R
    BSF LATC,0,0 ;G
    GOTO WAIT_RX
    
PLAY_AUTO
    ;Configuramos los LEDs
    BSF LATC,2,0 ;Y
    BCF LATC,1,0 ;R
    BCF LATC,0,0 ;G
    RETURN
;------------------- POLLING BOTONES -----------------------------  
    
WAIT_CHANGE_MODE
    BTFSC PORTB,0,0
    GOTO WAIT_CHANGE_MANUAL
    CALL ESPERA_16ms
    BTFSC PORTB,0,0
    GOTO WAIT_CHANGE_MANUAL
    WAIT_SOLTAR_MODE
	BTFSS PORTB,0,0
	GOTO WAIT_SOLTAR_MODE
	CALL ESPERA_16ms
	BTFSS PORTB,0,0
	GOTO WAIT_SOLTAR_MODE
	BTFSS PORTC,0,0 ;Comprobar en que modo estaba antes, y cambiar al nuevo
	GOTO PLAY_MANUAL ;Si estaba en automatico, pasar a manual
	CALL PLAY_AUTO ;Si estaba en manual, pasar a auto
	GOTO WAIT_CHANGE_MANUAL

WAIT_CHANGE_MANUAL
    BTFSC PORTB,1,0
    GOTO WAIT_BOTON_AUX
    CALL ESPERA_16ms
    BTFSC PORTB,1,0
    GOTO WAIT_BOTON_AUX
    WAIT_SOLTAR_MANUAL
	BTFSS PORTB,1,0
	GOTO WAIT_SOLTAR_MANUAL
	;Comprobar que estemos en modo manual
	BTFSC PORTC,0,0
	BTG LATA,3,0 ;Invertir valor del LED de Modo Manual
	GOTO WAIT_BOTON_AUX

WAIT_BOTON_AUX
    BTFSC PORTB,2,0
    GOTO WAIT_RX
    CALL ESPERA_16ms
    BTFSC PORTB,2,0
    GOTO WAIT_RX
    WAIT_SOLTAR_AUX
	BTFSS PORTB,2,0
	GOTO WAIT_SOLTAR_AUX
	BTG LATA,3,0 ;Invertir valor del LED de Modo Manual
	GOTO WAIT_RX
	
;---------------------- JOYSTICK ---------------------------------
SET_ADCON_X
    ;Seleccionamos AN0
    MOVLW b'00000001'
    MOVWF ADCON0,0
    RETURN
SET_ADCON_Y
    ;Seleccionamos AN1
    MOVLW b'00000101'
    MOVWF ADCON0,0
    RETURN
    
WAIT_CONV
    CALL SET_ADCON_X
    BSF ADCON0,1,0
    WAIT_DONE
	BTFSC ADCON0,1,0
	GOTO WAIT_DONE
    CALL READ_XY
    GOTO WAIT_CHANGE_MODE
    
READ_XY
    BSF LATA,3,0
    MOVFF ADRESH, LATD
    RETURN
    ;Comprobar Joystick X
    MOVFF ADRESH, AD_VALUE
    CALL CHECK_RIGHT 
    ;Comprobar Joystick Y -> Hay que crear la tabla flash de valores
    CALL SET_ADCON_Y
    WAIT_ADCON_Y
	BSF ADCON0,1,0
	BTFSC ADCON0,1,0
	GOTO WAIT_ADCON_Y
	;GOTO CHECK_Y
    RETURN
    
CHECK_LEFT
    MOVLW .0 ;TODO -> Poner valor inferior a 1.5V aprox
    CPFSGT AD_VALUE,0
    GOTO WAIT_CHANGE_MODE ;Si el valor no es superior 
    WAIT_SOLTAR_L_JS ;Esperar a que el JS vuelva a la posicion inicial
        CPFSLT AD_VALUE,0
	GOTO MOVE_LEFT
	GOTO WAIT_SOLTAR_L_JS
    ;Una vez ha vuelto, tocar la tecla correspondiente
    MOVE_LEFT
	;Comprobar que no esté ya en la ultima tecla por la izquierda
	MOVLW 0x30
	CPFSGT TBLPTRL,0
	GOTO WAIT_CHANGE_MODE ; Si esta en la posicion de la izq del todo, no hacer nada y volver al polling
	MOVLW 0x01
	SUBWF TBLPTRL,1,0 ;Sino, reducimos una posicion del puntero
	MOVF TBLPTRL,0,0
	;Comprobar que estamos en manual antes de pulsar la tecla
	BTFSS PORTC,2,0
	GOTO PLAY_TECLA ;Manual, pulsamos
	GOTO WAIT_RX ;Auto, volvemos a polling
	
CHECK_RIGHT
    MOVLW .0 ;TODO -> Poner valor superior a 3V y comparar con el leído en el JS_X
    CPFSGT AD_VALUE,0
    GOTO CHECK_LEFT
    WAIT_SOLTAR_R_JS ;Esperar a que el JS vuelva a la posicion inicial
        CPFSGT AD_VALUE,0
	GOTO MOVE_RIGHT
	GOTO WAIT_SOLTAR_R_JS
    ;Una vez ha vuelto, tocar la tecla correspondiente
    MOVE_RIGHT
	;Comprobar que no esté ya en la ultima tecla por la derecha
	MOVLW 0x37
	CPFSLT TBLPTRL,0
	GOTO WAIT_CHANGE_MODE ; Si esta en la posicion de la derecha del todo, no hacer nada y volver al polling
	MOVLW 0x01
	ADDWF TBLPTRL,1,0 ;Sino, avanzamos una posicion del puntero
	MOVF TBLPTRL,0,0
	;Comprobar que estamos en manual antes de pulsar la tecla
	BTFSS PORTC,2,0
	GOTO PLAY_TECLA ;Manual, pulsamos
	GOTO WAIT_RX ;Auto, volvemos a polling
        
    
;------------------- INICIALIZACION ------------------------------
INIT_PORTS
   BCF INTCON2,RBPU,0 ;Pull-ups del puerto B activos
   SETF TRISA,0 ;Puertos A de entrada para Joystick (Analog)
   BCF TRISA,3,0;Puerto A3 de salida para Led de Modo Manual
   BCF TRISA,4,0
   SETF TRISB,0 ;Puertos B de entrada para poner pulsadores
   CLRF TRISC,0 ;Puertos C de salida para Servos, TX Y LEDs
   BCF TRISC,6 ;Puerto C bit 7 de entrada para RX Y TX
   BCF TRISC,7
   CLRF TRISD,0 ;Puerto D de salida para el 7seg
   CLRF LATD,0
   CLRF TBLPTRU,0
   CLRF TBLPTRH,0
   ;Empieza en auto
   BSF LATC,2,0 ;Yellow (Blue)
   BCF LATC,1,0 ;Red
   BCF LATC,0,0 ;Green
   BCF LATA,3,0 ;Empieza en modo manual con Joystick
   BCF LATA,4,0 ;DEBUG
   ;Config de Baudcon
   ;Fosc = 40Mhz, Baudrate 9600, SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 1040, ERR: 0.06%
   MOVLW b'00100100'
   MOVWF TXSTA,0
   MOVLW b'10010000'
   MOVWF RCSTA,0
   BSF BAUDCON,BRG16,0 
   ;Ponemos el 1024 en el SPBRG
   MOVLW b'00000100'
   MOVWF SPBRGH,0
   MOVLW b'00010000'
   MOVWF SPBRG,0
   ;Posicion inicial servo X
   MOVLW .226
   MOVWF POS_SERVO_X,0
   MOVLW .230
   MOVWF POS_SERVO_Y,0
   ;Configuracion ADCONs para Joystick
   ;-- ADCON0 --
   ;Inicializamos el ADCON0 para escuchar el AN0
   MOVLW b'00000001'
   MOVWF ADCON0,0
   ;-- ADCON1 --
   MOVLW b'00001101'
   ;-- ADCON2 --
   MOVLW b'00001001'
   
   RETURN
   
;------------------- REPRODUCCION CANCION -------------------------
WAIT_TIME
    CLRF DONE_COUNT
    CLRF COUNT_TIME
    KEEP_WAITING
	MOVLW 0xFF
	CPFSEQ DONE_COUNT
	GOTO KEEP_WAITING
	RETURN
	
WAIT_RX
    BTFSS PIR1,RCIF,0
    GOTO WAIT_CONV
    MOVFF RCREG,TECLA
    GOTO CHECK_P
CHECK_MODE
    BTFSS PORTC,2,0
    GOTO CHECK_C
    GOTO WAIT_CONV
CHECK_P
    MOVLW 'P'
    CPFSEQ TECLA,0
    GOTO CHECK_MODE
    CALL SEND_ACK
    CALL PLAY_AUTO
    GOTO PLAY_SONG
    
;Lee la tecla correspondiente de la tabla y la muestra por el 7seg
PLAY_TECLA
    ;Mostrar tecla por 7 segments
    MOVWF TBLPTRL,0
    TBLRD*
    MOVFF TABLAT,LATD
    ;Mover Tecla servo
    MOVLW 0x10
    ADDWF TBLPTRL,1,0
    TBLRD*
    ;Mover X
    MOVFF TABLAT,POS_SERVO_X
    MOVLW .10
    MOVWF T_WAIT,0
    CALL WAIT_TIME
    ;Mover Y abajo
    MOVLW .226
    MOVWF POS_SERVO_Y,0
    MOVLW .5
    MOVWF T_WAIT,0
    CALL WAIT_TIME
    ;Mover Y arriba
    MOVLW .230
    MOVWF POS_SERVO_Y,0
    BTFSS PORTC,2,0
    GOTO WAIT_RX
    GOTO PLAY_SONG

CHECK_C
    MOVLW 'C'
    CPFSEQ TECLA,0
    GOTO CHECK_d
    MOVLW 0x20
    GOTO PLAY_TECLA
CHECK_d
    MOVLW 'D'
    CPFSEQ TECLA,0
    GOTO CHECK_E
    MOVLW 0x21
    GOTO PLAY_TECLA
CHECK_E
    MOVLW 'E'
    CPFSEQ TECLA,0
    GOTO CHECK_F
    MOVLW 0x22
    GOTO PLAY_TECLA
CHECK_F
    MOVLW 'F'
    CPFSEQ TECLA,0
    GOTO CHECK_g
    MOVLW 0x23
    GOTO PLAY_TECLA
CHECK_g
    MOVLW 'G'
    CPFSEQ TECLA,0
    GOTO CHECK_A
    MOVLW 0x24
    GOTO PLAY_TECLA
CHECK_A
    MOVLW 'A'
    CPFSEQ TECLA,0
    GOTO CHECK_b
    MOVLW 0x25
    GOTO PLAY_TECLA
CHECK_b
    MOVLW 'B'
    CPFSEQ TECLA,0
    GOTO CHECK_c
    MOVLW 0x26
    GOTO PLAY_TECLA
CHECK_c
    MOVLW 'c'
    CPFSEQ TECLA,0
    GOTO WAIT_RX
    MOVLW 0x27
    GOTO PLAY_TECLA
CHECK_S
    MOVLW 'S'
    CPFSEQ TECLA,0
    GOTO CHECK_C
    GOTO WAIT_RX
    
PLAY_SONG
   BTFSS PIR1,RCIF,0
   GOTO PLAY_SONG
   MOVFF RCREG,TECLA
   GOTO CHECK_S

SEND_ACK
    MOVLW 'K'
    MOVWF TXREG,0
    ;Esperamos a que se complete el envio
    WAIT_TX
	BTFSC TXSTA,TRMT,0
	GOTO WAIT_TX
        RETURN	
	
;------------------- MAIN ------------------------
MAIN
    CALL INIT_PORTS
    CALL CONFIG_TIMER
    GOTO WAIT_CHANGE_MANUAL
END