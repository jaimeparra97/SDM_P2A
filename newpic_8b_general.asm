    LIST P=PIC18F4321	F=INHX32
    #include <p18f4321.inc>

    CONFIG  OSC=HSPLL ; L?oscil.lador a HighSpeed
    CONFIG  PBADEN=DIG ; Volem que el PORTB sigui DIGital
    CONFIG  WDT=OFF ; Desactivem el WatchDog Timer
    CONFIG LVP=OFF

    ORG 0x0000
    GOTO MAIN
    ORG 0x0008
    GOTO HIGH_RSI
    ORG 0x0018
    RETFIE  FAST

    
PART_LOW    EQU 0x00  ; Variable
PART_HIGH   EQU 0x01  ;	Variable 
TECLA	    EQU 0x02 ; Tecla java
POS_SERVO_X EQU 0x03
COUNT_TIME  EQU 0x04
DONE_COUNT  EQU 0x05 
POS_SERVO_Y EQU 0x06
PART_LOW_X  EQU 0x07
PART_HIGH_X EQU 0x08
PART_LOW_Y  EQU 0x09
PART_HIGH_Y EQU 0x0A
AD_VALUE    EQU 0x0B ;Valor convertido del ADC
T_WAIT	    EQU 0x0C
OFFSET	    EQU 0x0D ;Posicion del JS
COUNT_1s    EQU 0x0E
    
TAULA7S	    EQU 0x20
TAULA_SERVO_X EQU 0x30
TAULA_SERVO_Y EQU .254

 
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
    
    ORG TAULA_SERVO_Y
    DB .230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230
    DB .230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230,.230
    DB .229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229
    DB .229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.229
    DB .229,.229,.229,.229,.229,.229,.229,.229,.229,.229,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228
    DB .228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228
    DB .228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228,.228
    DB .227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227
    DB .227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.227
    DB .227,.227,.227,.227,.227,.227,.227,.227,.227,.227,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226
    DB .226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226
    DB .226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226
    DB .226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226,.226
        
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
    ;Contador para 1s
    CALL CHECK_1s
    INCF    COUNT_TIME,1,0
    MOVF    T_WAIT,0,0
    CPFSEQ COUNT_TIME,0
    GOTO NEXT_STEP
    MOVLW 0xFF
    MOVWF DONE_COUNT,0
    NEXT_STEP	 
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
    CLRF LATD,0
    BCF LATC,2,0 ;Y
    BCF LATC,1,0 ;R
    BSF LATC,0,0 ;G
    BSF LATA,3,0 ;Modo JAVA por defecto
    GOTO WAIT_CHANGE_MANUAL
    
PLAY_AUTO
    ;Configuramos el LED Amarillo
    BSF LATC,2,0 ;Y
    BCF LATC,1,0 ;R
    BCF LATC,0,0 ;G
    RETURN
    
PLAY_RECORD
    ;Configuramos el LED Rojo
    BCF LATC,2,0 ;Y
    BSF LATC,1,0 ;R
    BCF LATC,0,0 ;G
    RETURN
    
;------------------- POLLING BOTONES -----------------------------  
    
WAIT_CHANGE_MODE
    ;Comprobamos que no esté grabando
    BTFSC PORTC,1,0
    GOTO WAIT_CHANGE_MANUAL
    BTFSC PORTB,0,0
    ;GOTO WAIT_CHANGE_MANUAL
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
    ;Primero comprobamos si estamos en modo Manual
    BTFSS PORTC,0,0
    GOTO WAIT_PLAY_REC
    ;Comprobamos que no esté grabando
    BTFSC PORTC,1,0
    GOTO WAIT_PLAY_REC
    ;Comprobamos que se haya pulsado el boton
    BTFSC PORTB,1,0
    GOTO WAIT_PLAY_REC
    ;Comprobamos rebotes
    CALL ESPERA_16ms
    BTFSC PORTB,1,0
    GOTO WAIT_PLAY_REC
    WAIT_SOLTAR_MANUAL
	BTFSS PORTB,1,0
	GOTO WAIT_SOLTAR_MANUAL
	BTG LATA,3,0 ;Invertir valor del LED de Modo Manual
	BTFSS LATA,3,0
	CALL INIT_JS
	GOTO WAIT_PLAY_REC

WAIT_PLAY_REC
    ;Primero comprobamos si estamos en modo Manual
    BTFSS PORTC,0,0
    GOTO WAIT_RX
    BTFSC PORTB,2,0
    GOTO WAIT_RX
    CALL ESPERA_16ms
    BTFSC PORTB,2,0
    GOTO WAIT_RX
    WAIT_SOLTAR_PLAY
	BTFSS PORTB,2,0
	GOTO WAIT_SOLTAR_PLAY
	;GOTO PLAY_RECORDING ToDo
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
    ;Comprobar que estamos en manual antes de seguir
    BTFSS PORTC,0,0
    GOTO WAIT_CHANGE_MODE ;Auto, salimos
    ;Comprobar manual JS
    BTFSC PORTA,3,0
    GOTO WAIT_CHANGE_MODE ;Java, salimos
    
    CALL SET_ADCON_X
    CALL LISTEN_ADC
    ;Comprobar Joystick X
    GOTO CHECK_RIGHT

CHECK_Y
    CALL SET_ADCON_Y
    CALL LISTEN_ADC
    BSF LATA,4,0
    MOVFF AD_VALUE, TBLPTRL
    TBLRD*
    MOVFF TABLAT,POS_SERVO_Y
    CALL WAIT_RETURN_Y
    GOTO WAIT_CHANGE_MODE
    
CHECK_LEFT
    MOVLW .50 
    CPFSLT AD_VALUE,0
    GOTO WAIT_CHANGE_MODE ;Si el valor no es superior 
    ;Esperar a que el JS vuelva a la posicion inicial
    CALL WAIT_RETURN_LEFT
    ;Una vez ha vuelto, tocar la tecla correspondiente
    MOVE_LEFT
	;Comprobar que no esté ya en la ultima tecla por la izquierda
	MOVLW 0x00
	CPFSGT OFFSET,0
	GOTO WAIT_CHANGE_MODE ; Si esta en la posicion de la izq del todo, no hacer nada y volver al polling
	MOVLW 0x01
	SUBWF OFFSET,1,0 ;Sino, reducimos una posicion del offset
	CALL UPDATE_POINTER
    GOTO CHECK_Y
	
CHECK_RIGHT
    MOVLW .200 
    CPFSGT AD_VALUE,0
    GOTO CHECK_LEFT
    ;Esperar a que el JS vuelva a la posicion inicial
    CALL WAIT_RETURN_RIGHT
    ;Una vez ha vuelto, tocar la tecla correspondiente
    MOVE_RIGHT
	;Comprobar que no esté ya en la ultima tecla por la izquierda
	MOVLW 0x07
	CPFSLT OFFSET,0
	GOTO WAIT_CHANGE_MODE ; Si esta en la posicion de la derecha del todo, no hacer nada y volver al polling
	MOVLW 0x01
	ADDWF OFFSET,1,0 ;Sino, aumentamos una posicion del offset
	CALL UPDATE_POINTER
    GOTO CHECK_LEFT
	
UPDATE_POINTER
    ;Ponemos en W la direccion inicial de la tabla + el offset
    MOVLW 0x20
    ADDWF OFFSET,0,0 
    CALL MOVE_X ;Movemos la tecla
    RETURN
LISTEN_ADC
    ;Activamos GO/DONE
    BSF ADCON0,1,0
    WAIT_DONE
	BTFSC ADCON0,1,0
	GOTO WAIT_DONE
	MOVFF ADRESH, AD_VALUE
    RETURN
WAIT_RETURN_RIGHT
    CALL LISTEN_ADC
    MOVLW .180 
    CPFSLT AD_VALUE,0
    GOTO WAIT_RETURN_RIGHT
    RETURN
WAIT_RETURN_LEFT
    CALL LISTEN_ADC
    MOVLW .50
    CPFSGT AD_VALUE,0
    GOTO WAIT_RETURN_LEFT
    RETURN
WAIT_RETURN_Y
    CALL LISTEN_ADC
    MOVLW .180
    CPFSLT AD_VALUE,0
    GOTO WAIT_RETURN_Y
    RETURN
;---------------------- GRABACIÓN --------------------------------
;1- Comprobar que lleva +1s apretando (y ha soltado)
;2- Limpiar la RAM de la grabacion anterior
;3- Cada vez que se pulsa una tecla (Move Y) se guarda esa tecla (El valor de POS_SERVO_X?) en la RAM
;4- Controlar que no se cambie de modo mientras se graba la cancion
    
CLEAN_RAM ;Limpiamos el Bank 1
    LFSR FSR0, 100h ;100h es la posicion inicial de memoria del Bank 1
    NEXT 
	CLRF POSTINC0 ;Limpiar registro INDF e incrementar posicion hasta el fin del bank 1
	BTFSS FSR0H,1
	BRA NEXT
    ;Posicionar puntero al principio
    LFSR FSR0,100h
    RETURN
    
SAVE_RAM
    ;Posicionar punteros
    ;Escribir RAM y aumentar posicion 
    MOVFF POS_SERVO_X, POSTINC0
    RETURN
    
CHECK_1s
    BTFSC PORTB,0,0
    CLRF COUNT_1s,0
    MOVLW 0x01
    ADDWF COUNT_1s,1,0
    MOVLW .50
    CPFSGT COUNT_1s,0
    RETURN
    ;Si está grabando, pausamos la grabación
    BTFSS PORTC,1,0
    CALL START_REC
    BTFSC PORTC,1,0
    RETURN
    BCF LATC,1,0
    BSF LATC,0,0
    RETURN
START_REC
    ;Comprobamos que esté en modo manual
    BTFSS PORTC,0,0
    RETURN
    BSF LATC,1,0
    BCF LATC,0,0
    BCF LATC,2,0
    RETURN

;------------------- INICIALIZACION ------------------------------
INIT_JS
    ;El JS empieza en la F
    MOVLW 0x03
    MOVWF OFFSET,0
    MOVLW 0x23
    CALL MOVE_X
    RETURN
	
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
   BSF LATA,3,0 ;Empieza en modo manual con JAVA
   BCF LATA,4,0 ;DEBUG
   ;El JS empieza en la F
    MOVLW 0x03
    MOVWF OFFSET,0
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
   ;Posicion inicial servo
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
   MOVWF ADCON1,0
   ;-- ADCON2 --
   MOVLW b'00010001'
   MOVWF ADCON2,0
   ;Escogemos el bank 1
   MOVLW 0x01
   MOVWF BSR,1
   RETURN
   
;------------------- REPRODUCCION CANCION -------------------------
WAIT_TIME
    CLRF DONE_COUNT,0
    CLRF COUNT_TIME,0
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
CHECK_P
    MOVLW 'P'
    CPFSEQ TECLA,0
    GOTO CHECK_MODE
    CALL SEND_ACK ;Si esta en auto, devuelve ACK y reproduce la cancion
    CALL PLAY_AUTO
    GOTO PLAY_SONG
CHECK_MODE
    BTFSC PORTC,2,0
    GOTO WAIT_CONV ;Si esta en auto, vuelve al polling
    BTFSS PORTA,3,0
    GOTO WAIT_CONV ; Si esta en manual JS, vuelve a polling
    GOTO CHECK_C ;Si esta en manual Java, toca la tecla
    
;Lee la tecla correspondiente de la tabla y la muestra por el 7seg
PLAY_TECLA
    CALL MOVE_X
    CALL MOVE_Y
    BTFSS PORTC,2,0
    GOTO WAIT_RX
    GOTO PLAY_SONG
    
MOVE_X
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
    ;Si esta grabando, guardar tecla en RAM
    BTFSC PORTC,1,0
    CALL SAVE_RAM
    RETURN
MOVE_Y
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
    RETURN
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