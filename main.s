#include <xc.inc>

;<editor-fold defaultstate="collapsed" desc="Info & Contacto">
    /*Gomez Vieyra Mucio Fausto
    Estudiante - Unidad Profesional Interdisciplinaria en Ingenierï¿½a y
    Tecnologias Avanzadas del Intituto Politecnico Nacional
    Ingenieria Mecatronica
    01/11/2020
    Reto: Hebocon
	MPLAB X IDE v5.40
	MCU: PIC18f2550
	Tarjeta de desarrollo: Pecten Alpha
	Lenguaje: Ensamblador
	Compilador pic-as(2.30)
    Este codigo permite que el sistema pueda comunicarse por medio de
    un modulo HC-05 con un protocolo USART integrado en el microcontrolador
    con un dispositivo externo via Bluetooth, el cual, de acuerdo a los Bytes
    recibidos por un dispositivo ejecutara rutinas de movimiento o de ataque
    simulando y facilitando el prototipado de un robot de pelea*/
;</editor-fold>

;<editor-fold defaultstate="collapsed" desc="Configuracion de Fusibles">
  ; CONFIG1L
    CONFIG  PLLDIV = 5            ; PLL Prescaler Selection bits (Divide by 5 (20 MHz oscillator input))
    CONFIG  CPUDIV = OSC1_PLL2    ; System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
    CONFIG  USBDIV = 1            ; USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)
  ; CONFIG1H
    CONFIG  FOSC = HS             ; Oscillator Selection bits (HS oscillator (HS))
    CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
    CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
  ; CONFIG2L
    CONFIG  PWRT = ON             ; Power-up Timer Enable bit (PWRT enabled)
    ;CONFIG BOR = OFF             ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
    CONFIG  BORV = 3              ; Brown-out Reset Voltage bits (Minimum setting 2.05V)
    CONFIG  VREGEN = OFF          ; USB Voltage Regulator Enable bit (USB voltage regulator disabled)
  ; CONFIG2H
    CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
    CONFIG  WDTPS = 32768         ; Watchdog Timer Postscale Select bits (1:32768)
  ; CONFIG3H
    CONFIG  CCP2MX = ON           ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
    CONFIG  PBADEN = OFF           ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
    CONFIG  LPT1OSC = OFF         ; Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
    CONFIG  MCLRE = ON            ; MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
  ; CONFIG4L
    CONFIG  STVREN = ON           ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
    CONFIG  LVP = OFF             ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
    CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
  ; CONFIG5L
    CONFIG  CP0 = OFF             ; Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
    CONFIG  CP1 = OFF             ; Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
    CONFIG  CP2 = OFF             ; Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
    CONFIG  CP3 = OFF             ; Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)
  ; CONFIG5H
    CONFIG  CPB = OFF             ; Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
    CONFIG  CPD = OFF             ; Data EEPROM Code Protection bit (Data EEPROM is not code-protected)
  ; CONFIG6L
    CONFIG  WRT0 = OFF            ; Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
    CONFIG  WRT1 = OFF            ; Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
    CONFIG  WRT2 = OFF            ; Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
    CONFIG  WRT3 = OFF            ; Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)
  ; CONFIG6H
    CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
    CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
    CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM is not write-protected)
  ; CONFIG7L
    CONFIG  EBTR0 = OFF           ; Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
    CONFIG  EBTR1 = OFF           ; Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
    CONFIG  EBTR2 = OFF           ; Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
    CONFIG  EBTR3 = OFF           ; Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)
  ; CONFIG7H
    CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)
;</editor-fold>

;<editor-fold defaultstate="collapsed" desc="Constantes">
    ;Constatnes empleadas para el control del robot
    #define servoT  40;numero de ciclos en el pulso PWM del servomotor
    /*Al modificar los siquientes datos estaremos modificando la infromacion
    que esperamos recibir del dispoditivo bluetooth externo (1 Byte)*/
    #define up	    00000001B;dato de movimiento hacia adelante
    #define right   00000010B;dato de movimiento hacia la derecha 
    #define left    00000011B;dato de movimiento hacia la izquierda
    #define down    00000100B;dato de movimiento hacia atrás
    #define stop    00001111B;dato de suspencion de movimiento
    #define play    11110000B;dato que acciona el arma
;</editor-fold>

psect udata_acs;seccion de programa para el almacenamiento de variables
;<editor-fold defaultstate="collapsed" desc="Variables">
    DCounter1:	    ds 1;contador para los retardos de tiempo
    DCounter2:	    ds 1;contador para los retardos de tiempo
    DCounter3:	    ds 1;contador para los retardos de tiempo
    TEMPORAL:	    ds 1;registro para almacen temporal de datos
    RESP_W:	    ds 1;respaldo del registro w
    RESP_STATUS:    ds 1;respaldo del registro status
    RESP_BSR:	    ds 1;respaldo del registro bsr
    BANDERAS:	    ds 1;registro de banderas de interrupcion
    TRANSMICION:    ds 1;registro para almacenar el dato a transmitir
    RECEPCION:	    ds 1;registro para almacenar el dato a recibir
;</editor-fold>

psect vecReset,class=CODE,reloc=2;seccion para el reinicio del programa
;<editor-fold defaultstate="collapsed" desc="Vector Reset">
    VecReset:
	PAGESEL ProgPrin;seleccion de la pagina de memoria de ProgPrin
	goto ProgPrin;direccionamiento al programa Principal
;</editor-fold>

psect vecInt,class=CODE,reloc=2;seccion para las interrupciones del programa
;<editor-fold defaultstate="collapsed" desc="Vector Interrupcion">
    VecInt:
	/*En la siguiente seccion de programa se establece el algoritmo de 
	respado de registros importantes*/
	movwf RESP_W;respaldamos el contenido de W
	movf BSR,W;movemos el contenido de BSR a W
	movwf RESP_BSR;cargamos el registro de respaldo con la info de BSR
	clrf BSR;borramos el registro BSR ya que fue respaldado
	swapf STATUS,W;movemos STATUS a w usando swap para no modificarlo
	movwf RESP_STATUS;cargamos el registro de respaldo con la info de STATUS
	clrf STATUS;borramos el registro STATUS
	/*Se establece el algoritmo de deteccion de interrupciones
	Interrupcioines activas: 
	    USART*/
	btfsc RCIF;si interrupio USART ejecuta la subrrutina, si no, se sale
	call BlueRX;llamada a la subrrutina de recepcion de datos
    SalInt:
	/*Se establece el algoritmo de restablecimiento de registros FSR */
	movf RESP_BSR,W;restauramos el registro BSR
	movwf BSR
	swapf RESP_STATUS,W;restauramos el registro STATUS usando swap de nuevo
	movwf STATUS
	movf RESP_W,W;restauramos el registro W
    retfie;salimos de las interrupciones regresando al estado del ProgPrin

    /*Subrrutina de recepcion de datos*/
    BlueRX:
	/*Se establece el algoritmo para determinar la accion a ejecutar de
	acuerdo al tipo de dato recibido*/
	;<editor-fold defaultstate="collapsed" desc="Switch de Recepcion">
	movf RCREG,W
	movwf RECEPCION
	movlw up
	subwf RECEPCION,W
	btfsc ZERO
	goto Adelante
	movlw down
	subwf RECEPCION,W
	btfsc ZERO
	goto Atras
	movlw right
	subwf RECEPCION,W
	btfsc ZERO
	goto Derecha
	movlw left
	subwf RECEPCION,W
	btfsc ZERO
	goto Izquierda
	movlw stop
	subwf RECEPCION,W
	btfsc ZERO
	goto Alto
	movlw play
	subwf RECEPCION,W
	btfsc ZERO
	goto Arma
	goto SalBlueRX
	;</editor-fold>
	/*Se establece el algoritmo para ejecutar las acciones de acuerdo al 
	resultado del Switch de recepcion*/
    Adelante:
	movlw 00010111B;Se carga el dato al driver L293D del robot
	movwf PORTB
	goto SalBlueRX
    Atras:
	movlw 00101011B;Se carga el dato al driver L293D del robot
	movwf PORTB
	goto SalBlueRX
    Derecha:
	movlw 0000101B;Se carga el dato al driver L293D del robot
	movwf PORTB
	goto SalBlueRX
    Izquierda:
	movlw 00010010B;Se carga el dato al driver L293D del robot
	movwf PORTB
	goto SalBlueRX
    Alto:
	movlw 00000000B;Se carga el dato al driver L293D del robot
	movwf PORTB
	goto SalBlueRX
    Arma:
	movlw servoT;Carga la constante para el ciclo 1 de PWM del servomotor
	movwf TEMPORAL;utilizamos el registro temproal para manipular el dato
	Mov1:
	    /*PWM para el movimiento de 90° del servomotor*/
	    bsf RA2;pulso alto PWM
	    call Delay1ms;llamada al retardo de 1ms
	    bcf RA2;pulso bajo PWM
	    call Delay19ms;llamada al retardo de 19ms
	    decfsz TEMPORAL;condicional de salida para el ciclo 1 PWM (servoT)
	goto Mov1;Se repite el ciclo 1
	movlw servoT;Carga la constante para el ciclo 2 de PWM del servomotor
	movwf TEMPORAL;utilizamos el registro temproal para manipular el dato
	Mov2:
	    /*PWM para el movimiento de 0° del servomotor*/
	    bsf RA2;pulso alto PWM
	    call Delay1ms;llamada al retardo de 2ms en total
	    call Delay1ms
	    bcf RA2;pulso bajo PWM
	    call Delay19ms;llamada al retardo de 19ms ~~ 18ms 
	    decfsz TEMPORAL;condicional de salida para el ciclo 2 PWM (servoT)
	goto Mov2;Se repite el ciclo 2
	goto SalBlueRX;direccionamiento a la salida de la subrrutina BlueRX

    SalBlueRX:
	bcf RCIF;bajamos la bandera de interrupcion
    return;regresamos de la subrrutina

    Delay1ms:
	/*Algoritmo de retardo de 1ms*/
	MOVLW 0X7b
	MOVWF DCounter1
	MOVLW 0X07
	MOVWF DCounter2
	LOOP1ms:
	DECFSZ DCounter1, 1
	GOTO LOOP1ms
	DECFSZ DCounter2, 1
	GOTO LOOP1ms
	NOP
	NOP
    return;regresa

    Delay19ms:
	/*Algoritmo de retardo de 19ms*/
	MOVLW 0X5d
	MOVWF DCounter1
	MOVLW 0X7c
	MOVWF DCounter2
	LOOP19ms:
	DECFSZ DCounter1, 1
	GOTO LOOP19ms
	DECFSZ DCounter2, 1
	GOTO LOOP19ms
	NOP
	NOP
    return;regresa
;</editor-fold>

psect code
;<editor-fold defaultstate="collapsed" desc="ProgramaInicial">
    ProgIni:
	/*Algoritmo para la configuracion de perifericos, puertos, 
	interrupciones, etc*/
	bcf IPEN;desactivamos la prioridad de interrupciones
	bcf SBOREN;desactivamos el reinicio por caida de tension
	movlw 00000000B;apagamos el conversor analogo a digital
	movwf ADCON0;
	movlw 00001111B;establecemos todos los pines digitales
	movwf ADCON1;
	movlw 11000000B;activamos interrupciones globales y de perifericos
	movwf INTCON
	;<editor-fold defaultstate="collapsed" desc="config Bluetooth">
	bcf BRGH;Baud Rate de baja velocidad
	bcf BRG16;8-bit Baud Rate Generator
	movlw 31;9600 bauds
	movwf SPBRG
	bcf SYNC
	bsf SPEN
	bsf RCIE;habilitamos interrupcion por recepcion
	bcf RX9;deshabilitamos transmision de 9 bits
	bsf CREN;recepcion habilitada
	bcf TXEN;transmision deshabilitada
	;</editor-fold>
	movlw 11111011B;configuramos el pin RA2 como salida, resto como entrada
	movwf TRISA
	movlw 11000000B;configuramos RB7, RB6 como entrada, resto como salida
	movwf TRISB
	movlw 11111011B;configuramos RC2 como salida, resto como entrada
	movwf TRISC
    return;regresa
;</editor-fold>
;<editor-fold defaultstate="collapsed" desc="ProgramaPincipal">
    ProgPrin:
	call ProgIni;llamada al programa inicial (configuraciones)
	Loop:
	    nop;no hacer nada
	    nop;no hacer nada
	    nop;no hacer nada
	goto Loop;ciclo infinito
    goto ProgPrin;ciclo infinito (proteccion)
;</editor-fold>

end ProgPrin;finaliza el programa en ensambladro
