;*****************************************************************************************************
; PROGRAMA PARA ENVIAR 10 BITS POR COMUNICACIÓN DE RADIOFRECUENCIA, MEDIANTE EL MÓDULO NRF
;***************************************************************************************************** 
; Configuración (fuses)
#include "p18f4550.inc"
LIST p=18F4550

; CONFIG1L
  CONFIG  PLLDIV = 1            ; PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
  CONFIG  CPUDIV = OSC1_PLL2    ; System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
  CONFIG  USBDIV = 1            ; USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

; CONFIG1H
  CONFIG  FOSC = INTOSC_HS      ; Oscillator Selection bits (Internal oscillator, HS oscillator used by USB (INTHS))
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  PWRT = OFF            ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  BOR = ON              ; Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
  CONFIG  BORV = 3              ; Brown-out Reset Voltage bits (Minimum setting 2.05V)
  CONFIG  VREGEN = OFF          ; USB Voltage Regulator Enable bit (USB voltage regulator disabled)

; CONFIG2H
  CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
  CONFIG  WDTPS = 32768         ; Watchdog Timer Postscale Select bits (1:32768)

; CONFIG3H
  CONFIG  CCP2MX = ON           ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
  CONFIG  PBADEN = ON           ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
  CONFIG  LPT1OSC = OFF         ; Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
  CONFIG  MCLRE = OFF           ; MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)

; CONFIG4L
  CONFIG  STVREN = OFF          ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
  CONFIG  LVP = OFF             ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
  CONFIG  ICPRT = OFF           ; Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
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

; Reservación de memoria
    UDATA_ACS
    dato_a_enviar RES 1			    ; Reservamos un byte de memoria para enviar datos por SPI
    retardoMinimo_tx RES 1			    ; Reservamos un byte de memoria para retardoMinimo minimo 
    retardo_modo_tx RES 1			    ; Reservamos un byte de memoria para retardoMinimo
    dato_estado RES 1			    ; Reservamos un byte de memoria para configuraci�n del status
    dato_configuracion_C RES 1		    ; Reservamos un byte de memoria para configuraci�n del NRF24L01
    dato_configuracion_A RES 1		    ; Reservamos un byte de memoria para configuraci�n de memoria
    dato_contenido RES 1			    ; Reservamos un byte de memoria para enviar datos comprimidos
    dato_guardar RES 1			    ; Reservamos un byte de memoria para los datos retornados
    dato_guardar_leer RES 1			    ; Reservamos un byte de memoria para los datos retornados por read
    contador_i RES 2			    ; Reservamos dos byte de memoria para leer retorno de datos
    contador_j RES 1			    ; Reservamos un byte de memoria para leer retorno de datos
    resultado_operacion_tx RES 1		    ; Reservamos un byte de memoria para leer resultado de la operaci�n Tx
    leer_status_config RES 1		    ; Reservamos un byte de memoria para leer retorno de status
    trans_tx RES 1			    ; Reservamos un byte de memoria para leer valores de retransmisi�n
    proceso_finalizado RES 1		    ; Reservamos un byte de memoria para verificar escritura de datos
    register_address RES 1		    ; Reservamos un byte de memoria para registro de direcciones 
    enable_checksum RES 1			    ; Reservamos un byte de memoria para comprobar uso del checksum
    mensaje_a_enviar RES 1			    ; Reservamos un byte de memoria para env�ar el los datos que queremos transferir
    mensaje RES 1				    ; Reservamos un byte de memoria para datos a env�ar
    contador_tx RES 1			    ; Reservamos un byte de memoria para transferencia de datos

    CODE 0x00
    GOTO Configuracion				    ; Etiqueta Inicio
        
Configuracion:
    ;Config. oscilador
    MOVF OSCCON, W, ACCESS		    ; Movemos OSCCON al registro W
    IORLW b'01010011'			    ; Oscilador interno de 2Mhz
    MOVWF OSCCON, ACCESS		    ; W a OSCCON
    
    ; ADC Configuración
    ; Puerto A Entrada = 1 / Salida = 0  
    MOVLW	B'00000001'	    ;  Configurar Puerto B como entrada [A0]
    MOVWF	TRISA    
    
    ;Puerto D como salida
    CLRF TRISD, ACCESS	    ;  Configurar puerto D como salida	[D7]
    
    ; Paso 1.1 Configurar pin analógicos [Voltage de Referencia y Digital] [ADCCON1]
    MOVLW   B'00000000'     ;
    MOVWF   ADCON1		; Se configura el registro ADCCON1
    
    ; Paso 1.2 Seleccionar El canal de entrada	[ADCCON0]
    MOVLW	B'00000000'
    MOVWF	ADCON0

    ; Paso 1.3 Seleccionar tiempo de adquisición	[ADCCON2]
    ; Paso 1.4 Seleccionar Reloj de conversión	[ADCCON2]
    ; 500 ns ; 4*TAD; Justificación a la derecha
    MOVLW b'10010000'			    ; Se mueve el valor de 0x90 al registro W
    MOVWF ADCON2			    ; Se mueve lo que est� en el registro W a ADCON2
    
    ; Paso 1.5 Encender A/D módulo			[ADCCON0]
    MOVLW b'00000001'			    ; Se mueve el valor de 0x01 al registro W, para habilitar ADC
    MOVWF ADCON0			    ; Se mueve lo que est� en el registro W a ADCON0
    
    CLRF contador_tx			    ; Limpiamos la direcci�n de memoria para env�o de datos
    
    ; Configuración para el transmisor del NRF24L01
    BCF ADCCON0,0,ACCESS			    ; Deshabilitamos Conversor A/D
    CLRF PORTE				            ; Limpiar el puerto E
    
LOOP:
    CALL SPI_Start			    ; Llamamos a configuraci�n del SPI
    CALL Configuration_port		    ; LLamamos a configuraci�n del NRF24L01
    CALL Start_TX_Mode_nrf		    ; Llamamos a configuraci�n como modo Tx
    CALL Conversion_ADC			    ; Llamamos a la conversi�n del ADC
    ;*************************************************************************************
    ; Parte Alta:  ADRESH    se obtienen 2 bits
    ; Parte Baja:  ADRESL    se obtienen 8 bits 
    ;************************************************************************************
    MOVLW 0x01				    ; Movemos al registro W la cantidad de 0x01
    CPFSEQ contador_tx,0		    ; Comparamos contador_tx = 0x01, si es as� salta
    CALL Send_part_low			    ; Si contador_tx = 0x00 entonces se va a la parte baja
    CALL Send_part_high			    ; Si contador_tx = 0x01 entonces se va a la parte alta
    
message_send_rx:
   
    CALL Get_Mode_Tx			    ; Llamamos a la verificaci�n de los datos que se transfirieron Tx
    CALL Finish_operation		    ; Llamamos a deshabilitaci�n de transferencia del NRF24L01
    
    ;Verificamos si el mensaje se ha enviado correctamente o no, de igual manera si hubo un fallo del NRF24L01
    MOVF trans_tx,W,ACCESS		    ; Movemos trans_tx al registro W
    ANDLW b'00000011'			    ; Realizamos un AND con el registro W, para ver resultado
    MOVWF trans_tx,ACCESS		    ; Pasamos lo que est� en el registro W a trans_tx
    
    ;El mensaje fue env�ado correctamente
    MOVLW 0x01				    ; Movemos al registro W la cantidad 0x01
    CPFSEQ trans_tx,0			    ; Si trans_tx == 0x01, realizamos un salto...
    GOTO Fail_message			    ; Si no es igual entonces debe ser mensaje fallido o error de NRF24L01
    CALL Sent_message			    ; En el caso que sea igual, entonces el mensaje se env�o correctamente
    
Fail_message:
    ; El mensaje fue env�ado incorrectamente
    MOVLW 0x00				        ; Movemos al registro W la cantidad 0x00
    CPFSEQ trans_tx,0			    ; Si trans_tx == 0x00, realizamos un salto...
    GOTO Fail_NRF24L01			    ; Si no es igual entonces debe ser error de NRF24L01
    CALL Not_sent_message		    ; En el caso que sea igual, entonces el mensaje no se env�o correctamente
    
Fail_NRF24L01:     
    
    ;Hubo un error en el NRF24L01
    MOVLW 0x11				    ; Movemos al registro W la cantidad 0x11
    CPFSEQ trans_tx,0			    ; Si trans_tx == 0x11, realizamos un salto...
    GOTO Finish				    ; Si no es igual entonces es error de NRF24L01
    CALL Not_sent_message		    ; En el caso que sea igual, entonces el mensaje no se env�o correctamente
    
Finish:    
    
    BRA LOOP


;*******************************************************************************************************
; Función para inicializar SPI 
;*******************************************************************************************************

SPI_Start:
    CLRF PORTC				    ; Limpiamos puerto C
    CLRF PORTB				    ; Limpiasmo puerto B

    ; SCK -> B1 ( salida )
    ; SDI ( MISO ) -> B0 ( Entrada )
    BCF TRISB,1,ACCESS              ;  B1 como salida
    BSF TRISB,0,ACCESS			    ;  B0 como entrada
    
    ; SDO C7 (salida)
    BCF TRISC,7,ACCESS                      ; C7 como salida
    
    ;Configuraci�n del Registro SSPCON1
    ; WCOL  The SSPBUF register is written while it is still transmitting the previous word
    ; SSPOV Receive Overflow Indicator bit
    ; SSPEN Master Synchronous Serial Port Enable bit
    ; CKP   Clock Polarity Select bit 
    MOVLW b'11110010'               ; Limpiar registro SSPCON1 y asignar frecuencia base (fosc/64)
    MOVWF SSPCON1,ACCESS			; W a SSPCON1
    

    ;Configuraci�n del Registro SSPSTAT, realizar muestreo al final del tiempo de salida de datos por flanco de bajada
    
    BSF SSPSTAT,7,ACCESS		    ; Habilitamos muestreo por tiempo de salida de datos 
                                    ;   (SMP)  Slew Rate Control bit
    BSF SSPSTAT,6,ACCESS		    ; Habilitamos flanco de bajada 
                                    ; (CKE) SMBUS select bit
   
    ; interrupci�n SPI en el registro PIE
    BCF PIE1,3,ACCESS			    ; Ponemos el Registro PIE1 la deshabilitaci�n de la interrupci�n de MSSP
      
    ;Prioridad de interrupci�n SPI ; 1 Alta prioridad ; 0 Baja prioridad (SSPIP)
    BCF IPR1,3,ACCESS			    ; Ponemos el Registro IPR1 como baja prioridad
   
    ; configuraci�n del SPI (SSPIF)
    BCF PIR1,3,ACCESS			    ; Ponemos el Registro PIR1 en espera para transmitir o recibir
   
    ;Configuraci�n SPI en el pin 5 (SSPEN)
    BSF SSPCON1,5,ACCESS		    ; Habilita el puerto serie y configura SCK, SDO, SDI y SS
   
    RETURN				    ; Retornamos
    
;*******************************************************************************************************
; Función para inicializar los puertos necesarios para la comunicación PIC - NRF 
; (nRF24L01_Ports_Start)
;*******************************************************************************************************
Configuration_port:
    ;Configuraci�n de puertos como Digitales
    MOVF ADCON1, W, ACCESS		    ; ADCON1 al registro W
    IORLW b'00001111'			    ; Configuración pin Digital
    MOVWF ADCON1, ACCESS		    ; W  a ADCON1
    
    ;Configuraci�n -------------
    ; CE Salida
    ; CSN Salida
    ; IRQ Entrada
    ; ---------------------------
    BCF TRISB,2,ACCESS                      ; B2 como salida  (CE)
    BCF TRISB,3,ACCESS                      ; B3 como salida  (CSN)
    BSF TRISB,4,ACCESS                      ; B4 como entrada (IRQ)
    
    ;Inicializamos salida del SPI
    BCF LATB,2,ACCESS			    ; B2 (CE) en 0
    BSF LATB,3,ACCESS			    ; B3 (CSN) en 1
    
    RETURN	


;*******************************************************************************************************
; Función para inicializar el modo de transmisión y encender el  NRF11
;*******************************************************************************************************
Start_TX_Mode_nrf:
    
    BCF LATB,2,ACCESS			    ; CE con 0
    
    CALL Reset_configuration		    ; Reseteamos la configuraci�n de NRF24L01 en el caso que exista algo
    
    ;Habilitar reconocimiento Autom�tico de Receptor
    ;Accedemos a la direcci�n de reconocimiento Autom�tico
    MOVF dato_configuracion_A, W, ACCESS    ;  dato_configuracion_A al registro W
    ANDLW b'00000001'			    ;  Modo Rx
    MOVWF dato_configuracion_A, ACCESS	    ; W pasarlo a dato_configuracion_A
    
    ;Habilitamos el reconocimiento Autom�tico
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'00111111'			    ; Realizamos una AND con el registro W, habilitamos el Modo Rx
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para reconocimiento Autom�tico
    
    
    ;Habilitar Receptor
    ;Accedemos a la direcci�n de receptor
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000010'			    ; Realizamos una AND con el registro W, direcci�n para habilitar Rx
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Habilitamos el reconocimiento Autom�tico
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'00111111'			    ; Realizamos una AND con el registro W, habilitamos el Rx
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para configuraci�n de Receptor
    
    
    ;Habilitamos ancho de direcci�n para transmisor y receptor
    ;Accedemos a la direcci�n para habilitar el ancho de direcciones
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000011'			    ; Realizamos una AND con el registro W, direcci�n para ancho de FR
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Ponemos como ancho de banda 5 bytes, para transmisor y receptor
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'00000011'			    ; Realizamos una AND con el registro W, ponemos 5 bytes de ancho de de direcciones
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para ancho de direcciones
    
    
    ;Configuramos retransmisi�n autom�tica, contemplando tiempo de retraso y cantidad m�xima de intentos
    ;Accedemos a la direcci�n para retransmisi�n autom�tica
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000100'			    ; Realizamos una AND con el registro W, direcci�n para retransmisi�n
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Configuramos retraso autom�tico
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'00000000'			    ; Realizamos una AND con el registro W, y ponemos valor de retraso autom�tico
    MULLW b'00010000'			    ; Multiplicamos con el registro W, para obtener el tiempo total de retraso
    ADDLW b'00001010'			    ; Adicionamos con el registro W, para determinar cantidad m�xima de autotransmisi�n
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para retraso autom�tico
    
    ;Definimos frecuencia Freq = 2400 + 64 = 2464
    ;Accedemos a la direcci�n para anexar frecuencia
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000101'			    ; Realizamos una AND con el registro W, direcci�n para frecuencia
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Configuramos frecuencia, teniendo como base 2400. Anexamos 64 hz faltantes
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'01000000'			    ; Realizamos una AND con el registro W, y ponemos los 64 Hz
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para frecuencia a utilizar
    
    ;Configuraci�n de velocidad de RF, potencia y ganancia de amplificador LNA
    ;Accedemos a la direcci�n para configuraci�n RF
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000110'			    ; Realizamos una AND con el registro W, direcci�n para frecuencia
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Configuramos velocidad de RF como 1 Mbps, potencia a 2.4GHz y ganancia
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'00000000'			    ; Realizamos una AND con el registro W, para definir velocidad de RF
    ADDLW b'00000110'			    ; Realizamos una ADD con el registro W, para determinar potencia m�xima
    ADDLW b'00000001'			    ; Adicionamos con el registro W, la ganancia de CRC de detecci�n de errores
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para transmisor y receptor (RF)
    
    ;Configuraci�n de FIFO Rx y ancho de banda de checksum en caso de errores
    ;Accedemos a la direcci�n para reservar FIFO Rx
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00001011'			    ; Realizamos una AND con el registro W, direcci�n para FIFO Rx
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Configuramos ancho de banda de checksum mediante payload de 8 bytes y habilitamos cheksum
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'00001000'			    ; Realizamos una AND con el registro W, para definir ancho de checksum
    ADDLW b'00000001'			    ; Adicionamos con el registro W, la habilitaci�n de checksum
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para FIFO Rx y checksum
    

    ;Configuraci�n modo Tx habilitado y codificaci�n CRC de 2 bytes para detecci�n de errores
    ;Accedemos a la direcci�n para configuraci�n general
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000000'			    ; Realizamos una AND con el registro W, direcci�n general
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Configuramos de modo Tx y habilitar detecci�n de errores
    MOVF dato_configuracion_C, W, ACCESS    ; Movemos dato_configuracion_C al registro W
    ANDLW b'00001000'			    ; Realizamos una AND con el registro W, para habilitar CRC
    ADDLW b'00000100'			    ; Adicionamos con el registro W, para determinar 2 bytes en CRC
    ADDLW b'00000010'			    ; Adicionamos con el registro W, para poner POWER DOWN
    ADDLW b'00000000'			    ; Adicionamos con el registro W, para poner Modo Tx
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para Modo Tx y detecci�n de errores
    
    MOVF retardo_modo_tx, W, ACCESS	    ; Movemos retardo_modo_tx al registro W
    ANDLW b'00110010'			    ; Realizamos una AND con el registro W, para designar tiempo
    MOVWF retardo_modo_tx, ACCESS		    ; Lo que esta en el registro W pasarlo a retardo_modo_tx
    CALL retardoMinimo			    ; Esperamos un tiempo m�nimo
    
    
    RETURN
; ***********************************************************************************************************
; Reset NRF24L01
; ***********************************************************************************************************
Reset_configuration:   
    
    BCF LATB,3,ACCESS			    ;  CSN en 0 para activarlo
    
    ;Enviar datos del SSBUF mientras recibe estado de transferencia
    MOVF dato_a_enviar, W, ACCESS	;  dato_a_enviar al registro W
    IORLW b'11100001'			    ;  limpiar FIFO en Tx
    MOVWF dato_a_enviar, ACCESS		;  W pasarlo a dato_a_enviar
    CALL SPI_transfer			    ; Transferencia a NRF
    
    BSF LATB,3,ACCESS			    ; CSN en 1 para desactivarlo
    
    MOVF retardo_modo_tx, W, ACCESS	    ;retardo_modo_tx al registro W
    IORLW b'00000101'			        ;   designar tiempo de espera
    MOVWF retardo_modo_tx, ACCESS		    ; W pasarlo a retardo_modo_tx
    CALL retardoMinimo			            ; llamar a delay
    
    BCF LATB,3,ACCESS			    ;  CSN en 0 para activarlo
    
    ;Recibir datos del SSBUF mientras esta transfiriendo 
    ; (FLUSH_RX)
    MOVF dato_a_enviar, W, ACCESS	    ; dato_a_enviar al registro W
    IORLW b'11100010'			        ; limpiar FIFO en Rx
    MOVWF dato_a_enviar, ACCESS		    ; W pasarlo a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
    
    BSF LATB,3,ACCESS			    ;  CSN en 1 para desactivarlo
    
    ;En la configuraci�n del Status para saber si llegan nuevos datos, eliminamos el estatus RX_DR
    ; RX_DR
    MOVF dato_contenido,	W, ACCESS	    ; dato_contenido al registro W
    IORLW b'01000000'			    ; limpiar estatus RX_DR
    MOVWF dato_contenido, ACCESS		    ;  W pasarlo a dato_contenido
    CALL write_status_nrf			    ; Realizamos escritura en status

    ;En la configuraci�n del Status para saber si los datos se env�an, eliminamos el estatus TX_DR
    ; (TX_DR)
    MOVF dato_contenido,	W, ACCESS	    ; dato_contenido al registro W
    IORLW b'00100000'			    ; limpiar estatus TX_DR
    MOVWF dato_contenido, ACCESS		    ; o W pasarlo a dato_contenido
    CALL write_status_nrf			    ; Realizamos escritura en status
    
    ; eliminamos el estatus MAX_RT
    ; MAX_RT
    MOVF dato_contenido,	W, ACCESS	    ; dato_contenido al registro W
    IORLW b'00010000'			    ; limpiar estatus MAX_RT
    MOVWF dato_contenido, ACCESS		    ;  W pasarlo a dato_contenido
    CALL write_status_nrf	    		    ; Realizamos escritura en status
    RETURN

;-------------------------------------------Env�o parte alta------------------------------------------------    
Send_part_high:
    
    MOVFF ADRESH , mensaje		    ; Pasamos la parte alta del ADC hacia el mensaje que se quiere env�ar en message
    CALL Send_Data_TX_Mode_nRF24L01	    ; Llamamos al env�o de datos
    BCF contador_tx,0,ACCESS		    ; Ponemos contador_tx = 0x00 para que se vaya a la parte baja
    
    GOTO message_send_rx		    ; Regresamos a la etiqueta message_send_rx

;-------------------------------------------Env�o parte baja------------------------------------------------    
Send_part_low: 
    
    MOVFF ADRESL , mensaje		    ; Pasamos la parte baja del ADC hacia el mensaje que se quiere env�ar en message
    CALL Send_Data_TX_Mode_nRF24L01	    ; Llamamos al env�o de datos
    BSF contador_tx,0,ACCESS		    ; Ponemos contador_tx = 0x01 para que se vaya a la parte alta
    
    GOTO message_send_rx		    ; Regresamos a la etiqueta message_send_rx
    
;--------------------------------------------Conversi�n del ADC------------------------------------------------
Conversion_ADC:
    ;Ponemos en marcha el ADC
    MOVLW	b'00000011'		    ; Movemos 0x03 al registro W
    MOVWF	ADCON0			    ; Lo que est� en el registro W lo pasamos a ADCON0
    CALL VerificarBandera				    ; Llamamos a la subrutina Flag
    RETURN
    
;-----------------------------------------Verificaci�n del ADC-----------------------------------------------
VerificarBandera:
    MOVLW .3
    XORWF ADCON0, W
    BTFSC STATUS, Z	    ; Leer bandera de cero
    BRA	  VerificarBandera  ; Z=1 ADCON[GO/DONE'] = 0
    RETURN				    ; En caso de Z = 0, retornamos 
    
;------------------------------------Si el mensaje se env�o correctamente--------------------------------------
Sent_message:
    BSF LATE,0,ACCESS                        ; Se usa E0 para verificar si el dato ha sido enviado de manera correcta
    MOVF retardo_modo_tx, W, ACCESS	    ; Movemos retardo_modo_tx al registro W
    ANDLW b'00011110'			        ; Realizamos una AND con el registro W, para designar tiempo
    MOVWF retardo_modo_tx, ACCESS		    ; Lo que esta en el registro W pasarlo a retardo_modo_tx
    CALL retardoMinimo			        ; Esperamos un tiempo m�nimo

    BCF LATE,0,ACCESS			        ; Desactivamos LATE pin 0 

    RETURN

;------------------------------------Si el mensaje no se env�o correctamente--------------------------------------
Not_sent_message:
   BSF LATE,1,ACCESS                      ; E1 para verificar si el dato no fue enviado
   
   ;Esperamos un tiempo para visualizar en led
   MOVF retardo_modo_tx, W, ACCESS	    ; Movemos retardo_modo_tx al registro W
   ANDLW b'00011110'			    ; Realizamos una AND con el registro W, para designar tiempo
   MOVWF retardo_modo_tx, ACCESS		    ; Lo que esta en el registro W pasarlo a retardo_modo_tx
   CALL retardoMinimo			    ; Esperamos un tiempo m�nimo
   
   BCF LATE,1,ACCESS			    ; Desactivamos LATE pin 1
   
   RETURN   
 
;---------------------------------------El NRF24L01 tuvo un error--------------------------------------------
Error_NRF24L01:
    ; E2 (O) = error
   BSF LATE,2,ACCESS                      ; E2 para error del NRF
   
   ;Esperamos un tiempo para visualizar en led
   MOVF retardo_modo_tx, W, ACCESS	    ; Movemos retardo_modo_tx al registro W
   ANDLW b'00011110'			    ; Realizamos una AND con el registro W, para designar tiempo
   MOVWF retardo_modo_tx, ACCESS		    ; Lo que esta en el registro W pasarlo a retardo_modo_tx
   CALL retardoMinimo			    ; Esperamos un tiempo m�nimo
   
   BCF LATE,2,ACCESS			    ; Desactivamos LATE pin 2
   
   GOTO Finish				    ; Regresamos a subrutina Finish  
   
   
 
;-------------------------------------Finalizar tranferencia NRF24L01----------------------------------------      
Finish_operation:
    
    ; Lea el registro general anterior para cambiar solo el bit PWR_UP
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000000'			    ; Realizamos una AND con el registro W, 
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    CALL Read_configuration		    ; Llamamos a la subrutina Read_configuration
    
    
    ;Habilitar bit para habilitar transmisor como bajo
    ;Accedemos a la direcci�n del registro general
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00000000'			    ; Realizamos una OR con el registro W, direcci�n para Modo Rx
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    
    ;Deshabilitamos el el transmisor
    MOVF dato_guardar_leer, W, ACCESS	    ; Movemos dato_guardar_leer al registro W
    ANDLW b'11111101'			    ; Realizamos una OR con el registro W, habilitamos el Modo Rx
    MOVWF dato_configuracion_C, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_C
    CALL write_nrf_register		    ; Escribimos la configuraci�n para reconocimiento Autom�tico
     
    RETURN
    
    
 
    
;----------------------------------Obtener informe de status del NRF24L01----------------------------------------
Get_Mode_Tx:   
    
   ; Inicializamos los contadores
   MOVF contador_i, W, ACCESS		    ; Movemos contador_i al registro W
   ANDLW b'00000000'			    ; Realizamos un AND con contador_i, para inicializar en 0
   MOVWF contador_i, ACCESS		    ; Lo que esta en el registro W pasarlo a contador_i
   
   MOVF contador_i+1, W, ACCESS		    ; Movemos contador_i+1 al registro W
   ANDLW b'00000000'			    ; Realizamos un AND con contador_i+1, para inicializar en 0
   MOVWF contador_i+1, ACCESS		    ; Lo que esta en el registro W pasarlo a contador_i+1
   
   MOVF contador_j, W, ACCESS		    ; Movemos contador_j al registro W
   ANDLW b'00000000'			    ; Realizamos un AND con contador_j, para inicializar en 0
   MOVWF contador_j, ACCESS		    ; Lo que esta en el registro W pasarlo a contador_j
   
; Esperamos hasta que el n�mero m�ximo de interrupciones de reintentos de TX o datos enviados TX FIFO
; est�n en alto y luego IRQ se establezca en bajo
Wait_IRQ_LOW:
    
    MOVLW 0xFF				    ; Movemos al registro W 0xFF
    CPFSEQ contador_i,0			    ; Si contador_i == 255, realiza un salto...
    CALL Increment_i_1			    ; En el caso que no realice un salto se va a la subrutina Increment_i_1
    CALL Increment_i_2			    ; En el caso que contador_i == 255, entonces saltamos a la subrutina Increment_i_2

Jump_cont:
    ;Realizamos un total de 333 muestreos para la comprobaci�n de que IRQ est� en bajo
    MOVLW 0x4E				    ; Movemos al registro W 0x4E, el resto de datos para formar 014D
    CPFSEQ contador_i+1,0		    ; Si contador_i == 78, realizamos un salto...
    GOTO Jump_cont_2			    ; Si no es igual, salta hacia Jump_cont_2
    CALL Increment_i_j			    ; Si es igual a 78 bits contador_i+1, entonces saltamos a la subrutina Increment_i_j 

Jump_cont_2:
    ;Ponemos como limite 7 veces la cantidad de muestreos que se realizar� 
    MOVLW 0x07				    ; Movemos al registro W 0x07
    CPFSEQ contador_j,0			    ; Si contador_j == 7, realizamos un salto...
    GOTO Get_IRQ			    ; Si no es igual, salta hacia Get_IRQ
    CALL Error_IRQ			    ; Si es igual entonces saltamos a la subrutina Error_IRQ
    
Get_IRQ:    
    ; IRQ = B4
    MOVF TRISB, W, ACCESS		    ; Metemos entrada del TRIS B al registro W
    ANDLW b'00000000'			    ; Realizamos un AND con b'00000000' para verificar si el pin 4, esta en bajo.
    CPFSEQ TRISB,0			        ; Si TRISB est� en alto salta..
    BRA Wait_IRQ_LOW			    ; En caso contrario se repite y regresa a Wait_IRQ_LOW
    
    
; Esperamos hasta que el n�mero m�ximo de interrupciones de reintentos de TX o datos enviados TX al FIFO est�n en alto
Wait_TX_DS_or_MAX_RT:
    
    CALL Read_status			    ; Llamamos la subrutina Read_status, para obtener status de IRQ
    
    ;Comprobamos si el n�mero m�ximo de interrupciones en Tx est� en alto
    MOVF leer_status_config, W, ACCESS	    ; Pasamos leer_status_config al registro W
    ANDLW b'00010000'			            ; Realizamos un AND con el registro W, para saber si est� el n�mero m�ximo de interrupciones
    MOVWF leer_status_config, ACCESS	    ; Lo que esta en el registro W se pasa a leer_status_config 
    
    MOVLW   0x10			    ; Movemos al registro W 0x10
    CPFSEQ leer_status_config,0		    ; En el caso que leer_status_config == 0x10 entonces, salta...
    GOTO Jump_status			    ; En el caso que no sean iguales se va Jump_status
    CALL MAX_RT_status			    ; En caso de ser iguales, llamamos a la subrutina MAX_RT_status
    
Jump_status: 
    
    ;Comprobamos los datos env�amos por Tx al FIFO si esta en alto
    MOVF leer_status_config, W, ACCESS	    ; Pasamos leer_status_config al registro W
    ANDLW b'00100000'			    ; Realizamos un AND con el registro W, para saber los datos del FIFO en Tx
    MOVWF leer_status_config, ACCESS	    ; Lo que esta en el registro W se pasa a leer_status_config 
    
    MOVLW   0x20			    ; Movemos al registro W 0x20
    CPFSEQ leer_status_config,0		    ; En el caso que leer_status_config == 0x20 entonces, salta...
    GOTO Jump_status_2			    ; En el caso que no sean iguales se va Jump_status
    CALL TX_DS_status			    ; En caso de ser iguales, llamamos a la subrutina TX_DS_status
    
Jump_status_2:    
    MOVF TRISB, W, ACCESS		    ; Metemos entrada del TRIS B al registro W
    ANDLW b'00001000'			    ; Realizamos un AND con b'00001000' para verificar si el pin 4, no esta en bajo.
    CPFSEQ TRISB,0			    ; Si TRISB est� en alto salta..
    BRA Wait_TX_DS_or_MAX_RT		    ; En caso contrario se repite y regresa a Wait_TX_DS_or_MAX_RT
 
Finish_status:    
  
    RETURN
    
 
;-----------------------------Incremento de contador_i, primeros 255 bits------------------------------------
Increment_i_1:
    INCF contador_i,F,0                     ; Incrementa el contenido de la localidad de memoria contador_i
    GOTO Jump_cont			    ; Regresamos a Jump_cont

;-----------------------------Incremento de contador_i+1, primeros 78 bits------------------------------------   
Increment_i_2:
    INCF contador_i+1,F,0		    ; Incrementa el contenido de la localidad de memoria contador_i+1
    GOTO Jump_cont			    ; Regresamos a Jump_cont
    
;--------------------------Incremento de contador_j y poner en cero contador_i--------------------------------
Increment_i_j:
    
    ;Inicializamos contador_i y contador_i+1 con 0x00
    MOVF contador_i, W, ACCESS		    ; Movemos contador_i al registro W
    ANDLW b'00000000'			    ; Realizamos un AND con contador_i, para inicializar en 0
    MOVWF contador_i, ACCESS		    ; Lo que esta en el registro W pasarlo a contador_i
   
    MOVF contador_i+1, W, ACCESS		    ; Movemos contador_i+1 al registro W
    ANDLW b'00000000'			    ; Realizamos un AND con contador_i+1, para inicializar en 0
    MOVWF contador_i+1, ACCESS		    ; Lo que esta en el registro W pasarlo a contador_i+1
   
    INCF contador_j,F,0			    ; Incrementa el contenido de la localidad de memoria contador_j
    GOTO Jump_cont_2			    ; Regresamos a Jump_cont_2
    
;-------------------------------------Comprobamos el error de IRQ--------------------------------------------    
Error_IRQ:
    
    ;Ponemos valor fijo en el caso que exista el error en IRQ
    MOVF resultado_operacion_tx, W, ACCESS     ; Movemos resultado_operacion_tx al registro W
    ANDLW b'00000011'			    ; Realizamos una AND con el registro W, para poner fijo un valor
    MOVWF resultado_operacion_tx, ACCESS	    ; Lo que esta en el registro W pasarlo a resultado_operacion_tx
  
    MOVFF resultado_operacion_tx,trans_tx	    ; Pasamos el resultado de Read_configuration a trans_tx
    
    GOTO Wait_TX_DS_or_MAX_RT		    ; Regresamos a Wait_TX_DS_or_MAX_RT
    
    
;-------------------------------------Se escribe para env�ar datos-------------------------------------------
Send_Data_TX_Mode_nRF24L01:
    
    ; Pasamos el mensaje que queremos env�ar 
    MOVFF message,mensaje_a_enviar	    ; message a mensaje_a_enviar
    
    ;Definimos la direcci�n para transmitir
    MOVLW 0x10				    ; Mover TX_ADDR a W en la direci�n de memoria register_address
    CALL Write_nrf_Address_Register	    ; Llamamos la subrutina Write_nrf_Address_Register
    
    ;Definimos la direcci�n de recibir datos con la misma direcci�n para transmitir
    MOVLW 0x0A				    ; Mover RX_ADDR_P0 a W en la direcci�n de de memoria register_address
    CALL Write_nrf_Address_Register	    ; Llamamos la subrutina Write_nrf_Address_Register
    
    ; Definimos enable_checksum como 1
    MOVF enable_checksum, W, ACCESS	     ; Movemos enable_checksum al registro W
    ANDLW b'00000001'			     ; Realizamos un AND con el registro W
    MOVWF enable_checksum, ACCESS	     ; Lo que est� en el registro W lo pasamos a enable_checksum
    
    ; Env�amos los datos que deseamos
    CALL Write_nrf_TX_Payload		     ; LLamamos a la subrutina Write_nrf_Tx_Payload
    
    BSF LATB,2,ACCESS			     ; Habilitamos el CE con 1
    
    ;Esperamos un tiempo por habilitaci�n del CE
    MOVF retardo_modo_tx, W, ACCESS	    ; Movemos retardo_modo_tx al registro W
    ANDLW b'11001000'			    ; Realizamos una AND con el registro W, para designar tiempo
    MOVWF retardo_modo_tx, ACCESS		    ; Lo que esta en el registro W pasarlo a retardo_modo_tx
    CALL retardoMinimo			    ; Esperamos un tiempo m�nimo
  
    BCF LATB,2,ACCESS			     ; Desabilitamos el CE con 0
    
    ;Esperamos un tiempo por habilitaci�n del CE
    MOVF retardo_modo_tx, W, ACCESS	    ; Movemos retardo_modo_tx al registro W
    ANDLW b'00110010'			    ; Realizamos una AND con el registro W, para designar tiempo
    MOVWF retardo_modo_tx, ACCESS		    ; Lo que esta en el registro W pasarlo a retardo_modo_tx
    CALL retardoMinimo			    ; Esperamos un tiempo m�nimo
    
    RETURN
    

;---------------------------------------Escritura del mensaje a env�ar---------------------------------------
Write_nrf_TX_Payload:
   
    BCF LATB,3,ACCESS			    ; Ponemos a CSN en 0 para activarlo
    
    ;Enviar palabra de instrucci�n mientras recibe status
    MOVLW 0xA0				    ; Se almacena W_TX_PAYLOAD en W
    MOVWF dato_a_enviar, ACCESS		    ; Se copia W_TX_PAYLOAD a variable para enviar
    CALL SPI_transfer			    ; Llamamos la subrutina SPI_transfer
    
    MOVLW 0x01				    ; Pasamos al registro W, la cantidad de 0x01
    CPFSEQ enable_checksum,0		    ; verificamos si enable_checksum == 0x01
    CALL Checksum_fail			    ; Si es distinto el checksum no se est� utilizando
    CALL Checksum_no_fail		    ; Si es igual entonces, el checksum se esta utilizando
    
Finish_checksum:
    
    BSF LATB,3,ACCESS			    ; Ponemos a CSN en 1 para desactivarlo
    
    RETURN
    
;-------------------------------------Cuando hay un error en el checksum-------------------------------------;    
Checksum_fail:
    
    ; Mensaje que se quiere env�ar
    MOVFF mensaje_a_enviar, dato_a_enviar	    ; Movemos los datos que queremos env�ar a dato_a_enviar
    MOVWF dato_a_enviar, ACCESS		    ; Se copia W_TX_PAYLOAD a variable para enviar
    CALL SPI_transfer			    ; Llamamos la subrutina SPI_transfer
    
    GOTO Finish_checksum
    
    
;-------------------------------------Cuando no hay error en el checksum-------------------------------------;    
Checksum_no_fail:
    
    ; Mensaje que se quiere env�ar
    MOVFF mensaje_a_enviar, dato_a_enviar	    ; Movemos los datos que queremos env�ar a dato_a_enviar
    MOVWF dato_a_enviar, ACCESS		    ; Se copia W_TX_PAYLOAD a variable para enviar
    CALL SPI_transfer			    ; Llamamos la subrutina SPI_transfer
    
    ; Actualizamos resultado del mensaje
    MOVF dato_guardar, W, ACCESS		    ; Pasamos byte para limpiar datos al registro W 
    ADDLW b'00000000'			    ; Se realiza una adici�n con el registro W, para limpiar mensaje env�ado
    MOVWF dato_a_enviar, ACCESS		    ; Pasamos lo que esta en el registro W a dato_a_enviar
    CALL SPI_transfer			    ; Llamamos la subrutina SPI_transfer
    
    GOTO Finish_checksum   
    
    
;---------------------------------Escribiendo en el registro de direcciones---------------------------------
Write_nrf_Address_Register:
    
    CLRF proceso_finalizado		    ; Se coloca en cero la variable que determina si ha terminado el proceso (bandera)

; El proceso de escribir la direcci�n se repite hasta que se confirma
Wait_write_address:
    
    BCF LATB,3,ACCESS			    ; Ponemos a CSN en 0 para activarlo
    
    ;Escribimos en la direcci�n de registro una instrucci�n
    ADDLW b'00100000'			    ; Realizamos una ADD con el registro W y TX_ADDR (W_REGISTER + TX_ADDR)
    MOVWF dato_a_enviar, ACCESS		    ; Lo que esta en el registro W pasarlo a dato_a_enviar
    MOVWF register_address, ACCESS	    ; Lo que esta en el registro W pasarlo a register_address
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
    
    ;Ponemos las direcci�n de transmisor y receptor
    MOVLW 0xB2				    ; Direcci�n transmisi�n[1] se carga a W
    MOVWF dato_a_enviar, ACCESS		    ; Se almacena dir_trans[1] a dato_a_enviar
    CALL SPI_transfer			    ; Enviar dato[1]
    
    ;Iteraci�n 2 
    MOVLW 0xB2				    ; Direcci�n transmisi�n[1] se carga a W
    MOVWF dato_a_enviar, ACCESS		    ; Se almacena dir_trans[1] a dato_a_enviar
    CALL SPI_transfer			    ; Enviar dato[1
    
    ;Iteraci�n 3 
    MOVLW 0xB3				    ; Direcci�n transmisi�n[1] se carga a W
    MOVWF dato_a_enviar, ACCESS		    ; Se almacena dir_trans[1] a dato_a_enviar
    CALL SPI_transfer			    ; Enviar dato[1]
    
    ;Iteraci�n 4 
    MOVLW 0xB4				    ; Direcci�n transmisi�n[1] se carga a W
    MOVWF dato_a_enviar, ACCESS		    ; Se almacena dir_trans[1] a dato_a_enviar
    CALL SPI_transfer			    ; Enviar dato[1]
    
    ;Iteraci�n 5
    MOVLW 0x01				    ; Direcci�n transmisi�n[1] se carga a W
    MOVWF dato_a_enviar, ACCESS		    ; Se almacena dir_trans[1] a dato_a_enviar
    CALL SPI_transfer			    ; Enviar dato[1]
    
    BSF LATB,3,ACCESS			    ; Ponemos a CSN en 1 para desactivarlo

    CALL Read_nRF24L01_Address_Register     ; Verificamos direcciones 
    
    BSF proceso_finalizado, 0, ACCESS	    ; Ponemos 0x01 en proceso_finalizado
    
    ; Verificamos si es correcto la escritura
    MOVF dato_guardar, W, ACCESS		    ; Movemos dato_guardar al registro W
    CPFSEQ dato_a_enviar,0		    ; Comparamos dato_a_enviar = dato_guardar, si es igual salta...
    CLRF proceso_finalizado		    ; En el caso que sean distintos entonces se limpia proceso_finalizado

    ;Verificamos proceso que sea bajo, ya que no se ha terminado la transferencia de datos
    MOVF proceso_finalizado, W, ACCESS	    ; Movemos proceso_finalizado al registro W
    IORLW b'00000000'			    ; Realizamos una OR con el registro W, para verificar proceso
    MOVWF proceso_finalizado, ACCESS	    ; Lo que esta en el registro W pasarlo a proceso_finalizado
    
    MOVLW 0x00				    ; Movemos al registro W, la cantidad de 0x00
    CPFSEQ proceso_finalizado,0		    ; Comparamos proceso_finalizado = 0x00, si es igual salta...
    GOTO Exit_write			    ; En caso que sea distinto 
    BRA Wait_write_address		    ; En caso que sea igual a cero
    
Exit_write:    
    
    RETURN
    
    
;-----------------------------------Lectura de la direcci�n de registros--------------------------------------    
Read_nRF24L01_Address_Register:
    
    BCF LATB,3,ACCESS			    ; Ponemos a CSN en 0 para activarlo
    
    ;Recibimos datos mientras se env�an datos ficticios 
    MOVFF register_address, dato_a_enviar    ; Movemos la direcci�n del registro a dato_a_enviar
    MOVLW 0x00				    ; Movemos 0x00 al registro W
    MOVWF dato_a_enviar, ACCESS		    ; Pasamos lo que est� en el registro W a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
  
    BSF LATB,3,ACCESS			    ; Ponemos a CSN en 1 para desactivarlo  
    
    RETURN
    
;------------------------------Verificamos si no existe errores en interrupciones----------------------------
MAX_RT_status:
    
    ; Obtenemos el restablecimiento del contador de retransmisi�n despu�s de cada nueva transmisi�n de paquetes
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00001000'			    ; Realizamos una AND con el registro W, verificamos Tx
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    CALL Read_configuration		    ; Llamamos a la subrutina Read_configuration
  
    MOVF dato_guardar_leer, W, ACCESS	    ; Movemos dato_guardar_leer al registro W
    ANDLW b'00001111'			    ; Realizamos una AND con el registro W, verificamos Tx
    MULWF b'00000100'			    ; Realizamos una Multiplicaci�n para el restablecimiento
    ADDLW b'00000000'			    ; Realizamos una Adici�n con el registro W, para decir que los datos no se env�aron correctamente
    MOVWF dato_guardar_leer, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_guardar_leer
     
    MOVFF dato_guardar_leer,trans_tx	    ; Pasamos el resultado de Read_configuration a trans_tx
     
    ; Borramos en status la verificaci�n de Tx
    MOVF dato_contenido, W, ACCESS	    ; Movemos dato_contenido al registro W
    ANDLW b'00010000'			    ; Realizamos una AND con el registro W, para borrar verificaci�n de Tx
    MOVWF dato_contenido, ACCESS		    ; Lo que esta en el registro W pasarlo a dato_contenido
    CALL write_status_nrf			    ; Llamamos a la subrutina write_status_nrf
    
    GOTO Jump_status			    ; Regresamos a Jump_status
    
    
;------------------------------Verificamos los datos se env�an del Tx al FIFO---------------------------------
TX_DS_status:
    
    ;Los datos se enviaron correctamente
    MOVF resultado_operacion_tx, W, ACCESS     ; Movemos resultado_operacion_tx al registro W
    ANDLW b'00000001'			    ; Realizamos una AND con el registro W, para poner fijo un valor
    MOVWF resultado_operacion_tx, ACCESS	    ; Lo que esta en el registro W pasarlo a resultado_operacion_tx
    
    ; Obtenemos el restablecimiento del contador de retransmisi�n despu�s de cada nueva transmisi�n de paquetes
    MOVF dato_configuracion_A, W, ACCESS    ; Movemos dato_configuracion_A al registro W
    ANDLW b'00001000'			    ; Realizamos una AND con el registro W, verificamos Tx
    MOVWF dato_configuracion_A, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_configuracion_A
    CALL Read_configuration		    ; Llamamos a la subrutina Read_configuration
  
    MOVF dato_guardar_leer, W, ACCESS	    ; Movemos dato_guardar_leer al registro W
    ANDLW b'00001111'			    ; Realizamos una AND con el registro W, verificamos Tx
    MULWF b'00000100'			    ; Realizamos una Multiplicaci�n para el restablecimiento
    ADDLW b'00000001'			    ; Realizamos una Adici�n con el registro W, para decir que los datos se env�aron correctamente
    MOVWF dato_guardar_leer, ACCESS	    ; Lo que esta en el registro W pasarlo a dato_guardar_leer
    
    MOVFF dato_guardar_leer,trans_tx	    ; Pasamos el resultado de Read_configuration a trans_tx
    
   ; Borramos en status la verificaci�n de paquetes en FIFO de Tx
    MOVF dato_contenido, W, ACCESS	    ; Movemos dato_contenido al registro W
    ANDLW b'00100000'			    ; Realizamos una AND con el registro W, para borrar verificaci�n de Tx
    MOVWF dato_contenido, ACCESS		    ; Lo que esta en el registro W pasarlo a dato_contenido
    CALL write_status_nrf			    ; Llamamos a la subrutina write_status_nrf
    
    GOTO Finish_status			    ; Regresamos a Finish_status
    
     
;----------------------------------Verificamos el estatus de la lectura--------------------------------------
Read_status:
    
    BCF LATB,3,ACCESS			    ; Ponemos a CSN en 0 para activarlo
   
    ;Enviar datos, sin instrucci�n para obtener estatus
    MOVF dato_a_enviar, W, ACCESS	    ; Movemos dato_a_enviar al registro W
    IORLW b'11111111'			    ; Realizamos una OR con el registro W, para leer status
    MOVWF dato_a_enviar, ACCESS		    ; Lo que esta en el registro W pasarlo a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
    
    BSF LATB,3,ACCESS			    ; Ponemos a CSN en 1 para desactivarlo
    
    MOVFF dato_guardar,leer_status_config	    ; Movemos el resultodo del status a leer_status_config
   
    RETURN
    


;----------------------------------Escribiendo datos en configuraci�n----------------------------------------
write_nrf_register:
    
    BCF LATB,3,ACCESS			    ; Ponemos a CSN en 0 para activarlo
    
    ;Enviar datos por words para configuraci�n
    MOVFF dato_configuracion_A,dato_a_enviar ; Pasamos los datos de la direcci�n de memoria dato_configuracion_A a dato_a_enviar
     
    ;Enviamos la word mientras se recibe el estatus
    MOVF dato_a_enviar, W, ACCESS	    ; Movemos dato_a_enviar al registro W
    ADDLW b'00100000'			    ; Realizamos una ADD con el registro W, para escribir.
    MOVWF dato_a_enviar, ACCESS		    ; Lo que esta en el registro W pasarlo a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
    
    ;Enviar datos por words para memoria
    MOVFF dato_configuracion_C,dato_a_enviar ; Pasamos los datos de la direcci�n de memoria dato_configuracion_C a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01 

    BSF LATB,3,ACCESS			    ; Ponemos a CSN en 1 para desactivarlo
     
    ;Enviar datos por words para memoria
    MOVFF dato_configuracion_A,dato_a_enviar ; Pasamos los datos de la direcci�n de memoria dato_configuracion_A a dato_a_enviar
    CALL Read_configuration		    ; Realizamos lectura de transferencia 
    
;Configuramos la transferencia de datos y verificamos que los datos se enviaron bien
Wait_transfer_write:
    
    BCF LATB,3,ACCESS			    ;  CSN en 0 para activarlo
    
    ;Enviar datos por words para configuraci�n
    MOVFF dato_configuracion_A,dato_a_enviar ; dato_configuracion_A a dato_a_enviar
     
    ;Enviamos la word mientras se recibe el estatus
    MOVF dato_a_enviar, W, ACCESS	    ;  dato_a_enviar al registro W
    ADDLW b'00100000'			    ; Realizamos una ADD con el registro W, para escribir.
    MOVWF dato_a_enviar, ACCESS		    ;  W pasarlo a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01 
    
    ;Enviar datos por words para memoria
    MOVFF dato_configuracion_C,dato_a_enviar ;  dato_configuracion_C a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01 
 
    BSF LATB,3,ACCESS			    ; CSN en 1 para desactivarlo
    
    MOVF dato_guardar_leer, W, ACCESS	    ; dato_guardar_leer al registro W, sin perder el dato
    CPFSEQ dato_configuracion_C,0	    ; Comparamos retorno de dato_guardar_leer  con dato_configuracion_C
    BRA Wait_transfer_write
   
    RETURN
    
;----------------------------------Lectura datos de configuraci�n--------------------------------------------
Read_configuration:
    
    BCF LATB,3,ACCESS			    ;  CSN en 0 para activarlo
    
    ;Enviar datos por words para configuraci�n
    MOVFF dato_configuracion_A,dato_a_enviar ;  dato_configuracion_A a dato_a_enviar
     
    ;Enviar instrucci�n de lectura 
    MOVF dato_a_enviar, W, ACCESS	    ; ata_to_send al registro W
    ADDLW b'00000000'			    ; Realizamos una ADD con el registro W, para leer.
    MOVWF dato_a_enviar, ACCESS		    ; W pasarlo a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
    
    ;Recibir datos mientras se env�a datos ficticios 
    MOVF dato_a_enviar, W, ACCESS	    ; dato_a_enviar al registro W
    IORLW b'00000000'			    ; Realizamos una ADD con el registro W, para leer.
    MOVWF dato_a_enviar, ACCESS		    ;  W pasarlo a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
    
    BSF LATB,3,ACCESS			    ; CSN en 1 para desactivarlo
    
    MOVFF dato_guardar,dato_guardar_leer
    
    RETURN
    
;************************************************************************************************************
;      Introducir datos para ser cargados en registro STATUS
;************************************************************************************************************
write_status_nrf:
    
    BCF LATB,3,ACCESS			    ; Ponemos a CSN en 0 para activarlo
    
    ;Enviamos la word mientras se recibe el estatus
    MOVF dato_estado, W, ACCESS		    ; Movemos dato_estado al registro W
    ADDLW b'00100000'			    ; Realizamos una ADD con el registro W, para escribir.
    MOVWF dato_estado, ACCESS		    ; Lo que esta en el registro W pasarlo a dato_estado
    
    MOVF dato_estado, W, ACCESS		    ; Movemos dato_estado al registro W
    ADDLW b'00000111'			    ; Realizamos una ADD con el registro W, para el registro de STATUS
    MOVWF dato_estado, ACCESS		    ; Lo que esta en el registro W pasarlo a dato_estado
    
    ;Enviar datos por words
    MOVFF dato_contenido,dato_a_enviar	    ;  dato_contenido a dato_a_enviar
    CALL SPI_transfer			    ; Realizamos transferencia hacia el NRF24L01
    
    BSF LATB,3,ACCESS			    ; CSN en 1 para desactivarlo
    RETURN
        
;----------------------------------Configuraci�n la transferencia de datos------------------------------------
SPI_transfer:
    ;Configuraci�n de la tranferencia de datos
    MOVFF dato_a_enviar,SSPBUF		    ; Pasamos los datos a tranferiar al Registro SSPBUF
    
Wait_transfer:
    ;Verificacion de comunicación
    MOVLW 0x00				    ;  0x00 al registro W
    CPFSEQ PIR1,0			    ; si PIR1 == 0
    BRA Wait_transfer			    ; Repetir hasta que la transferencia se complete
    
    ;Limpiamos bandera de la interrupci�n del SPI
    BCF PIR1,3,ACCESS			    ; Ponemos el Registro PIR1 y bajamos bandera
   
    ;Obtenemos los datos del registro y lo guardamos
    MOVFF SSPBUF,dato_guardar		    ; Pasamos los datos del Registro SSPBUF a la direcci�n de memoria dato_guardar
   
    RETURN
    
;----------------------------Subrutina para el cambio entre displays-----------------------------------;
retardoMinimo:
    MOVF retardo_modo_tx, W, ACCESS	    ; Movemos retardo_modo_tx al registro W
    MOVWF retardoMinimo_tx			    ; Pasamos lo que est� en el registro W a retardoMinimo_tx
Wait_retardoMinimo:
    DECFSZ retardoMinimo_tx,1,0		    ; Decrementamos hasta que retardoMinimo_tx sea igual a cero,entonces salta...
    BRA Wait_retardoMinimo			    ; Si retardoMinimo_tx no es cero entonces regresamos a Wait_retardoMinimo
    RETURN				    ; Retornar
   END

