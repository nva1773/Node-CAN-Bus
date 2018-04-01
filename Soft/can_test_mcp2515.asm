; ***********************************************
; * cannode.asm                                 *
; * Revision 1.0   December 2017                *
; * Developed using MPLAB V8.0					*
; *                                             *
; * This code implements SPI communications for *
; * the Microchip PIC12F675 8 pin micro         *
; * using a three wire interface				*
; * and a Microchip MCP2515 Stand Alone CAN     *
; * controller.                                 *
; *                                             *
; ***********************************************

; ***********************************************
; * Setup the MPASM assembler options           *
; ***********************************************

        LIST    p=12f675


; ***********************************************
; * Include the standard PIC12F675 include file *
; * and the custom MCP2515 support files        *
; ***********************************************

		#include <p12f675.inc>                  
		#include "mcp2515.inc"                  
		errorlevel -302

; ***********************************************
; * Setup the PIC12C672 configuration Word      *
; ***********************************************

        __CONFIG   _CP_OFF & _WDT_OFF & _MCLRE_OFF & _INTRC_OSC_NOCLKOUT

; ***********************************************
; * Constants definitions                       *
; ***********************************************
; Timer0 prescaler 256 => 38 * 256 * 1us = 9.728ms
; 255 - 38 = 217 = 0xD9
; Timer Reload value:
TMR_COUNT   EQU     0xD9

; Wake-up on pin change interrupt for GP3
int_pin         EQU     3

; Leds pins
led_1			EQU		0
led_2			EQU		1

; ***********************************************
; * Variable definitions                        *
; ***********************************************
		CBLOCK 0x20
		spi_cnt, spi_data, spi_gpio
		byte_cnt
		save_w, save_s
		ENDC

; ***********************************************
; * PIC Initialization                          *
; ***********************************************
        org     0x00
        goto    start                   ; Jump around ISR vector
        
; ***********************************************
; * Interrupt service vector initialization     *
; ***********************************************
        org     0x04			
        goto    isr			            ; Point ISR vector to the ISR handler

; ***********************************************
; * Start of Main Code                          *
; ***********************************************
start
        clrf	STATUS					;
        banksel	OSCCAL
        call	0x3FF					;
        movwf 	OSCCAL					; Load OSC for INTRC
		
		banksel	OPTION_REG
		movlw   0x87                    ; Disable internal pullups
                                        ; Interrupt on negative going edge on GP2
                                        ; Prescaler = 1:256
        movwf   OPTION_REG              ; Load the OPTION register
        
        banksel	CMCON
        movlw	0x07					; Disable Comaparator
        movwf	CMCON					; -
        
        banksel	TRISIO
        movlw   0x08                    ; 0x0B -- 001011
        movwf   TRISIO                  ; set all ports output, except GP3/1/0
        
        banksel	IOCB
        movlw	0x08					; Enable interrupt on change on GP3
        movwf	IOCB					;
         
        banksel	ANSEL
        ;movlw   0x13                    ; Fosc/8, GP4&2 = DIO, GP0&1= ADC
        movlw	0x00					; all pins digital
        movwf   ANSEL                  	; -
                                         
; Initialize the A/D converter
        banksel	ADCON0
        movlw   0x00                    ; ADMF = left, Vref = Vdd, AN0 conversion
        movwf   ADCON0                  ; Turn off A/D module

; Set up initial conditions for the SPI  and MCP2515
        banksel	GPIO
        movlw   0x20                    ; CS high, INT high, data/clk low, leds off
        movwf   GPIO                    ; write to port         
  
        call    MCP2515_init            ; initialize the MCP2515

; Initialize Timer0 = 10ms
		banksel	TMR0
        movlw   TMR_COUNT               ; Initialize Timer0
        movwf   TMR0                    ; Timer0 interrupt every 9.995mS
        
; GPIE set - interrupt on pin change
        clrf	STATUS		            ; select bank0
        movf	GPIO, W					; Read PORT
        movlw	0x28					; GIE cleared - global interrupts disabled
        movwf	INTCON					; TRM0 and GPIE enable, interrupt flags cleared 
        bsf     INTCON,GIE              ; Enable Interrupts

; *******************************************
; * Main wait loop                          *
; *******************************************
wait                                    ; wait for interrupt to occur
        nop                             ; while not processing a message
        goto    wait                    ; go back to wait

; ***********************************************
; * MCP2515 Initialization - 76 ms              *
; * Using standart indefication (11 bits)       *
; * RXB0 - not used, mask 0x7FF					*
; * RXB1 - used RXF2..RXF5 for command 3F0..3F3 *
; *        3F0 - Read Analog Channel 1          *
; *        3F1 - Read Digital Input TX0RTS,1,2  *
; *        3F2 - Change Digital Output RX0BF    *
; *        3F3 - Change Digital Output RX1BF    *
; * TXB0 - message valide 3F8..3FB              *
; *        3F8 - Analog Value Channel 1         *
; *        3F9 - Current Value Digital Inputs   *
; *        3FA - Acknowledgement for 3F2        *
; *        3FB - Acknowledgement for 3F3        *
; * TXB1 - 3FC - Analog Value Channel 0, (10ms) *
; * TXB2 - 3FE - Can Error                      *
; *        3FF - System Error                   *
; ***********************************************
MCP2515_init
; Reset MCP2515
        movlw   CAN_RESET               ; reset command
        bcf     GPIO,cs_pin             ; lower CS to enable MCP2515
        call    spi_send                ; send comand
        bsf     GPIO,cs_pin             ; raise CS to terminate operation
                      
; Mode CONFIG                                    
        movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; lower CS to enable MCP2515
        call    spi_send                ; send comand
        movlw   CANCTRL                 ; select CANCTRL register address
        call    spi_send                ; and send it
        movlw   REQOP_CONFIG            ; Request Config Mode
        call    spi_send                ; send data
        bsf     GPIO,cs_pin             ; raise CS to terminate operation
        
; Choice mode CONFIG, max loop = 256
		clrf	byte_cnt				; byte_cnt = 256
choice_mode
		bsf		GPIO,led_1				; Led_1 ON
		decfsz	byte_cnt, F				;
		goto	$+2						;
		goto	error_init				;
		bcf     GPIO,cs_pin             ; lower CS to enable MCP2515
        movlw   CAN_READ                ; read command
        call    spi_send                ; send comand
        movlw   CANSTAT                 ; select CANSTAT register address
        call    spi_send                ; and send it
		call	spi_receive				; read chip status 
		bsf     GPIO,cs_pin             ; yes, raise CS to disable MCP2515
		andlw	REQOP					; mask mode bits
		xorlw	OPMODE_CONFIG			; choice from OPMODE_CONFIG
		btfss	STATUS, Z				; EQ?
		goto	choice_mode				; no, goto loop
		goto	next_init				; yes, goto next initialization
		
error_init
		bcf		GPIO,led_1				; LED_1 OFF
		clrf	byte_cnt				; byte_cnt = 256
		goto	choice_mode				; proba next

next_init
; BFPCTRL
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXRTSCTRL               ; write to BFPCTRL register
        call    spi_send                
        movlw   0x0C					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation
        
; TXRTSCTRL
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXRTSCTRL               ; write to TXRTSCTRL register
        call    spi_send                
        movlw   0x00					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation
        
; CNF1
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   CNF1               		; write to CNF1 register
        call    spi_send                
        movlw   0x03					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation
        
; CNF2
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   CNF2               		; write to CNF2 register
        call    spi_send                
        movlw   0x90					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation
 
; CNF3
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   CNF3               		; write to CNF3 register
        call    spi_send                
        movlw   0x02					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation
 
; CANINTE
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   CANINTE               	; write to CANINTE register
        call    spi_send                
        movlw   0x00					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation

; TXB0CTRL
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXB0CTRL               	; write to TXB0CTRL register
        call    spi_send                
        movlw   0x03					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation      
        
; TXB0SIDH
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXB0SIDH               	; write to TXB0SIDH register
        call    spi_send                
        movlw   0x7F					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation
        
; TXB0SIDL
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXB0SIDL               	; write to TXB0SIDL register
        call    spi_send                
        movlw   0x80					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation

; TXB0SIDH
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXB0SIDH               	; write to TXB0SIDH register
        call    spi_send                
        movlw   0x7F					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation

; TXB0DLC
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXB0DLC               	; write to TXB0DLC register
        call    spi_send                
        movlw   0x01					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation

; TXB0DB0
		movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   TXB1D0               	; write to TXB1D0 register
        call    spi_send                
        movlw   0x55					; 
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation

; Mode NORMAL
        movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   CANCTRL                 ; write to CANCTRL register
        call    spi_send                
        movlw   REQOP_NORMAL|OSM_ENABLE ; Normal and One Shot Mode
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation

		bsf		GPIO,led_2				; Led_2 ON
        return
       
; *******************************************************************
; * Interrupt Service Routine                                       * 
; * The ISR determines whether a TMR0 interrupt or an external INT  *
; * pin interrupt occurs and then proceeds accordingly              *
; *******************************************************************
isr                                     
        bcf     STATUS,RP1              ; select bank 0/1

        btfss   INTCON,T0IE             ; Timer0 interrupt?
        goto    isr_pin                 ; No, so jump to external interrupt pin ISR

isr_tmr0
        movlw   TMR_COUNT               ; reload
        movwf   TMR0                    ; Timer0 

		movlw   CAN_RTS_TXB0            ; Send RTS command for TXB0
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        bsf     GPIO,cs_pin             ; terminate operation

        bcf     INTCON, T0IF            ; clear TMR0 interrupt flag
        retfie                          ; return to main routine

isr_pin                                  ; Message received interrupt
       movf		GPIO, W					; Read PORT
       bcf      INTCON,GPIF             ; reset interrupt flag                     
      
       retfie							; exit isr and re-enable interrupts


; **************************************************
; * Include the custom three wire SPI support file *
; **************************************************

		#include "spi.inc"              ; SPI routines  

;--------------------------------------------------------------------
		END                             
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
