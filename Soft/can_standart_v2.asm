; ***********************************************
; * can_standart.asm                            *
; * Release v2.0.0  April 2018                  *
; * Developed using MPLAB V8.0                  *
; *                                             *
; * This code implements SPI communications for *
; * the Microchip PIC12F675 8 pin micro         *
; * using a three wire interface                *
; * and a Microchip MCP2515 Stand Alone CAN     *
; * controller.                                 *
; *                                             *
; ***********************************************

; ***********************************************
; * Description release:                        *
; *                                             *
; * when WDT event - node wake up from WDT and  *
; *                  read pin TXRTS, choice and *
; *                  toggle pin RX1BF MCP2515,  *
; *                  every one second:          *
; *                  transmit value ADC0.       *
; *                                             *
; * when RTR msg   - node wake up form GPIO and *
; *                  transmit value ADC1.       *
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
; * Setup the PIC12F675 configuration Word      *
; ***********************************************

        __CONFIG   _CP_OFF & _WDT_ON & _MCLRE_OFF & _INTRC_OSC_NOCLKOUT

; ***********************************************
; * Constants definitions                       *
; ***********************************************
; Wake-up on pin change interrupt for GP3
int_pin     EQU     3

; ***********************************************
; * Variable definitions                        *
; ***********************************************
		CBLOCK 0x20
		spi_cnt, spi_data, spi_gpio
		can_cnt, can_data, can_addr, can_temp
		delay_cnt
		adc_delay, adc_data
		ENDC

; ***********************************************
; * PIC Initialization                          *
; ***********************************************
        org     0x00
        goto    start                   ; Jump around START vector
        
; ***********************************************
; * Start of Main Code                          *
; ***********************************************
start
        clrf	STATUS			;
        banksel	OSCCAL
        call	0x3FF			;
        movwf 	OSCCAL			; Load OSC for INTRC
		
	banksel	OPTION_REG
	movlw   0x88                    ; Disable internal pullups
        movwf   OPTION_REG              ; Prescaler = 1:1 for WDT - 18ms
         
        banksel	CMCON
        movlw	0x07			; Disable Comaparator
        movwf	CMCON			; -
        
        banksel	TRISIO
        movlw   0x0B                    ; 0x0B -- 001011
        movwf   TRISIO                  ; set all ports output, except GP3/1/0
        
        banksel	IOCB
        movlw	0x08			; Enable interrupt on change on GP3
        movwf	IOCB			;
         
        banksel	ANSEL
        movlw   0x53                    ; Fosc/16, GP4&2 = DIO, GP0&1= ADC
        movwf   ANSEL                  	; -
                                         
; Initialize the A/D converter
        banksel	ADCON0
        movlw   0x00                    ; ADMF = left, Vref = Vdd, AN0 conversion
        movwf   ADCON0                  ; Turn off A/D module

; Set up initial conditions for the SPI and MCP2515
        banksel	GPIO
        movlw   0x20                    ; CS high, INT high, data/clk low, leds off
        movwf   GPIO                    ; write to port
        
; Init counter
	clrf	delay_cnt
        
; Init MCP2515
        call    MCP2515_init            ; initialize the MCP2515

; GPIE set - interrupt on pin change
        clrf	STATUS		        ; select bank0
        movf	GPIO, W			; Read PORT
        movlw	0x08			; GIE cleared - global interrupts disabled
        movwf	INTCON			; GPIE enable, interrupt flags cleared
	clrwdt

; ***********************************************
; * Main cycle - sleep, wake up from WDT or ISR *
; ***********************************************
main
        sleep				; SLEEP
        nop                             ; 
        
        btfss	STATUS, NOT_TO		; if ~TO = 0 
        call	wakeup_wdt              ; then wake up from WDT
        
        btfsc   INTCON, GPIF		; if GPIF = 1
        call	wakeup_pin		; then wake up from GPIO

        goto    main                    ; go back to main

; ***********************************************************************
; * MCP2515 Initialization - 77 ms                                      *
; * Using standart indefication (11 bits)                               *
; * RXB0 - used RXF0 for RTR - 3FE - Read Analog Channel 1              *
; * RXB1 - not used, mask 0x7FF					                        *
; * TXB0 - 3FD - Analog Value Channel 0,(every one 10ms - from Timer0 ) *
; * TXB1 - 3FE - Analog Value Channel 1,( Remote Transmission Request )	*
; * TXB2 - 3FF - Can Error                                              *
; ***********************************************************************
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
        movlw   CANCTRL                 ; select CANCTRL register can_address
        call    spi_send                ; and send it
        movlw   REQOP_CONFIG            ; Request Config Mode
        call    spi_send                ; send data
        bsf     GPIO,cs_pin             ; raise CS to terminate operation
        
; Choice STATUS == CONFIG, max loop = 256
	clrf	can_cnt			; can_cnt = 256
loop_read_mode
	clrwdt
	decfsz	can_cnt, F		;
	goto	$+2			;
	goto	MCP2515_init		; error read mode -> new init cycle
	bcf     GPIO,cs_pin             ; lower CS to enable MCP2515
        movlw   CAN_READ                ; read command
        call    spi_send                ; send comand
        movlw   CANSTAT                 ; select CANSTAT register can_address
        call    spi_send                ; and send it
	call	spi_receive		; read chip status 
	bsf     GPIO,cs_pin             ; yes, raise CS to disable MCP2515
	andlw	REQOP			; mask mode bits
	xorlw	OPMODE_CONFIG		; choice from OPMODE_CONFIG
	btfss	STATUS, Z		; EQ?
	goto	loop_read_mode		; no, goto loop
					; yes, next reg init		
	movlw   0x71                    ; number of can_addresses to be written
        movwf   can_cnt                 ; load into byte counter 
        movlw	0x00			; register number
        movwf	can_temp		; = 0
        movlw   0x01			; data adderss
        movwf   can_addr		; = 1
seq_wr                                  ; sequential write loop
	clrwdt
	movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                ; send command
        movf    can_temp, W             ; register address
        call    spi_send                ; send can_address
        movlw   HIGH reg_init_tbl       ; get high byte of reg_int_tbl can_address
        movwf   PCLATH                  ; load into high byte of PC counter
        movfw   can_addr                ; write into jump table pointer (can_addr) 
        decf    can_addr,F              ;
        movf    can_addr,W              ;
        call    reg_init_tbl            ; fetch byte to be written
        call    spi_send                ; send it to MCP2515
	bsf     GPIO,cs_pin             ; disable MCP2515
        incf    can_addr,F              ; increment the jump table pointer
        incf    can_addr,F              ; twice to point to the next byte
        incf	can_temp, F		; next can reg
	decfsz  can_cnt,F               ; decrement the byte counter and test for zero
        goto    seq_wr                  ; not done so repeat

; Mode NORMAL
        movlw   CAN_WRITE               ; write command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   CANCTRL                 ; write to CANCTRL register
        call    spi_send                
        movlw   REQOP_NORMAL|OSM_ENABLE ; Normal and One Shot Mode
        call    spi_send
        bsf     GPIO,cs_pin             ; terminate operation
        
        return                          ; end init CAN

; *******************************************************************
; * Wake Up Service WDT                                             * 
; *******************************************************************
wakeup_wdt
; Counter for one second = 55 * 18ms = 990 ms
	incf	delay_cnt, F
	movlw	.55
	xorwf	delay_cnt, W
	btfss	STATUS, Z		; couner is finish
	goto	pin_mcp2515		; no, goto read pin MCP2515
	clrf	delay_cnt		; yes, clear counter and transmit ADC0

;------------------------------------------------------------------------------
; ADC conversion channel 0
        bcf     ADCON0,CHS0             ; select ADC channel 0
        call    adc_cnv                 ; go do the conversion

; write ADC to TXB0
        bcf     GPIO,cs_pin             ; enable MCP2515
        movlw   CAN_WRITE               ; send write command to MCP2515
        call    spi_send                ;
        movlw   TXB0D0                  ; set write address to TXB0D0
        call    spi_send                ;
        movfw   adc_data                ; write ADC conversion result
        call    spi_send                ;
        bsf     GPIO,cs_pin             ; terminate SPI operation;

        bcf     GPIO,cs_pin             ; enable MCP2515
        movlw   CAN_RTS_TXB0            ; Send RTS command for TXB0
        call    spi_send                
        bsf     GPIO,cs_pin		; disable MCP2515
        
        return                          ; end wake up service WDT
;------------------------------------------------------------------------------
; Read input and write output MCP2515
pin_mcp2515
        bcf     GPIO,cs_pin             ; enable MCP2515
        movlw   CAN_READ                ; send read command to MCP2515
        call    spi_send                ;
        movlw   TXRTSCTRL               ; set read address to TXRTSCTRL
        call    spi_send                ;
        call    spi_receive             ; read data
        bsf     GPIO,cs_pin		; disable MCP2515
        
        movwf	can_data		; save data
        rrf		can_data, F	; data >> 3
        rrf		can_data, F	; -
        rrf		can_data, F	; -
        movlw	0x01			; mask B0BFS bit
        andwf	can_data, F		; -
	movlw   0xFF		        ; assume that B0BFS is to be set
        btfss   can_data, 0             ; test the value received in message and if it is 0
        movlw   0x00                    ; load w register to reset bit in BFPCTRL register
	movwf	can_data		; save into can_data
		
	movlw   CAN_BIT_MODIFY          ; use bit modify command to 
        bcf     GPIO,cs_pin             ; set/reset the B1BFS bit of BFPCTRL register
        call    spi_send
        movlw   BFPCTRL
        call    spi_send
        movlw   B1BFS
        call    spi_send
        movfw	can_data
        call    spi_send
        bsf     GPIO,cs_pin		; disable MCP2515
        
        return                          ; end wake up service WDT
        
; *******************************************************************
; * Wake Up Service GPIO                                            * 
; *******************************************************************
wakeup_pin                              ; Message received interrupt
        clrwdt				; Clear WDT
        movf	GPIO, W			; Read PORT
        bcf     INTCON, GPIF            ; clear GPIF interrupt flag
        
        btfsc	GPIO, int_pin		; if int_pin = 1 ( active 0 )
	return                    	; return
      					; else		
; reading CANINTF 
		movlw    CAN_READ                
        bcf      GPIO,cs_pin            ; lower CS line
        call     spi_send               ; send read command to MCP2515
        movlw    CANINTF                ; the interrupt flag register (CANINTF)
        call     spi_send
        call     spi_receive            ; read the data from the MCP2515
        bsf      GPIO,cs_pin            ; terminate SPI read
        movwf    can_data               ; save CANINTF value

; test interrupt bits
        btfsc    can_data,RX0IF         ; test CANINTF for RX0IF
        call     msg_rcvd               ; if RX0IF set go process message
                                         
        btfsc    can_data,ERRIF         ; test CANINTF for ERRIF 
        call     can_err                ; if ERRIF set go process CAN error

; clear CANINTF
	    bcf     GPIO,cs_pin    	; enable MCP2515
        movlw   CAN_WRITE               ; send write command to MCP2515
        call    spi_send                ;
        movlw   CANINTF                 ; set write address to CANINTF
        call    spi_send                ;
        movlw   0x00		        ; 
        call    spi_send                ;
        bsf     GPIO,cs_pin             ; terminate SPI operation	
       
        return
   
; *******************************************************************
; * Anadlog to Digital Conversion Routine                           * 
; * This routine initiates the A/D conversion.  The ADC channel     *
; * select bits (CHS1:0) have to be set prior to this routine being *
; * called.  The routine waits for the conversion to complete       *
; * before returning to the calling function.                       *
; *******************************************************************
adc_cnv
	bsf	ADCON0,ADON		; ADC On
	movlw	.10			; delay for charge capacity ADC
	movwf	adc_delay		; -
	decfsz	adc_delay, F		; -
	goto	$-1			; -
        bsf     ADCON0,GO		; ADC Start
adc_busy
        btfsc   ADCON0,GO_DONE          ; wait for ADC to complete
        goto    adc_busy
        movf	ADRESH, W		; store ADC value
	movwf	adc_data		; -
        bcf	ADCON0,ADON		; ADC Off
        return

; *******************************************************************
; * CAN Msg Received Routine - Remote Frame (RxRTR <3> = 1)         * 
; * This routine is called when a message has been received into    *
; * TXB0 of the MCP2515.  This routine reads the filter bits to     *
; * determine the type of message received and then initiates the   *
; * appropriate response.                                           *
; *******************************************************************
msg_rcvd
        movlw   CAN_READ                ; SPI read command
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                
        movlw   RXB0CTRL                ; Read buffer 0 control register
        call    spi_send
        call    spi_receive
        bsf     GPIO,cs_pin             ; terminate function
        andlw   B'00001001'             ; mask off all but the RXRTR and FILHIT bit
        movwf   can_temp                ; store value in temp

        movlw   0x08                    ;
        subwf   can_temp,W
        btfsc   STATUS,Z                ; filter 0 match?
        goto    filter0
        
        movlw   0x09
        subwf   can_temp,W
        btfsc   STATUS,Z                ; filter 1 match?
        goto    filter1
        
        goto	filter_done

filter0
        bsf     ADCON0,CHS0             ; select ADC channel 1
        call    adc_cnv                 ; go do the conversion

        bcf     GPIO,cs_pin             ; enable MCP2515
        movlw   CAN_WRITE               ; send write command to MCP2515
        call    spi_send                ;
        movlw   TXB1D0                  ; set write address to TXB1D0
        call    spi_send                ;
        movfw   adc_data                ; write ADC conversion result
        call    spi_send                ; 
        bsf     GPIO,cs_pin             ; terminate SPI operation
        
        movlw   CAN_RTS_TXB1            ; last step is to send the 
        bcf     GPIO,cs_pin             ; request to send command for 
        call    spi_send                ; transmit buffer zero
        bsf     GPIO,cs_pin
        
        goto    filter_done

filter1
        ; not used
        goto    filter_done

filter_done
        return

; *******************************************************************
; * CAN Error routine ID = 3FF                                      * 
; * This routine reads the value of the MCP2515 Error flag (EFLG)   *
; * register, writes it to byte 0 of TXB2, and then transmits the   *
; * TXB2 message                                                    * 
; *******************************************************************
can_err
	movlw   CAN_READ                ; SPI Read operation
        bcf     GPIO,cs_pin             ; enable MCP2515
        call    spi_send                ; 
        movlw   EFLG                    ; EFLG register to be read
        call    spi_send                ;
        call    spi_receive             ; read the data
        bsf     GPIO,cs_pin             ; terminate SPI operation
        movwf   can_temp                ; save the value of EFLG register

        movlw   CAN_WRITE               ; now write to MCP2515
        bcf     GPIO,cs_pin             ;
        call    spi_send                ;
        movlw   TXB2D0                  ; write to data byte 0 of TXB2
        call    spi_send                ; 
        movfw   can_temp                ; write EFLG register contents
        call    spi_send                ;
        bsf     GPIO,cs_pin             ; terminate SPI operation

        movlw   CAN_RTS_TXB2            ; send request to send 
        bcf     GPIO,cs_pin             ; for transmit buffer 2
        call    spi_send
        bsf     GPIO,cs_pin
        
        return
		
; **************************************************
; * Include the custom three wire SPI support file *
; **************************************************

		#include "spi.inc"              ; SPI routines

; *******************************************************************
; * MCP2515 register initialization table                           * 
; * Store at the end of ROM memory                                  *
; * Note that all can_addresses are initialized to simplify the     *
; * initialization code.                                            * 
; *******************************************************************
 
        org     0x0300                  ; Initialization table can_address
reg_init_tbl
        addwf   PCL, F                  ; Register  Addr
                                        ; --------- ----
        retlw   0x7F                    ; RXF0SIDH  0x00    Filter 0 matches 0x3FE - Read Analog 1
        retlw   0xC0                    ; RXF0SIDL  0x01
        retlw   0xff                    ; RXF0EID8  0x02
        retlw   0xff                    ; RXF0EID0  0x03
        retlw   0xff                    ; RXF1SIDH  0x04    Filter 1 matches
        retlw   0xff                    ; RXF1SIDL  0x05
        retlw   0xff                    ; RXF1EID8  0x06
        retlw   0xff                    ; RXF1EID0  0x07
        retlw   0xff                    ; RXF2SIDH  0x08    Filter 2 matches
        retlw   0xff                    ; RXF2SIDL  0x09
        retlw   0xff                    ; RXF2EID8  0x0A
        retlw   0xff                    ; RXF2EID0  0x0B
        retlw   0x3C                    ; BFPCTRL   0x0C    BFP pins as digital outputs, initial state hi
        retlw   0x00                    ; TXRTSCTRL 0x0D    TXRTS pins as digital inputs
        retlw   0x80                    ; CANSTAT   0x0E
        retlw   0x80                    ; CANCTRL   0x0F
        
        retlw   0xff                    ; RXF3SIDH  0x10    Filter 3 matches
        retlw   0xff                    ; RXF3SIDL  0x11
        retlw   0xff                    ; RXF3EID8  0x12
        retlw   0xff                    ; RXF3EID0  0x13
        retlw   0xff                    ; RXF4SIDH  0x14    Filter 4 matches
        retlw   0xff                    ; RXF4SIDL  0x15
        retlw   0xff                    ; RXF4EID8  0x16
        retlw   0xff                    ; RXF4EID0  0x17
        retlw   0xff                    ; RXF5SIDH  0x18    Filter 5 matches
        retlw   0xff                    ; RXF5SIDL  0x19
        retlw   0xff                    ; RXF5EID8  0x1A
        retlw   0xff                    ; RXF5EID0  0x1B
        retlw   0x00                    ; TEC       0x1C
        retlw   0x00                    ; REC       0x1D
        retlw   0x80                    ; CANSTAT   0x1E
        retlw   0x80                    ; CANCTRL   0x1F
        
        retlw   0x7F                    ; RXM0SIDH  0x20    Set RXM0 to match msg ID's of 0x000 to 0x3FF
        retlw   0xE0                    ; RXM0SIDL  0x21     
        retlw   0x00                    ; RXM0EID8  0x22
        retlw   0x00                    ; RXM0EID0  0x23
        retlw   0xFF                    ; RXM1SIDH  0x24    Set all mask bits so that no msg's
        retlw   0xFF                    ; RXM1SIDL  0x25	are received into RXB1
        retlw   0xFF                    ; RXM1EID8  0x26
        retlw   0xFF                    ; RXM1EID0  0x27
        retlw   0x02                    ; CNF3      0x28    PHSEG2 = 3TQ
        retlw   0x90                    ; CNF2      0x29    PHSEG1 = 3TQ, PRSEG = 1TQ 
        retlw   0x03                    ; CNF1      0x2A    SJW = 1TQ, BRP set to 4
        retlw   0x21                    ; CANINTE   0x2B    MERRIE and RX0IE enabled = 0x21
        retlw   0x00                    ; CANINTF   0x2C
        retlw   0x00                    ; EFLG      0x2D
        retlw   0x80                    ; CANSTAT   0x2E
        retlw   0x80                    ; CANCTRL   0x2F
        
        retlw   0x03                    ; TXB0CTRL  0x30    Highest priority
        retlw   0x7F                    ; TXB0SIDH  0x31    3FD - Analog Channel 0
        retlw   0xA0                    ; TXB0SIDL  0x32
        retlw   0x00                    ; TXB0EID8  0x33
        retlw   0x00                    ; TXB0EID0  0x34
        retlw   0x01                    ; TXB0DLC   0x35
        retlw   0x00                    ; TXB0DB0   0x36
        retlw   0x00                    ; TXB0DB1   0x37
        retlw   0x00                    ; TXB0DB2   0x38
        retlw   0x00                    ; TXB0DB3   0x39
        retlw   0x00                    ; TXB0DB4   0x3A
        retlw   0x00                    ; TXB0DB5   0x3B
        retlw   0x00                    ; TXB0DB6   0x3C
        retlw   0x00                    ; TXB0DB7   0x3D
        retlw   0x80                    ; CANSTAT   0x3E
        retlw   0x80                    ; CANCTRL   0x3F
           
        retlw   0x03                    ; TXB1CTRL  0x40    Highest priority
        retlw   0x7F                    ; TXB1SIDH  0x41    3FE - Analog Channel 1
        retlw   0xC0                    ; TXB1SIDL  0x42    -
        retlw   0x00                    ; TXB1EID8  0x43
        retlw   0x00                    ; TXB1EID0  0x44
        retlw   0x01                    ; TXB1DLC   0x45
        retlw   0x00                    ; TXB1DB0   0x46
        retlw   0x00                    ; TXB1DB1   0x47
        retlw   0x00                    ; TXB1DB2   0x48
        retlw   0x00                    ; TXB1DB3   0x49
        retlw   0x00                    ; TXB1DB4   0x4A
        retlw   0x00                    ; TXB1DB5   0x4B
        retlw   0x00                    ; TXB1DB6   0x4C
        retlw   0x00                    ; TXB1DB7   0x4D
        retlw   0x80                    ; CANSTAT   0x4E
        retlw   0x80                    ; CANCTRL   0x4F
        
        retlw   0x03                    ; TXB2CTRL  0x50    Highest priority
        retlw   0x7F                    ; TXB2SIDH  0x51    3FF - CAN Error
        retlw   0xE0                    ; TXB2SIDL  0x52    -
        retlw   0x00                    ; TXB2EID8  0x53
        retlw   0x00                    ; TXB2EID0  0x54
        retlw   0x01                    ; TXB2DLC   0x55
        retlw   0x00                    ; TXB2DB0   0x56
        retlw   0x00                    ; TXB2DB1   0x57
        retlw   0x00                    ; TXB2DB2   0x58
        retlw   0x00                    ; TXB2DB3   0x59
        retlw   0x00                    ; TXB2DB4   0x5A
        retlw   0x00                    ; TXB2DB5   0x5B
        retlw   0x00                    ; TXB2DB6   0x5C
        retlw   0x00                    ; TXB2DB7   0x5D
        retlw   0x80                    ; CANSTAT   0x5E
        retlw   0x80                    ; CANCTRL   0x5F
        
        retlw   0x20                    ; RXB0CTRL  0x60    Receive only Standard Id's that match Masks/Filters
        retlw   0x00                    ; RXB0SIDH  0x61
        retlw   0x00                    ; RXB0SIDL  0x62
        retlw   0x00                    ; RXB0EID8  0x63
        retlw   0x00                    ; RXB0EID0  0x64
        retlw   0x00                    ; RXB0DLC   0x65
        retlw   0x00                    ; RXB0DB0   0x66
        retlw   0x00                    ; RXB0DB1   0x67
        retlw   0x00                    ; RXB0DB2   0x68
        retlw   0x00                    ; RXB0DB3   0x69
        retlw   0x00                    ; RXB0DB4   0x6A
        retlw   0x00                    ; RXB0DB5   0x6B
        retlw   0x00                    ; RXB0DB6   0x6C
        retlw   0x00                    ; RXB0DB7   0x6D
        retlw   0x80                    ; CANSTAT   0x6E
        retlw   0x80                    ; CANCTRL   0x6F
        
        retlw   0x20                    ; RXB1CTRL  0x70    Receive only Standard Id's that match Masks/Filters  

;--------------------------------------------------------------------
		END                             
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
