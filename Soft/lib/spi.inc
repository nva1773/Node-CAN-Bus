; ***********************************************
; * spi.asm                                     *
; * Revision v1.0.0   December 2017             *
; * Developed using MPLAB V8.0					*
; *                                             *
; * This code implements SPI communications for *
; * the Microchip PIC12F675 8 pin micro         *
; * using a three wire interface                *
; *                                             *
; * Fosc 4MHz -> one SCK = 12mks                *
; * Baud rate SPI = 83,3 kBis: (1/12mks=83333)  *
; * Used SPI bus Mode 0,0                       *
; *                                             *
; ***********************************************
 
; ***********************************************
; * Define I/O pin assignments                  *
; ***********************************************
cs_pin          EQU     5
sdi_pin         EQU     4
sdo_pin         EQU     4
sck_pin         EQU     2

; ***********************************************
; * Variable definitions                        *
; ***********************************************
; spi_cnt, spi_data, spi_gpio

; ***********************************************
; * SPI_SEND                                    *
; * This routine sends a single byte out.  The  *
; * byte to be transmitted must be loaded into  *
; * the w register prior to calling this        *
; * routine.                                    *
; ***********************************************
spi_send                                ; w reg ->
        movwf   spi_data                ; save data byte
        bsf     STATUS,RP0              ; select bank1
        movlw   0x0B                    ; set GP4 to output 0x0B
        movwf   TRISIO					; -
        bcf     STATUS,RP0              ; select bank0
        movlw   8						; set bit count
        movwf   spi_cnt                 ; -
        movf	GPIO, 0					; save bits GPIO
        movwf	spi_gpio				; into spi_gpio
write_loop
		bcf		spi_gpio, sck_pin		; set sck low
		rlf     spi_data,1              ; move bit into carry flag
        btfsc   STATUS,C                ; test bit, if high
        goto    send_hi 				; goto set gpio
        bcf     spi_gpio, sdo_pin		; else clear gpio
        goto    wr_clock
send_hi
        bsf     spi_gpio, sdo_pin
        nop
wr_clock
		movf	spi_gpio, 0				; load data and clk into GPIO
		movwf	GPIO					; -
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		bsf     spi_gpio,sck_pin        ; set sck high
		movf	spi_gpio, 0				; load data and clk into GPIO
		movwf	GPIO					; -
		decfsz  spi_cnt,1         		; send all bits?
        goto    write_loop				; no, goto next
        nop								; yes, end send
        nop
        nop
        nop
        nop
        nop
        bcf		spi_gpio, sck_pin		; set sck low		
        bcf     spi_gpio, sdo_pin		; sel data low
        movf	spi_gpio, 0				;
        movwf	GPIO					; to GPIO
        retlw   0
    
; ***********************************************
; * SPI_RECEIVE                                 *
; * This routine receives a single byte.   The  *
; * received byte is in the w register upon     *
; * returning to the calling function.          *
; ***********************************************
spi_receive
        bsf     STATUS,RP0              ; select bank1
        movlw   0x1B                    ; set GP4 to input 0x1B
        movwf   TRISIO					; -
        bcf     STATUS,RP0              ; select bank0
        bcf     GPIO,sck_pin			; set sck low
        clrf    spi_data				; clear spi_data
        movlw   8                       ; set bit count
        movwf   spi_cnt					; -
rd_loop        
        rlf     spi_data,1              ; rotate data
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        bsf     GPIO,sck_pin            ; set sck high
        btfsc   GPIO,sdi_pin            ; check input pin
        goto    read_hi	                ; 
        bcf     spi_data,0
        goto    shift
read_hi
        bsf     spi_data,0
        nop
shift   
        nop
        nop
        nop
        nop
        nop
        nop
        bcf     GPIO,sck_pin			; set sck low
        decfsz  spi_cnt,1
        goto    rd_loop
        movf	spi_data, W
        return       
