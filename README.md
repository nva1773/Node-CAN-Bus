<h2 align="center">Узел CAN шины на PIC12F675.</h2>

***

За основу был взят проект от Microchip: 
[AN215 A Simple CAN Node](http://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en011856)

***
### Папки и файлы:

    Hard       Электронная схема узла, а также файлы для печати в формате pdf и jpg.
    Soft       Исходные файлы проекта.
    README.md  Этот файл. 
   
***
### Цель проекта:

Изучение протокола обмена по CAN шине, изучение CAN контроллера MCP2515, а так же научится работать с CAN анализатором. 

***
### Описание:

В устройтсве используется модуль от **NiRen** собраный на Microchip MCP2515 CAN контроллер с TJA1050 CAN трансивером:

![NiRen circuit](https://github.com/nva1773/can-node/blob/master/Hard/MCP2515%20NiRen%20circuit.jpg)
![NiRen board](https://github.com/nva1773/can-node/blob/master/Hard/MCP2515%20NiRen%20board.png)

Для управления модулем был выбран микроконтроллер Microchip **PIC12F675**:

![CAN node circuit](https://github.com/nva1773/can-node/blob/master/Hard/can%20node%20circuit.jpg)

Программа написанна на ассемблере в среде **MPLAB IDE** и скомпелированна с помощью **MPLAB ASM**.

***
### CAN Standart Indefication

Инициализируем CAN контроллер MCP2515 следующим образом:

```assembly
; ***********************************************************************
; * Using standart indefication (11 bits)                               *
; * RXB0 - 3FE - used RXF0 for RTR - Read Analog Channel 1              *
; * RXB1 - not used - mask 0x7FF                                        *
; * TXB0 - 3FD - Transmit Analog Value Channel 0                        *
; * TXB1 - 3FE - Transmit Analog Value Channel 1                      	*
; * TXB2 - 3FF - Can Error                                              *
; ***********************************************************************
```

Версия V1.0.0

В этой версии сторожевой таймер отключен, поэтому в оновном цикле контроллер ожидает события от прерывания. 
Передача данных от узла происходит:

* каждые 10мс, пo прерыванию от Timer0, значения ADC0;
* по прерыванию от MCP2515, значение ADC1.

```assembly
//Assembly code 
; ***********************************************
; * Setup the PIC12C675 configuration Word      *
; ***********************************************

        __CONFIG   _CP_OFF & _WDT_OFF & _MCLRE_OFF & _INTRC_OSC_NOCLKOUT

; ***********************************************
; * Description release:                        *
; *                                             *
; * every one 10ms - node transmit value ADC0,  *
; *                  read pin TXRTS, choice and *
; *                  toggle pin RX1BF MCP2515.  *
; * when RTR msg   - node transmit value ADC1.  *
; *                                             *
; ***********************************************

; *******************************************
; * Main wait loop                          *
; *******************************************
main                                    ; wait for interrupt to occur
        nop                             ; while not processing a message
        goto    main                    ; go back to main

```

Версия V2.0.0

В этой версии сторожевой таймер включен, поэтому в оновном цикле контроллер ложится в сон, что значително уменьшае потребление энергии. 
Передача данных от узла происходит:

* каждые 990мс передаем значения ADC0;
* по прерыванию от MCP2515, значение ADC1.

```assembly
//Assembly code 
; ***********************************************
; * Setup the PIC12C675 configuration Word      *
; ***********************************************

        __CONFIG   _CP_OFF & _WDT_ON & _MCLRE_OFF & _INTRC_OSC_NOCLKOUT

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
; * Main cycle - sleep, wake up from WDT or ISR *
; ***********************************************
main
        sleep                           ; SLEEP
        nop                             ; 
        
        btfss	STATUS, NOT_TO          ; if ~TO = 0 
        call	wakeup_wdt              ; then wake up from WDT, every one 18ms
        
        btfsc   INTCON, GPIF            ; if GPIF = 1
        call	wakeup_pin              ; then wake up from GPIO, change GP3

        goto    main                    ; go back to main

```
