<h2 align="center">Узел CAN шины на PIC12F675.</h2>

***

За основу был взят проект от Microchip: 
[AN215 A Simple CAN Node](http://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en011856)

***
### Папки и файлы:

    Hard       Электронная схема узла, а также файлы для печати в формате pdf и jpg.
    Soft       Исходные файлы проекта.
    README.md  Этот файл. 
    
### Цель проекта:

Изучение протокола обмена по CAN шине, изучение CAN контроллера MCP2515, а так же научится работать с CAN анализатором. 

### Описание:

В устройтсве используется модуль от **NiRen** собраный на Microchip MCP2515 CAN контроллер с TJA1050 CAN трансивером:

![NiRen circuit](https://github.com/nva1773/can-node/blob/master/Hard/MCP2515%20NiRen%20circuit.jpg)
![NiRen board](https://github.com/nva1773/can-node/blob/master/Hard/MCP2515%20NiRen%20board.png)

Для управления модулем был выбран микроконтроллер Microchip **PIC12F675**:

![CAN node circuit](https://github.com/nva1773/can-node/blob/master/Hard/can%20node%20circuit.jpg)

Программа написанна на ассемблере в среде **MPLAB IDE** и скомпелированна с помощью **MPLAB ASM**.
