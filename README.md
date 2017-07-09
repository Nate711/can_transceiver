# Teensy 3.5 robot controller for 2 VESCs
Based on [Project Webpage](http://rummanwaqar.com/can-bus-with-teensy-3-1/)

Schematic ![sch](https://raw.githubusercontent.com/rummanwaqar/teensy_can/master/schematic.png)
Using [FlexCAN Library](https://github.com/teachop/FlexCAN_Library)

**[the schematic has the wrong pinout on the CAN transceiver]**

**can_transmitter** - sends two 8-bit characters every 500ms
**can_receiver** - reads data on CAN bus and displays it on Serial
