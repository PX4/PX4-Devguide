# Bebop 2 - Advanced

## FTDI connection
Follow the instructions to connect to the Parrot Bebop 2 via FTDI.
* Loosen the two Torx screws (T5) to take off the front cap.
![](images/hardware/bebop_torx.JPG)
* Use pins to connect to ground/RX/TX or solder cables onto the connectors.
![](images/hardware/bebop_serial.JPG)
* Connect the FTDI cable and run
```sh
screen /dev/ttyUSB0 115200
```
to connect to the Bebop.
![](images/hardware/bebop_ftdi.JPG)
