---
translated_page: https://github.com/PX4/Devguide/blob/master/en/advanced/parrot_bebop.md
translated_sha: 999dd32122118cdd75d02831ff71787a23b2ef84
---

# Bebop 2 - Advanced

## FTDI connection
Follow the instructions to connect to the Parrot Bebop 2 via FTDI.
* Loosen the two Torx screws (T5) to take off the front cap.
  ![](../../assets/hardware/bebop_torx.JPG)
* Use pins to connect to ground/RX/TX or solder cables onto the connectors.
  ![](../../assets/hardware/bebop_serial.JPG)
* Connect the FTDI cable and run
```sh
screen /dev/ttyUSB0 115200
```
to connect to the Bebop.
![](../../assets/hardware/bebop_ftdi.JPG)
