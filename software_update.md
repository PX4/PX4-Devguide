# Software Update

The method to update the PX4 software on the drone depends on the hardware platform. For microcontroller based applications new Firmware is flashed through USB or serial.


## Infrastructure

### Flashing the Bootloader

```bash
arm-none-eabi-gdb
  (gdb) tar ext /dev/serial/by-id/usb-Black_Sphere_XXX-if00
  (gdb) mon swdp_scan
  (gdb) attach 1
  (gdb) load tapv1_bl.elf
        ...
        Transfer rate: 17 KB/sec, 828 bytes/write.
  (gdb) kill
```
