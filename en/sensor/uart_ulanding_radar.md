# uLanding Radar

The uLanding radar is a product from [Aerotenna](http://aerotenna.com/sensors/) and can be used to measure distance to an object.


## Enable the driver for your hardware
Currently, this radar device is supported by any hardware which runs the OS NuttX and which can offer a serial port for the
interface. Since flash space is small on some hardware you may have to enable building the driver for your target yourself.
To do so add the following line to the cmake config file which corresponds to the target you want to build for:
```
drivers/ulanding
```

All config files are located [here.](https://github.com/PX4/Firmware/tree/master/cmake/configs)

## Start the driver
You will have to tell the sytem to start the driver for the radar during sytem startup.
You can simply add the following line to an [extras.txt](../advanced/system_startup.md) file located on your SD card.
```
ulanding_radar start /dev/serial_port
```

In the upper command you will have to replace the last argument with the serial port you have connected the hardware to.
If you don't specify any port the driver will use /dev/ttyS2 which is the TELEM2 port on Pixhawk.

**Warning**

If you are connecting the radar device to TELEM2 then make sure to set the parameter SYS_COMPANION to 0. Otherwise the serial port
will be used by a another application and you will get unexpected behaviour.
