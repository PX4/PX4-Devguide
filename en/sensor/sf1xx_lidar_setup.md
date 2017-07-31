# Lightware SF1XX lidar setup
----------------------------------------------------

This page shows you how to set up one of following lidars:
 1. SF10/a
 2. SF10/b
 3. SF10/c
 4. SF11/c

Driver supports only i2c connection.
![](../../assets/hardware/sf1xx_i2c.jpg)

## Configuring lidar
--------------------------------------------------------

To check the i2c address of the Lidar, connect to the sensor to via usb (it has an internal serial to usb converter). Open up the device with a terminal emulator (such as minicom). You should see a stream of measurements in your terminal once you successfully connect. Press `space` to bring up the little menu. Look through that to find the I2C bus address. If it is not '0x66', change it to '0x66'.
Newer sensor versions already have `0x66` preconfigured. Older have `0x55` which conflicts with `rgbled` module.

Note: it is insufficient to simply hook up the lidar to the i2c port; you also need to provide external power via the microusb connection. So the final wiring should include the four wires connected to i2c, as well as a microusb cable providing external power. 

## Configuring PX4
--------------------------------------------------------

Use the `SENS_EN_SF1XX` parameter to select the lidar model and then reboot.
* `0` lidar disabled
* `1` SF10/a
* `2` SF10/b
* `3` SF10/c
* `4` SF11/c

For PX4v2 you also need to add it to the make file:
open up /cmake/config/nuttx_px4fmu-v2_default.cmake

You should see a list of drivers.
Make sure /drivers/sf1xx is not commented out (no # sign in from of that line)

If you are using a pixhawk2, you will also have to edit the driver. 
Navigate to /src/drivers/sf1xx/
Open up sf1xx.cpp
Find the line that says #define SF1XX_BUS  PX4_I2C_BUS_EXPANSION
Change PX4_I2C_BUS_EXPANSION to PX4_I2C_BUS_ONBOARD



