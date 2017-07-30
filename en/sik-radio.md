# SiK Radio

The hardware for the SiK radio can be obtained from various stores.

### Vendors

* [3DR Radio](https://store.3drobotics.com/products/3dr-radio-set) \(small\)
* [HK Radio](http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=55559) \(small\)
* [RFD900u](http://rfdesign.com.au/products/rfd900u-modem/) \(small\)
* [RFD900](http://rfdesign.com.au/products/rfd900-modem/) \(long range\)

![](/assets/sik_radio.jpg)

## Build Instructions

The PX4 toolchain does not install the required 8051 compiler by default.

### Mac OS

Install the toolchain

```
brew install sdcc
```

build the image for the standard SiK Radio / 3DR Radio:

```sh
git clone https://github.com/LorenzMeier/SiK.git
cd SiK/Firmware
make install
```

upload it to the radio \(**change the serial port name**\):

```
tools/uploader.py --port /dev/tty.usbserial-CHANGETHIS dst/radio~hm_trp.ihx
```

## Configuration Instructions

The radio supports AT commands for configuration.

```
screen /dev/tty.usbserial-CHANGETHIS 57600 8N1
```

Then start command mode:

**DO NOT TYPE ANYTHING ONE SECOND BEFORE AND AFTER**

```
+++
```

List the current settings:

```
ATI5
```

Then set the net ID, write settings and reboot radio:

```
ATS3=55
AT
&
W
ATZ
```

**You might have to power-cycle the radio to connect it to the 2nd radio**

