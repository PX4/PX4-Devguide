# SiK Radio

[SiK radio](https://github.com/LorenzMeier/SiK) is a collection of firmware and tools for telemetry radios.

Information about *using* SiK Radio can be found it the *PX4 User Guide*: [Telemetry > SiK Radio](http://docs.px4.io/en/telemetry/sik_radio.html)

The ("developer") information below explains how to build SiK firmware from source and configure it using AT commands.


## Caveat

As of the time of this writing (25 FEB 2020), the SiK repository includes bootloaders and firmware for the following telemetry radios:

- HopeRF HM-TRP
- HopeRF RF50-DEMO
- RFD900
- RFD900a
- RFD900p
- RFD900pe
- RFD900u
- RFD900ue

*SiK does not currently support RFD900x or RFD900ux telemetry radios*. In order to update firmware on these radios (for instance, in order to support MAVLink v2.0), the following process is suggested:

1. Download the appropriate firmware from the [RFDesign website](https://files.rfdesign.com.au/firmware/).
2. On a Windows PC, download and install [RFD Modem Tools](https://files.rfdesign.com.au/tools/).
3. Use the RFD Modem Tools GUI to upload the firmware to your RFD900x or RFD900ux telemetry radio.


## Build Instructions (Mac OS X)

### Prerequisites

In order to build the command line tools included in the SiK repository, it is necessary to install the prerequisite dependencies. 

The dependency tree is summarized here for reference:

- [SiK](https://github.com/ArduPilot/SiK)
  - Mac OS X* or Linux based system
  - [SDCC](https://sourceforge.net/projects/sdcc/files/sdcc/3.5.0/)
  - [EC2](https://github.com/tridge/ec2)
    - Python 2.x
    - autoconf
    - boost
    - readline
    - libusb
    - gtk+
  - [Mono](https://www.mono-project.com/)

* Mac OS X users must also install *XCode Command Line Tools*, and the [*Homebrew*](https://brew.sh/) package manager.  While it is theoretically possible to use MacPorts to download some of the required packages, they will be stored in a non-default directory (typically `/opt/local/bin`), which complicates the SiK build process. Conversely, Homebrew stores packages in a directory (typically `/usr/local/opt`) that is in the default search path of the SiK build scripts.

### Installation

1. Many of the make files associated with SiK use a Python 2.x syntax; accordingly it is absolutely *essential* that a call to `python` on the command line opens a "version 2.x" instance of Python. A simple way to accomplish this is to activate a Conda (or Miniconda) virtual environment which uses Python 2.x:
  - Download [Conda](https://docs.conda.io/projects/conda/en/latest/) or [Miniconda](https://docs.conda.io/en/latest/miniconda.html).
  - Create a new virtual environment with Python 2.x, and activate this environment.  For example:
    ```sh
    conda create --name py27 python=2.7
    conda activate py27
    ```
Of course, there is another virtual Python environment manager you prefer (e.g. venv), you are more than welcome to use it.

2. It is possible (and in most circumstances, advisable) to install [SDCC](http://sdcc.sourceforge.net/) (8051 Small-Device C Compiler) using homebrew, with the command: `brew install sdcc`

However, SiK currently relies upon a [depreciated release of SDCC](https://github.com/ArduPilot/SiK/issues/53), and will not build if the most recent stable version -- the version installed by homebrew -- is used. Rather, one must install [SDCC 3.5](https://sourceforge.net/projects/sdcc/files/sdcc/3.5.0/) manually.

3. Download the dependencies for EC2 from the command line:

```sh
brew install autoconf boost readline libusb
```

4. Attempt to build the EC2 software from the command line:

```sh
git clone https://github.com/tridge/ec2.git
cd ec2
autoreconf -i
./configure
autoreconf -i
make
make install
```

If any of these steps fail, install the following additional packages:

```sh
brew install boost-build gtk-mac-integration libusb-compat libusbmuxd
```

Delete the entire ec2 directory, and repeat the steps specified above.

5. Build the image for the standard SiK Radio / 3DR Radio:

```sh
git clone https://github.com/LorenzMeier/SiK.git
cd SiK/Firmware
make install
```

Upload firmware to the radio \(**change the serial port name, and specify the appropriate firmware file from the SiK/Firmware/dst/ directory**\):

```
tools/uploader.py --port /dev/tty.usbserial-CHANGETHIS dst/[YOUR_RADIO_MODEL_FIRMWARE].ihx
```

6. Alternatively, one can build the SiK Uploader solution file (`SiK/SikUploader/SikUploader.sln`), and use the SiK Uploader GUI instead of uploader.py.


## Radio Configuration Instructions

The radio supports 'Hayes AT' commands for configuration. The radio configuration settings can be accessed using the MAC OS X and Linux native function `screen` (see below for detailed usage).

1. Plug the the radio unit into the computer's USB port via the included FTDI cable (6-pin to USB).

2. On the command line, type \(**change the serial port name**):

```sh
screen /dev/tty.usbserial-CHANGETHIS 57600 8N1
```

Do **not** change '57600 8N1'. This informs the serial terminal to use the following connection settings:
- baud rate: 57600 (default for all RFD radios, which use a Si1000 - Si102x/3x chip).
- 8 data bits per transmission
- no parity
- 1 stop bit

Then start command mode, using the `+++` command:

> **Note** DO NOT TYPE ANYTHING ONE SECOND BEFORE AND AFTER. WAIT FOR 'OK' TO APPEAR ON THE SCREEN.

```sh
+++
```

List the current settings on the USB-connected radio unit:

```sh
ATI5
```

If working correctly, the following settings will be displayed (specific values are default for RFD900x):

```sh
S0:FORMAT=34
S1:SERIAL_SPEED=57
S2:AIR_SPEED=64
S3:NETID=25
S4:TXPOWER=30
S5:ECC=0
S6:MAVLINK=1
S7:OPRESEND=0
S8:MIN_FREQ=915000
S9:MAX_FREQ=928000
S10:NUM_CHANNELS=20
S11:DUTY_CYCLE=100
S12:LBT_RSSI=0
S13:RTSCTS=0
S14:MAX_WINDOW=131
S15:ENCRYPTION_LEVEL=0
S16:GPI1_1R/CIN=0
S17:GPO1_1R/COUT=0
S18:ANT_MODE=0
S19:PKT_DROP_RSSI=0
R0:TARGET_RSSI=255
R1:HYSTERESIS_RSI=50
```

If the complementary remote radio unit is nearby, and powered via a 5VDC source, list its current settings:

```sh
RTI5
```

In order for the radios to work correctly, it is essential that the following fields match on both radios:

- S0:FORMAT        (the radio firmware version)
- S2:AIR_SPEED     (the air data rate in ‘one byte form’)
- S3:NETID         (differentiates multiple radio pairs operating at same frequency)
- S5:ECC           (error correction protocol)
- S8:MIN_FREQ      (minimum frequency in kHz)
- S9:MAX_FREQ      (maximum frequency in kHz)
- S10:NUM_CHANNELS (number of frequency hopping channels)
- S12:LBT_RSSI     ('listen before talk' threshold)
- S14:MAX_WINDOW   (max transmit window in msecs. 131 is the default, 33 recommended for low latency, but lower bandwidth)


To change the net ID (remember the transmitter and receiver must use the same NETID value), type:

```sh
ATS3=[YOUR_DESIRED_NETID_VALUE]
```

More generally, to query any individual parameter, use:

```sh
ATS[number]?
```

To change any individual parameter, use:

```sh
ATS[number]=[value]
```

To write/save the adjusted settings, and reboot radio:

```sh
AT&W
ATZ
```
> **Note** You might have to power-cycle the radio to connect it to the 2nd radio.

For more information on AT commands, please refer to the [RFD Software Reference Guide](http://files.rfdesign.com.au/Files/documents/Software%20manual.pdf)


## `screen` Serial Terminal Commands

### General Navigation
* C-a ? -- Show help
* C-a d -- Disconnect the session. The session continues in the background as a daemon which you can reconnect to later.
* C-a K -- KILL the current screen
* C-a c -- Create a shell in new virtual screen. This screen is added to the current session.
* C-a n -- Next screen
* C-a p -- Previous screen
* C-a esc -- Enter Copy/Scrollback mode (esc again to exit scrollback mode). Scrollback mode is very useful.
    * Use Vi-like keys to move around the scrollback history (the usual h,j,k,l for cursor, C-u for half page UP, C-d for half page DOWN).
    * Press SPACE to start selecting a region. Press SPACE again to Yank the region.
    * Press "C-a ]" to paste the yanked text into the current screen.

[source: http://www.noah.org/wiki/Screen_notes]

### To Close a Detached Session:
1. Type screen -list to identify the detached screen session. ~$ screen -list     
	There are screens on:
		20751.Melvin_Peter_V42  (Detached) 

	[NOTE: 20751.Melvin_Peter_V42 is your session id.]

2. Get attached to the detached screen session screen -r 20751.Melvin_Peter_V42 

3. Once connected to the session press Ctrl + A then type :quit

[source: https://stackoverflow.com/questions/1509677/kill-detached-screen-session]
