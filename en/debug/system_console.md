# PX4 System Console

The PX4 *System Console* provides low-level access to the system, debug output and analysis of the system boot process.

> **Tip** The console should be used for debugging if the system won't boot.
  The [MAVLink Shell](../debug/mavlink_shell.md) may otherwise be more suitable, as it is much easier to set up and can be used for [many of the same tasks](../debug/consoles.md#console_vs_shell).


## Wiring the Console

The console is made available through a (board-specific) system UART that can be connected to a computer USB port using a [3.3V FTDI](https://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393) cable.
This allows the console to be accessed using a terminal application.

Pixhawk controller manufacturers are expected to expose the console UART and SWD (JTAG) debug interfaces through a dedicated *debug port* that complies with the [Pixhawk Connector Standard](pixhawk_debug_port).
Unfortunately some boards predate this standard or a non-compliant.

The sections below outline the connections for many common boards.

> **Tip** Developers targeting a number of different boards may wish to use a Debug Adapter to simplify connecting multiple boards.
  For exmaple, the [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) comes with connectors for several different boards.

### Pixhawk Debug Port {#pixhawk_debug_port}

Flight controllers that adhere to the Pixhawk Connector standard use the [Pixhawk Standard Debug Port](
https://pixhawk.org/pixhawk-connector-standard/#dronecode_debug).

The port/FTDI mapping is shown below.

Pixhawk Debug Port | - | FTDI | -
--- | --- | --- | ---
1 (red) | TARGET PROCESSOR VOLTAGE | | N/C
2 (blk) | CONSOLE TX (OUT) | 5 | FTDI RX (yellow)
3 (blk) | CONSOLE RX (IN) | 4 | FTDI TX (orange)
4 (blk) | SWDIO | | N/C
5 (blk) | SWCLK | | N/C
6 (blk) | GND | 1 | FTDI GND (black) 
 


### Pixracer / Pixhawk v3: 

Connect the 6-pos JST SH 1:1 cable to the Dronecode probe or connect the individual pins of the cable to a FTDI cable like this:

| Pixracer / Pixhawk v3  |         | FTDI    |        |
| -- | -- | -- | -- |
|1         | +5V (red)     |         | N/C    |
|2         | UART7 Tx      | 5       | FTDI RX (yellow)  |
|3         | UART7 Rx      | 4       | FTDI TX (orange)  |
|4         | SWDIO      |         | N/C   |
|5         | SWCLK      |         | N/C   |
|6         | GND     | 1       | FTDI GND (black)   |

### Pixhawk v1

The system console can be accessed through the Dronecode probe or an FTDI cable.
Both options are explained in the section below.

#### Connecting via Dronecode Probe

Connect the 6-pos DF13 1:1 cable on the [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) to the SERIAL4/5 port of Pixhawk.

![Dronecode probe](../../assets/console/dronecode_probe.jpg)

#### Connecting via FTDI 3.3V Cable

If no Dronecode probe is at hand an FTDI 3.3V (Digi-Key: [768-1015-ND](https://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393)) will do as well.

| Pixhawk 1/2  |         | FTDI    |        |
| -- | -- | -- | -- |
|1         | +5V (red)     |         | N/C    |
|2         | S4 Tx      |         | N/C   |
|3         | S4 Rx      |         | N/C   |
|4         | S5 Tx      | 5       | FTDI RX (yellow)   |
|5         | S5 Rx      | 4       | FTDI TX (orange)   |
|6         | GND     | 1       | FTDI GND (black)   |

The connector pinout is shown in the figure below.

![Console Connector](../../assets/console/console_connector.jpg)

The complete wiring is shown below.

![Console Debug](../../assets/console/console_debug.jpg)


### Snapdragon Flight

The [Snapdragon Flight](https://docs.px4.io/master/en/flight_controller/snapdragon_flight.html) developer kit comes with a breakout board with three pins to access the console.
Connect the bundled FTDI cable to the header and the breakout board to the expansion connector.


## Opening the Console

After the console connection is wired up, use the default serial port tool of your choice or the defaults described below:

### Linux / Mac OS: Screen

Install screen on Ubuntu (Mac OS already has it installed):

```bash
sudo apt-get install screen
```

* Serial: Pixhawk v1 / Pixracer use 57600 baud
* Serial: Snapdragon Flight uses 115200 baud

Connect screen at BAUDRATE baud, 8 data bits, 1 stop bit to the right serial port (use `ls /dev/tty*` and watch what changes when unplugging / replugging the USB device). Common names are `/dev/ttyUSB0` and `/dev/ttyACM0` for Linux and `/dev/tty.usbserial-ABCBD` for Mac OS.

```bash
screen /dev/ttyXXX BAUDRATE 8N1
```

### Windows: PuTTY

Download [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) and start it.

Then select 'serial connection' and set the port parameters to:

* 57600 baud
* 8 data bits
* 1 stop bit


### Snapdragon DSP Console

When you are connected to your Snapdragon board via USB you have access to the PX4 shell on the POSIX side of things.
The interaction with the DSP side (QuRT) is enabled with the `qshell` posix app and its QuRT companion.

With the Snapdragon connected via USB, open the mini-dm to see the output of the DSP:
```
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

> **Note** Alternatively, especially on Mac, you can also use [nano-dm](https://github.com/kevinmehall/nano-dm).

Run the main app on the linaro side:
```sh
cd /home/linaro
./px4 -s px4.config
```

You can now use all apps loaded on the DSP from the linaro shell with the following syntax:
```sh
pxh> qshell command [args ...]
```

For example, to see the available QuRT apps:
```sh
pxh> qshell list_tasks
```

The output of the executed command is displayed on the minidm.


## Using the Console {#using-the-console}

For information see: [PX4 Consoles/Shells > Using Consoles/Shells](../debug/consoles.md#using_the_console).

