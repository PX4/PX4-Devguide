# SWD (JTAG) Hardware Debug Interface

PX4 usually runs on autopilot controller hardware that provides an ARM *Serial Wire Debug (SWD)* interface.
SWD is a low pin-count physical interface for JTAG debugging on ARM-processors.
It can be used with any SWD-compatible debug probe (e.g. [Segger J-Link EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/)) to set breakpoints in PX4 and step through the code running on a real device.

The SWD interface can also be used to add a new bootloader and/or firmware on a completely empty board (one that does not have the USB bootloader installed).

This topic explains how to connect the SWD interface on different boards (actually performing debugging is then covered in the associated [debugging topics](#debugging_topics)).

## Interface Definition

The SWD interface consists of the following pins.

Pin | Signal Type | Description
--- | --- | ---
`Vref` | Output| Target reference voltage. **Some** JTAG adapters (e.g. SEGGER J-Link) will use the `Vref` voltage to set the voltage on the SWD lines.
`SWDIO` | I/O | Single bi-directional data pin.
`SWCLK` | Output | Clock signal.
`GND` | - | Ground pin.

> **Note** An autopilot may also have an *Serial Wire Output (SWO)* trace output pin.
  This is optional and not "part" of SWD.
  If present it may be used in combination with *SWD* to emit real-time trace data/enable real-time tracing.

The `SWDIO`, `SWCLK` and `GND` pins on the debug probe must be connected to the corresponding pins on the autopilot.
The `VRef` pin on the debug probe must additionally be connected for debug adapters that require it (e.g. [SEGGER J-Link](segger_jlink_edu_mini)).

## Debug Ports {#debug_ports}

Autopilots use many different connectors or pinouts for exposing the SWD interface.

> **Tip** Engineers that need to debug multiple controllers should create/obtain a debug adapter board.
  This reduces potential wiring/rewiring issues that might complicate debugging.

SWD Debug port information can be found in [Autopilot Overview](http://docs.px4.io/master/en/flight_controller/) pages:
- [3DR Pixhawk](http://docs.px4.io/master/en/flight_controller/pixhawk.html#swd-port) - ARM 10-pin JTAG Connector (same interface is also used for FMUv2 boards including: *mRo Pixhawk*, *HobbyKing HKPilot32*.
- [CUAV V5nano](http://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html#debug_port) - [JST BM06B](https://www.digikey.com/product-detail/en/jst-sales-america-inc/SM06B-SRSS-TB(LF)(SN)/455-1806-1-ND/926877) (6-pin GH connector).
- [CUAV V5+](http://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html#debug-port) - [JST BM06B](https://www.digikey.com/product-detail/en/jst-sales-america-inc/SM06B-SRSS-TB(LF)(SN)/455-1806-1-ND/926877) (6-pin GH connector).


### Pixhawk Debug Port

The *standard* [Pixhawk Debug Port](https://pixhawk.org/pixhawk-connector-standard/#dronecode_debug) defines a pinout for both SWD and the [System Console](../debug/system_console.md), and mandates *JST SM06B* connector.

Debug Port | Pin
--- | ---
1 | Vtref
2 | Console TX
3 | Console RX
4 | SWDIO
5 | SWDCLK
6 | GND

This standard has not yet been widely adopted. 
While boards often use the above pinout, many use different connectors. 

> **Tip** Even for Pixhawk boards you will need to check the [debug ports](#debug_ports) definition. 








## Adapters and Wiring {#adapters_and_wiring}

Where possible, we highly recommend that you use adapter boards rather than custom cables for connecting to SWD/JTAG debuggers and computers.

This reduces the risk or poor wiring contributing to debugging problems, and has the benefit that adapters usually provide a common interface for connecting to multiple popular flight controller boards.

The following section outlines some of the available adapters, and the boards you might use with them.

### JLink EDU Mini {segger_jlink_edu_mini}

The [JLink EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/) debug adapter connector looks like this.

![connector_jlink_mini.png](../../assets/debug/connector_jlink_mini.png)

The *standard* [Pixhawk Debug Port](https://pixhawk.org/pixhawk-connector-standard/#dronecode_debug) for FMUv3, FMUv4, FMUv5 uses a *JST SM06B* connector and has the following pin outputs (and mapping to the JLink mini).
Note, the `-` indicates a pin that is not required for SWD.

Debug Port | J-Link Mini
--- | ---
1 (Vtref) | 1
2 (Console TX) | -
3 (Console RX) | -
4 (SWDIO) | 2
5 (SWDCLK)| 4
6 (GND) | 3 or 5

Many boards to not use the *standard* Pixhawk debug port *JST SM06B* connector (either in error, or because they predate the standard).

We've added information about the SWD connectors in many [Pixhawk Series](http://docs.px4.io/master/en/flight_controller/pixhawk_series.html) controller docs:
- [3DR Pixhawk](http://docs.px4.io/master/en/flight_controller/pixhawk.html#swd-port) - ARM 10-pin JTAG Connector (same interface is also used for FMUv2 boards including: *mRo Pixhawk*, *HobbyKing HKPilot32*.
- [CUAV V5nano](http://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html#debug_port) - [JST BM06B](https://www.digikey.com/product-detail/en/jst-sales-america-inc/SM06B-SRSS-TB(LF)(SN)/455-1806-1-ND/926877) (6-pin GH connector).
- [CUAV V5+](http://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html#debug-port) - [JST BM06B](https://www.digikey.com/product-detail/en/jst-sales-america-inc/SM06B-SRSS-TB(LF)(SN)/455-1806-1-ND/926877) (6-pin GH connector).

You may therefore need to create/purchase an adapter to connect to the specific board you are debugging.

The sections below outline a number of commercially available adapters, and provide links to information about debug connectors on different boards.


### Dronecode Probe (Adapter) {#dronecode_probe}

[Dronecode Probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) is a generic JTAG/SWD + UART console adapter compatible with most ARM Cortex based designs, and in particular with Pixhawk series flight controllers (and other hardware that PX4 supports).

The probe's USB interface exposes two separate virtual serial port interfaces: one for connecting to the [System Console](../debug/system_console.md) (UART) and the other for an embedded GDB server (SWD interface).

The probe provides a DCD-M connector cable for attaching to the [Pixhawk Debug port](https://pixhawk.org/pixhawk-connector-standard/#dronecode_debug) (a 6-pos *JST SM06B*), which is used on many Pixhawk flight controllers.

The probe also provides a *6-pos DF13*m which can be attached to the Pixhawk 1 SERIAL4/5 port to debug the System Console (Not SWD).

> **Note** The *Dronecode Probe* is based on the [Black Magic Probe](#black_magic_probe).
  
### Black Magic Probe {#black_magic_probe}

The [Black Magic Probe](https://github.com/blacksphere/blackmagic/wiki) is much like the [Dronecode probe](#dronecode_probe) but does not come with the same adapters for connecting to Pixhawk series flight controllers.

Adapters can be purchased separately:
- [Drone Code Debug Adapter](https://1bitsquared.com/products/drone-code-debug-adapter) (1 BIT SQUARED).


### NXP Debug Adapter {#nxp_dapter}

NXP provide a debug adaptor for the [Hovergames RDDRONE-FMUK66](#fmuk66) (see below).
This connects to the RDDRONE-FMUK66 using a 7-pin JST-GH connector and splits out an SWD cable and UART cable (for the [System Console](../debug/system_console.md)).

![Debug RDDRONE-FMUK66 using Jlink and debug interface](../../assets/debug/jlink_hovergames.png)

This can directly be used with:
* [Hovergames RDDRONE-FMUK66](#fmuk66)


### Custom Cables

You can also create or build your own cables for the board you want to use.

![Cables](../../assets/debug/just_cables.jpg)



## SWD Debugging Topics {#debugging_topics}

After connecting the DEBUG interfaces, these topics explain how you can then perform on-target debugging:

- [MCU Eclipse/J-Link Debugging for PX4](../debug/eclipse_jlink.md)

