# PX4 Consoles/Shells

PX4 enables terminal access to the system through the [MAVLink Shell](../debug/mavlink_shell.md) and the [System Console](../debug/system_console.md).

This page explains the main differences and how the console/shell are used.


## System Console vs. Shells {#console_vs_shell}

The PX4 *System Console* provides low-level access to the system, debug output and analysis of the system boot process.

There is just one *System Console*, which runs on one specific UART (the debug port, as configured in NuttX), and is commonly attached to a computer via an FTDI cable (or some other debug adapter like  a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation)).
- Used for *low-level debugging/development*: bootup, NuttX, startup scripts, board bringup, development on central parts of PX4 (e.g. uORB).
- In particular, is the only place where all boot output (including information about applications auto-started on boot) is printed.

Shells provide higher-level access to the system:
- Used for basic module testing/running commands.
- Only display the output of modules you start (and therefore cannot debug the boot process).
- Cannot display the output of tasks running on the work queue.

> **Tip** The *System Console* is particularly useful when the system does not boot (it displays the system boot log when power-cycling the board).

There can be several shells, either running on a dedicated UART, or via MAVLink.
Since MAVLink provides more flexibility, currently only the [MAVLink Shell](../debug/mavlink_shell.md) is used.


## Using Consoles/Shells {#using_the_console}

The MAVLink shell/console and the [System Console](../debug/system_console.md) are used in much the same way.

For example, type `ls` to view the local file system, `free` to see the remaining free RAM, `dmesg` to look at boot output.

```bash
nsh> ls
nsh> free
nsh> dmesg
```

Many other system commands and modules are listed in the [Modules and Command Reference](../middleware/modules_main.md) (e.g. `top`, `listener`, etc.).

> **Tip** Some commands may be disabled on some boards (i.e. the some modules are not included in firmware for boards with RAM constraints).
  In this case you will see the response: `command not found`

