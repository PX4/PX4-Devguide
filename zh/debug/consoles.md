# PX4终端/命令行

用户可以通过 [MAVLink Shell](../debug/mavlink_shell.md) 和 [System Console](../debug/system_console.md) 访问PX4命令行终端。

这里将说明它们的主要区别，以及如何使用。


## System Console vs. Shells {#console_vs_shell}

PX4系统控制台（System Console）提供对系统的底层访问能力，获取调试信息。也可用于分析PX4的启动过程。

PX4只有一个 *System Console* 。它运行在特定的串口上（由Nuttx配置为调试口），通常可以通过FTDI线或其他调试适配器连接至电脑，比如 [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) 。
- 用于 *底层开发调试* ，例如：系统启动过程、Nuttx、启动脚本、集成电路板启动，以及PX4中一些特定组件的开发，比如uORB。
- 更具体一点，这里是包括自启动的用户应用在内的整个PX4系统下所有启动过程唯一的输出位置。

Shells provide higher-level access to the system:
- Used for basic module testing/running commands.
- Only *directly* display the output of modules you start.
- Cannot *directly* display the output of tasks running on the work queue.
- Can't debug problems when the system doesn't start (as it isn't running yet).

> **Note** The `dmesg` command is now available through the shell on some boards, enabling much lower level debugging than previously possible. For example, with `dmesg -f &` you also see the output of background tasks.

There can be several shells, either running on a dedicated UART, or via MAVLink. Since MAVLink provides more flexibility, currently only the [MAVLink Shell](../debug/mavlink_shell.md) is used.

The [System Console](../debug/system_console.md) is essential when the system does not boot (it displays the system boot log when power-cycling the board). The [MAVLink Shell](../debug/mavlink_shell.md) is much easier to setup, and so is more generally recommended for most debugging.


## Using Consoles/Shells {#using_the_console}

The MAVLink shell/console and the [System Console](../debug/system_console.md) are used in much the same way.

For example, type `ls` to view the local file system, `free` to see the remaining free RAM, `dmesg` to look at boot output.

```bash
nsh> ls
nsh> free
nsh> dmesg
```

Many other system commands and modules are listed in the [Modules and Command Reference](../middleware/modules_main.md) (e.g. `top`, `listener`, etc.).

> **Tip** Some commands may be disabled on some boards (i.e. the some modules are not included in firmware for boards with RAM or FLASH constraints). In this case you will see the response: `command not found`
