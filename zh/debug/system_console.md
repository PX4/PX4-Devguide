# PX4 系统控制台

The PX4 *System Console* provides low-level access to the system, debug output and analysis of the system boot process. The most convenient way to connect it is by using a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation), but a plain FTDI cable can be used as well.

> **Tip** The [MAVLink Shell](#mavlink_shell) (accessed via *QGroundControl*) can be used for [many of the same tasks](#console_vs_shell) as the *System Console*, and is much easier to set up.

## System Console vs. Shells {#console_vs_shell}

只有一个 *System Console*，它在一个特定的 Uart（在 Nuttx 中配置的调试端口）上运行，通常通过 FTDI 电缆连接。

* 用于 *low 级调试/开发" *：启动、Nuttx、启动脚本、电路板安装、px4 核心部分 (如 uorb) 的开发。
* 特别地，这是打印所有引导输出（包括有关在启动时自动启动的应用程序的信息）的唯一位置。

Shell 提供对系统的更高级别的访问：

* 用于基本模块测试运行命令。
* 仅显示启动的模块的输出（因此无法调试引导过程）。
* 无法显示在工作队列上运行的任务的输出。

> **Tip** The is particularly useful when the system does not boot (it displays the system boot log when power-cycling the board).

可以有多个 Shell，可以在专用 Uart 上运行，也可以通过 MAVLink 运行。 由于 MAVLink 提供了更大的灵活性，现在只使用 [via MAVLink](#mavlink_shell)。

## 骁龙飞控：接线控制台

开发人员套件附带了一个带有三个引脚的电路板，用于访问控制台。 将捆绑的 FTDI 电缆连接到接口，将断路板连接到扩展连接器。

## Pixracer / Pixhawk v3: 接线控制台

将6脚 JST SH 1:1 线缆连接到飞控探测器或者按如下图示连接单独的 pin 脚：

| Pixracer / Pixhawk v3 |           | FTDI |               |
| --------------------- | --------- | ---- | ------------- |
| 1                     | + 5v (红色) |      | N/C           |
| 2                     | UART7 Tx  | 5    | FTDI RX （黄色）  |
| 3                     | UART7 Rx  | 4    | FTDI TX （橙色）  |
| 4                     | SWDIO     |      | N/C           |
| 5                     | SWCLK     |      | N/C           |
| 6                     | GND       | 1    | FTDI GND (黑色) |

## Pixhawk v1: 接线控制台

系统控制台可以通过 dronecode 探头或 ftdi 电缆访问。 下面的部分对这两个选项进行了说明。

### 使用 Dronecode 探头连接

Connect the 6-pos DF13 1:1 cable on the [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) to the SERIAL4/5 port of Pixhawk.

![Dronecode probe](../../assets/console/dronecode_probe.jpg)

### 通过 ftdi 3.3 v 电缆连接

If no Dronecode probe is at hand an FTDI 3.3V (Digi-Key: [768-1015-ND](https://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393)) will do as well.

| Pixhawk 1/2 |           | FTDI |               |
| ----------- | --------- | ---- | ------------- |
| 1           | + 5v (红色) |      | N/C           |
| 2           | S4 Tx     |      | N/C           |
| 3           | S4 Rx     |      | N/C           |
| 4           | S5 Tx     | 5    | FTDI RX （黄色）  |
| 5           | S5 Rx     | 4    | FTDI TX （橙色）  |
| 6           | GND       | 1    | FTDI GND (黑色) |

连接器引脚如下图所示。

![控制台连接器](../../assets/console/console_connector.jpg)

完整的布线如下所示。

![控制台调试](../../assets/console/console_debug.jpg)

## 打开控制台

连接控制台连接后，请使用您选择的默认串口工具或下面描述的默认工具：

### Linux / Mac OS: Screen

在 Ubuntu 上安装 screen （mac os 已经安装了它）：

```bash
sudo apt-get install screen
```

* 串口：pixhawk v1/pixracer 使用 57600 波特率
* 串口：骁龙飞控使用115200波特率

将 Screen 的波特率、8个数据位、1个停止位设置好，连接到正确的串行端口（使用 `ls/dev/tty*`，并观察拔下/复制 usb 设备时发生的变化）。 对于 linux 和 mac os，常见的名称 `/dev/ttyUSB0` 和 `/dev/ttyACM0`。

```bash
screen /dev/ttyXXX BAUDRATE 8N1
```

### Windows: PuTTY

下载 [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) 并启动它。

然后选择 "串行连接", 并将端口参数设置为:

* 57600 波特率
* 8 数据位
* 1 个停止位

## MAVLink Shell {#mavlink_shell}

For NuttX-based systems (Pixhawk, Pixracer, ...), the *nsh console* can also be accessed via MAVLink over serial (USB/Telemetry) or WiFi (UDP/TCP) links.

The easiest way to access the *nsh console* via MAVLink is using [QGroundControl](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html) (see **Analyze View > Mavlink Console**).

You can also access the shell in a terminal using the **mavlink_shell.py** script:

1. Shut down QGroundControl.
2. Install dependencies: 
        sh
        sudo pip3 install pymavlink pyserial

3. Open terminal (in Firmware directory) and start the shell: 
        sh
        # For serial port
        ./Tools/mavlink_shell.py /dev/ttyACM0
    
        sh
        # For Wifi connection
        ./Tools/mavlink_shell.py 0.0.0.0:14550

Use `mavlink_shell.py -h` to get a description of all available arguments.

## Getting Started on the Console/Shell {#getting-started-on-the-console}

The MAVLink shell/console and the System Console are used in much the same way.

> **Note** For more information about the differences see: [System Console vs. Shells](#console_vs_shell).

Type `ls` to view the local file system, `free` to see the remaining free RAM, `dmesg` to look at boot output.

```bash
nsh> ls
nsh> free
nsh> dmesg
```

Many other system commands and modules are listed in the [Modules and Command Reference](../middleware/modules_main.md) (e.g. `top`, `listener`, etc.).

> **Tip** Some commands may be disabled on some boards (i.e. the some modules are not included in firmware for boards with RAM constraints). In this case you will see the response: `command not found`

# 骁龙 DSP 控制台

When you are connected to your Snapdragon board via usb you have access to the px4 shell on the posix side of things. The interaction with the DSP side (QuRT) is enabled with the `qshell` posix app and its QuRT companion.

With the Snapdragon connected via USB, open the mini-dm to see the output of the DSP:

    ${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
    

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