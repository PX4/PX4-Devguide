# PX4 系统控制台

系统控制台允许对系统进行低级访问，调试输出和分析系统引导过程。 The most convenient way to connect it is by using a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation), but a plain FTDI cable can be used as well.

## 系统控制台 vs. Shells

只有一个 *System Console*，它在一个特定的 Uart（在 Nuttx 中配置的调试端口）上运行，通常通过 FTDI 电缆连接。

* 用于 *low 级调试/开发" *：启动、Nuttx、启动脚本、电路板安装、px4 核心部分 (如 uorb) 的开发。
* 特别地，这是打印所有引导输出（包括有关在启动时自动启动的应用程序的信息）的唯一位置。

Shell 提供对系统的更高级别的访问：

* 用于基本模块测试运行命令。
* 仅显示启动的模块的输出（因此无法调试引导过程）。
* 无法显示在工作队列上运行的任务的输出。

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

如果手边没有 dronecode 探头, ftdi 3.3 v (digi-key: [768-1015-ND](http://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393)) 也可以这样做。

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

## 控制台入门

输入 ls 查看本地文件系统， 输入 `free` 查看剩余内存。 飞控板上电时，终端同样可以显示系统启动日志。

```bash
nsh> ls
nsh> free
```

## MAVLink Shell{#mavlink_shell}

基于 Nuttx 的系统（Pixhawk, Pixracer, ...），nsh 终端也可以连接 MAVLink。 通过串口（USB/电台）或 WiFi（UDP/TCP）实现连接。 确保 QGC 没有运行，然后开启 shell：`./Tools/mavlink_shell.py /dev/ttyACM0`（在 Firmware 源码中， 你可能需要先安装依赖 `sudo pip install pymavlink pyserial`）。 使用 `./Tools/mavlink_shell.py -h` 获取可用参数描述，其中同样显示了 WiFi 连接的 IP 地址。 比如 `./Tools/mavlink_shell.py &lt;IP address&gt;` 可用于通过 WiFi 开启 nsh shell 连接飞控。 。

> **Tip** 你也可以使用 [QGC directly](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html) 的 nsh shell。

# 骁龙 DSP 控制台

当你通过 Usb 连接骁龙飞控时，你已经获取了 Px4 在 Posix 一方的权限。 DSP 一侧 (QuRT) 的相互作用可以通过 `qshell` 开启POSIX 应用和自身模块。

使用 USB 连接骁龙， 打开 mini-dm 查看 DSP 输出：

    ${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
    

注意：在 Mac 上可以使用 [nano-dm](https://github.com/kevinmehall/nano-dm)。

在 Linaro 运行主程序：

    cd /home/linaro
    ./px4 -s px4.config
    

用以下语法，可以通过 Linaro shell 使用已经加载到 DSP 上的所有 Apps：

    pxh> qshell command [args ...]
    

比如，查看可用的 QuRT Apps：

    pxh> qshell list_tasks
    

执行命令输出的结果显示在 minidm。