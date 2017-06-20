---
translated_page: https://github.com/PX4/Devguide/blob/master/en/debug/system_console.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# PX4 System Console(系统控制台)

System Console(系统控制台)允许访问系统底层，调试输出和分析系统启动流程。访问System Console最快捷的方式是使用 [Dronecode probe](http://nicadrone.com/index.php?id_product=65&controller=product), 但是常见的FTDI连线也是可以的。

## System Console vs. Shell


有好多种shell，但只有一个Console：系统控制台，它是打印所有引导输出（和引导中自动启动的应用程序）的位置。（可以理解为系统控制台是多个shell中唯一一个打印所有引导输出的shell）

The system console is the location where all boot output (and applications auto-started on boot) is printed.

  * System console（第一shell）：硬件串口
  * 其他shell : 连接至USB的Pixhawk(如Mac OS下显示为 /dev/tty.usbmodem1)

> **info**
> USB shell: 如果只是运行几个简单的命令或测试应用程序，连接到USB shell就足够了。
>MAVLink shell可以这么使用，参照下文。
>只有在调试启动流程或USB接口已被用于MAVlink连接地面站[GCS](../qgc/README.md)的时候，才需要使用硬件串口console。

## Snapdragon Flight : Console接线

Snapdragon Flight（骁龙开发平台）开发人员套件里面包含了一个3引脚的接线板，它可以用于访问console。 将附带的FTDI线连接到接头，并将接线板连接到扩展连接器。

## Pixracer / Pixhawk v3: Console接线

将6P JST SH 1：1线连接到Dronecode Probe，或者将连接线的每个引脚按照如下所示连接到FTDI线上：

| Pixracer / Pixhawk v3 |           | FTDI |              |
| --------------------- | --------- | ---- | ------------ |
| 1                     | +5V (红)  |      | N/C          |
| 2                     | UART7 Tx  | 5    | FTDI RX (黄) |
| 3                     | UART7 Rx  | 4    | FTDI TX (橙) |
| 4                     | SWDIO     |      | N/C          |
| 5                     | SWCLK     |      | N/C          |
| 6                     | GND       | 1    | FTDI GND (黑)|

## Pixhawk v1: Console连线

系统console可以通过Dronecode Probe或FTDI线访问。两种方式将在下面介绍。

### 通过Dronecode Probe连接

将 [Dronecode probe](http://nicadrone.com/index.php?id_product=65&controller=product) 的6P DF13 1:1线连接到Pixhawk的SERIAL4/5接口。

![](../../assets/console/dronecode_probe.jpg)

### 通过FTDI 3.3V 线连接

如果手头没有Dronecode Probe，也可以使用FTDI 3.3V (Digi-Key: [768-1015-ND](http://www.digikey.com/product-detail/en/TTL-232R-3V3/768-1015-ND/1836393)) 。

| Pixhawk 1/2 |           | FTDI |                  |
| ----------- | --------- | ---- | ---------------- |
| 1           | +5V (红)  |      | N/C              |
| 2           | S4 Tx     |      | N/C              |
| 3           | S4 Rx     |      | N/C              |
| 4           | S5 Tx     | 5    | FTDI RX (黄)     |
| 5           | S5 Rx     | 4    | FTDI TX (橙)     |
| 6           | GND       | 1    | FTDI GND (黑)    |

连接器引脚接线如下图所示。

![](../../assets/console/console_connector.jpg)

完整的接线如下图所示。

![](../../assets/console/console_debug.jpg)

## 打开Console

Console接线完成后, 使用你选择的默认串口工具或者下面描述的默认工具：

### Linux / Mac OS: Screen

Ubuntu下安装screen (Mac OS 已经默认安装了):

<div class="host-code"></div>

```bash
sudo apt-get install screen
```

  * 串口: Pixhawk v1 / Pixracer 使用 57600 波特率
  * 串行: Snapdragon Flight 使用 115200 波特率

按照 BAUDRATE baud, 8 data bits, 1 stop bit 将screen连接至正确的串口（使用 `ls /dev/tty*`命令，观察在拔下/重插USB设备时什么发生了变化）。Linux下的常见名称是 `/dev/ttyUSB0` 和 `/dev/ttyACM0` ，Mac OS下是`/dev/tty.usbserial-ABCBD`。

<div class="host-code"></div>

```bash
screen /dev/ttyXXX BAUDRATE 8N1
```

### Windows: PuTTY

下载 [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) 并启动它。

选择“串口连接”，然后设置串口参数：

  * 57600 baud
  * 8 data bits
  * 1 stop bit

## Console入门

输入`ls`查看本地文件系统，输入`free`查看剩余可用RAM。当飞控板带电重启时，console也可以显示系统启动日志。

```bash
nsh> ls
nsh> free
```

## MAVLink Shell
对于基于NuttX的系统（Pixhawk，Pixracer，...），也可以通过mavlink访问nsh console。它通过串口连接或WiFi（UDP/TCP）来工作。确保没有运行QGC，然后使用如下命令启动shell`./Tools/mavlink_shell.py /dev/ttyACM0`（在固件源代码中）。使用`-h`获得所有可用参数的描述。也许你先要使用`sudo pip install pymavlink pyserial`安装依赖文件。

# Snapdragon DSP Console
当通过USB连接到Snapdragon开发板，你可以访问PX4 shell操作posix相关资源 。与DSP侧（QuRT）的交互可以通过`qshell`posix应用程序及其QuRT companion。

将Snapdragon通过USB连接后，打开mini-dm就可以看到DSP的输出：
```
${HEXAGON_SDK_ROOT}/tools/debug/mini-dm/Linux_Debug/mini-dm
```

注意: 可选方法，尤其是在Mac上,你也可以使用 [nano-dm](https://github.com/kevinmehall/nano-dm)。

在linaro侧运行主程序：
```
cd /home/linaro
./px4 px4.config
```

你可以通过linaro shell使用DSP加载的所有的应用程序，通过以下语法：
```
pxh> qshell command [args ...]
```

例如，要查看可用QuRT应用程序：
```
pxh> qshell list_tasks
```

所执行命令的输出显示在minidm上。