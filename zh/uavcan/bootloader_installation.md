# 安装UAVCAN启动程序

> **警告：** 无人机控制器局域网络（Unmanned Aerial Vehicle Controller Area Network，UAVCAN）设备通常在出厂时就预安装了启动程序。 如果你不对UAVCAN设备进行开发，请不要试图去重复本章的任何操作。



## 概览

对于STM32设备，PX4项目包含一个标准的UAVCAN启动程序。

启动程序占用了flash内存的最开始8-16KB的位置，它是设备上电后首先运行的代码。通常，启动程序执行设备的简单初始化，如：自动确定CAN总线的波特率， 担当UAVCAN动态ID节点客户端去获得唯一的ID节点，并且在运行应用启动之前要等待飞行控制器确认。

这个启动程序能确保，在UAVCAN设备固件无效或者错误时，无需人为干扰就可以自动恢复，此外还允许固件自动升级。

## 前提条件

初始化或更新UAVCAN启动程序需要:

- 一个SWD接口或者JTAG接口（取决于设备），比如说 [BlackMagic Probe](http://www.blacksphere.co.nz/main/blackmagic) 或 [ST-Link v2](http://www.st.com/internet/evalboard/product/251168.jsp);
- 一条连接SWD接口或ＪＴＡＧ接口与ＵＡＶＣＡＮ设备调试端口的适配线;
- [支持ARM的工具链supported ARM toolchain](../setup/dev_env.md).

## 设备的前提准备

如果用以下的操作无法连接你的设备，有可能是因为设备上已存在的固件禁用了ＭＣＵ的调试引脚。为了恢复调试引脚，你需要将你接口的ＮＲＳＴ或nSRST引脚（通常为标准20引脚ARM连接器的15引脚）与你的设备MCU的NRST引脚连接。如果需要详细信息，可以通过查看你设备的原理图与PBC设计图或者直接联系制造商。 

## 安装

你可以编译生成或直接从其他地方获取你设备启动程序的image文件（参考设备文档获取详细信息），在此之后，启动程序必须被写入设备flash存储区的起始位置。

根据使用的是SWD接口或JTAG接口，初始化步骤有所不同。

## BlackMagic Probe

确保你的BlackMagic Probe [固件版本已经更新至最新](https://github.com/blacksphere/blackmagic/wiki/Hacking).

将你的UAVCAN设备与probe连接，并将你的电脑与probe连接。

确定你probe设备的名称，设备通常名称为`/dev/ttyACM<x>` 或 `/dev/ttyUSB<x>`。

给你的UAVCAN设备供电，然后执行:

<div class="host-code"></div>

```sh
arm-none-eabi-gdb /path/to/your/bootloader/image.elf
```

当出现指示符`gdb`后，执行:

<div class="host-code"></div>

```gdb
target extended /dev/ttyACM0
monitor connect_srst enable
monitor swdp_scan
attach 1
set mem inaccessible-by-default off
load
run
```

如果 `monitor swdp_scan` 返回错误，请确保你的拼写正确并确保你的BlackMagic固件版本是最新的。

## ST-Link v2

确保 [OpenOCD](http://openocd.org)的版本为最新，至少是0.9.0版本。

将你的UAVCAN设备与ST-Link连接，并将你的电脑与ST-Link连接。

给你的UAVCAN设备供电，然后执行:

<div class="host-code"></div>

```sh
openocd -f /path/to/your/openocd.cfg &
arm-none-eabi-gdb /path/to/your/bootloader/image.elf
```

当出现指示符 `gdb`后，执行:

<div class="host-code"></div>

```gdb
target extended-remote localhost:3333
monitor reset halt
set mem inaccessible-by-default off
load
run
```

## Segger J-Link 调试器

将你的UAVCAN设备与JLink连接，并将你的电脑与JLink连接。

给你的UAVCAN设备供电，然后执行:

<div class="host-code"></div>

```JLinkGDBServer -select USB=0 -device STM32F446RE -if SWD-DP -speed 20000 -vd```

打开另一个terminal终端，定位到包含px4esc_1_6-bootloader.elf文件的目录，for the esc，然后运行:

<div class="host-code"></div>

```arm-none-eabi-gdb px4esc_1_6-bootloader.elf```

当出现指示符 `gdb`后，执行:

<div class="host-code"></div>

```tar ext :2331
load
```

## 使用SEGGER JLink调试器擦除Flash

擦除flash内存写入出厂默认值是一种有效的恢复的方法，这样固件会使用默认参数。进入SEGGER初始化目录，运行JLinkExe程序，然后执行:

```
device <name-of-device>
erase
```

上文中`<name-of-device>` 代表微控制器的名称，比如Pixhawk ESC 1.6的名称为STM32F446RE，SV2470VC ESC的名称为STM32F302K8。
