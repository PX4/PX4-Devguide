---
translated_page: https://github.com/PX4/Devguide/blob/master/en/software_update/stm32_bootloader.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# STM32 BootLoader

PX4 bootloader的代码在Github的 [Bootloader](https://github.com/px4/bootloader) 仓库。

## 支持的飞控板

* FMUv1 \(PX4FMU, STM32F4\)
* FMUv2 \(Pixhawk 1, STM32F4\)
* FMUv3 \(Pixhawk 2, STM32F4\)
* FMUv4 \(Pixracer 3 和 Pixhawk 3 Pro, STM32F4\)
* FMUv5 \(Pixhawk 4, STM32F7\)
* TAPv1 \(TBA, STM32F4\)
* ASCv1 \(TBA, STM32F4\)

## 构建Bootloader

```
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
make
```

经过这一步会为所有支持的飞控板生成一系列elf文件，这些文件都在BootLoader目录中。

## 刷Bootloader

> 重要提醒：对于一些飞控板来说，为了使用JTAG\/SWD接口需要采取正确的供电顺序。正是按照所描述的这些步骤。 下列的说明适用于Blackmagic\/Dronecode探针。其他的JTAG探针可能需要使用类似的不同顺序。尝试刷BootLoader的开发者应该具备相关知识。如果你不知道如何进行这些操作，或许你应该再三考虑你是否确实需要更改BootLoader中的任何东西。

* 断开JTAG连线
* 连接USB电源线
* 连接JTAG

## 使用正确的串口

* LINUX: `/dev/serial/by-id/usb-Black_Sphere_XXX-if00`
* MAC OS: 确认使用的是xxx口而不是tty.xxx口: `tar ext /dev/tty.usbmodemDDEasdf`

```
arm-none-eabi-gdb
  (gdb) tar ext /dev/serial/by-id/usb-Black_Sphere_XXX-if00
  (gdb) mon swdp_scan
  (gdb) attach 1
  (gdb) mon option erase
  (gdb) mon erase_mass
  (gdb) load tapv1_bl.elf
        ...
        Transfer rate: 17 KB/sec, 828 bytes/write.
  (gdb) kill
```
### J-Link

关于 [J-Link GDB server](https://www.segger.com/jlink-gdb-server.html)的教程点进去就行了。

#### 必备条件

从Segger官网下载[J-Link](https://www.segger.com/downloads/jlink#) 软件并按照其教程进行安装。

#### 运行JLink GDB

FMUv1:
```bash
JLinkGDBServer -select USB=0 -device STM32F405RG -if SWD-DP -speed 20000
```

AeroFC:
```bash
JLinkGDBServer -select USB=0 -device STM32F429AI -if SWD-DP -speed 20000
```

#### 连接GDB

```bash
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```

## 故障检测

如果上述任意一条指令没有找到，要么你就是没有使用Blackmagic探针，或者是软件过时了。首先尝试升级探针软件。

如果出现这个错误： `Error erasing flash with vFlashErase packet`

断开目标连接（同时让JTAG保持连接），进而运行下列指令

```
mon tpwr disable
swdp_scan
attach 1
load tapv1_bl.elf
```

此举将禁用目标连接的电源并尝试另一个刷写循环。

