# UAVCAN 固件升级

## Ectorcontrol ESC 代码库（Pixhawk ESC 1.6 和 S2740VC）

下载电调代码

```sh
git clone https://github.com/thiemar/vectorcontrol
cd vectorcontrol
```

### 刷写 Bootloader

在通过 UAVCAN 更新固件之前，Pixhawk ESC 1.6 需要闪存 UAVCAN 引导加载程序。 要构建引导加载程序，请运行：

```sh
make clean && BOARD=px4esc_1_6 make -j8
```

构建之后，引导加载程序映像位于 `firmware/px4esc_1_6-bootloader.bin`，并且 OpenOCD 配置位于 `openocd_px4esc_1_6.cfg`。 按照 [these instructions](../uavcan/bootloader_installation.md) 在 ESC 上安装引导加载程序。

### 编译二进制文件

```sh
BOARD=s2740vc_1_0 make && BOARD=px4esc_1_6 make
```

这将为两个支持的电调来构建 UAVCAN 节点固件。 固件将位于 `com.thiemar.s2740vc-v1-1.0-1.0。&lt;git hash&gt;.bin` 和 `org.pixhawk.px4esc-v1-1.6-1.0。&lt;git hash&gt;.binn`。

## Sapog Codebase（Pixhawk ESC 1.4和Zubax Orel 20）

下载 Sapog 代码库：

```sh
git clone https://github.com/PX4/sapog
cd sapog
git submodule update --init --recursive
```

### 刷写 Bootloader

在通过 UAVCAN 更新固件之前，ESC 需要闪存 UAVCAN 引导加载程序。 引导加载程序可以构建如下：

```sh
cd bootloader
make clean && make -j8
cd ..
```

引导加载程序映像位于 `bootloader/firmware/bootloader.bin`，OpenOCD 配置位于 `openocd.cfg`。 按照 [these instructions](../uavcan/bootloader_installation.md) 在 ESC 上安装引导加载程序。

### 编译二进制文件

```sh
cd firmware
make RELEASE=1 # RELEASE is optional; omit to build the debug version
```

请注意，一些较新版本的 GCC 会在链接期间导致段错误。 版本 4.9 在撰写本文时确实有效。 固件映像将位于 `firmware/build/io.px4.sapog-1.1-1.7。&lt;xxxxxxxx&gt;.application.bin`, where`&lt;xxxxxxxx&gt;`是数字和字母的任意序列。 Zubax Orel 20 有两个硬件版本（1.0 和 1.1）。 确保将二进制文件复制到后续说明中的正确文件夹中。 ESC 固件将检查硬件版本并适用于这两种产品。

## Zubax GNSS

请参阅 [project page](https://github.com/Zubax/zubax_gnss) 以了解如何构建和刷新固件。 Zubax GNSS 配备了支持 UAVCAN 的引导加载程序，因此可以通过 UAVCAN 以统一的方式更新其固件，如下所述。

## 无人机上的固件安装

无论制造商如何，UAVCAN 节点文件名遵循命名约定，允许 Pixhawk 更新网络上的所有 UAVCAN 设备。 因此，必须将上述步骤中生成的固件文件复制到 SD 卡或 PX4 ROMFS 上的正确位置，以便更新设备。

固件映像名称的约定是：

    <uavcan name>-<hw version major>.<hw version minor>-<sw version major>.<sw version minor>.<version hash>.bin
    

e.g. `com.thiemar.s2740vc-v1-1.0-1.0.68e34de6.bin`

However, due to space/performance constraints (names may not exceed 28 charates), the UAVCAN firmware updater requires those filenames to be split and stored in a directory structure like the following:

    /fs/microsd/fw/<node name>/<hw version major>.<hw version minor>/<hw name>-<sw version major>.<sw version minor>.<git hash>.bin
    

e.g.

    s2740vc-v1-1.0.68e34de6.bin 
    /fs/microsd/fw/io.px4.sapog/1.1/sapog-1.7.87c7bc0.bin
    

The ROMFS-based updater follows that pattern, but prepends the file name with ```_``` so you add the firmware in:

    /etc/uavcan/fw/<device name>/<hw version major>.<hw version minor>/_<hw name>-<sw version major>.<sw version minor>.<git hash>.bin
    

## Placing the binaries in the PX4 ROMFS

The resulting finale file locations are:

* S2740VC ESC: `ROMFS/px4fmu_common/uavcan/fw/com.thiemar.s2740vc-v1/1.0/_s2740vc-v1-1.0.<git hash>.bin`
* Pixhawk ESC 1.6: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.px4esc-v1/1.6/_px4esc-v1-1.6.<git hash>.bin`
* Pixhawk ESC 1.4: `ROMFS/px4fmu_common/uavcan/fw/org.pixhawk.sapog-v1/1.4/_sapog-v1-1.4.<git hash>.bin``
* Zubax GNSS v1: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/1.0/gnss-1.0.<git has>.bin`
* Zubax GNSS v2: `ROMFS/px4fmu_common/uavcan/fw/com.zubax.gnss/2.0/gnss-2.0.<git has>.bin`

Note that the ROMFS/px4fmu_common directory will be mounted to /etc on Pixhawk.

### Starting the Firmware Upgrade process

When using the PX4 Flight Stack, enable UAVCAN in the 'Power Config' section and reboot the system before attempting an UAVCAN firmware upgrade.

Alternatively UAVCAN firmware upgrading can be started manually on NSH via:

```sh
uavcan start
uavcan start fw
```