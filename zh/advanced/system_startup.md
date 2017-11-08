---
translated_page: https://github.com/PX4/Devguide/blob/master/en/advanced/system_startup.md
translated_sha: 95b39d747851dd01c1fe5d36b24e59ec865e323e
---

# 系统启动


该PX4启动是由 [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d)文件夹下的shell脚本控制。

所有以数字和下划线（例如`10000_airplane`）开头的文件的都是内置的机型配置。他们在编译时被导出成一个`airframes.xml`文件，它被 [QGroundControl](http://qgroundcontrol.org)解析后提供给机身选择界面使用。添加新的配置参照[此处](../airframes/adding_a_new_frame.md)。

剩余的文件是常规启动逻辑的一部分，并且第一执行文件是[rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS)脚本，它调用所有其它的脚本。

## 调试系统启动

软件组件的驱动程序故障会导致启动中止。

> **Tip** 一个不完整的启动往往表现为地面站中参数丢失，因为无法启动的应用程序没有初始化它们的参数。

调试启动序列正确的方法是连接[系统控制台](../debug/system_console.md) 和为电路板供电。由此产生的启动日志包含引导序列的详细信息，而且应该包含引导中止得原因。

### 常见启动故障的原因

- 一个必须的传感器发生故障
- 对于自定义的应用程序: 该系统内存不足。运行 `free` 命令来查看可用内存量。
  - 软件故障或断言导致堆栈跟踪

## 更换系统启动

在大多数情况下，自定义修改默认的启动是更好的方法，这在后面介绍。如果要替换完整的boot，创建一个`/fs/microsd/etc/rc.txt`文件，它在microSD卡上的`etc`文件夹中。如果这个文件存在，系统中没有什么会自动启动。

## 自定义系统启动

自定义系统启动的最好的方式是引进新的 [机身配置](../airframes/adding_a_new_frame.md)。如果仅仅进行微调（如多启动一个应用程序或只是使用不同的混合器），启动时可以使用特别的hook。

> **Warning** 该系统启动文件是UNIX文件，这需要UNIX形式的行结尾。如果在Windows上编辑，请使用合适的编辑器。

主要有三个hook。需要注意的是microSD卡的根目录文件夹路径是 `/fs/microsd`。

- /fs/microsd/etc/config.txt
- /fs/microsd/etc/extras.txt
  - /fs/microsd/mixers/NAME_OF_MIXER

### 自定义配置 (config.txt)

主系统配置完成后，且在启动前，加载`config.txt` 文件，此时允许修改shell变量。

### 启动其他应用程序

 `extras.txt`可用于在主系统引导后启动额外的应用程序。通常，这些将是载荷控制器或类似的可选自定义组件。

### 启动一个自定义的mixer {#starting-a-custom-mixer}

系统默认从 `/etc/mixers`加载mixer。如果在 `/fs/microsd/etc/mixers`有相同名称的文件中存在，那么将加载该文件来代替。这允许定制混合器文件，而不需要重新编译固件。
