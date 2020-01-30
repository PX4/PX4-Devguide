# 系统启动

PX4 系统的启动由 shell 脚本文件控制。 在 NuttX 平台上这些脚本文件位于 [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) 文件夹下 - 该文件夹下的部分脚本文件也适用于 Posix (Linux/MacOS) 平台。 仅适用于 Posix 平台的启动脚本文件可以在 [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d-posix) 文件夹下找到。

All files starting with a number and underscore (e.g. `10000_airplane`) are predefined airframe configurations. 这些文件在编译时会被导出至 `airframes.xml` 文件中，[QGroundControl](http://qgroundcontrol.com) 通过解析该 xml 文件得到可以在 UI 界面上进行选择的机架构型。 如何添加一个新的配置请参阅 [这里](../airframes/adding_a_new_frame.md)。

其它的文件则是系统常规启动逻辑的一部分。 在启动过程中第一个被系统执行的脚本文件是 [init.d/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS) （Posix 平台则为 [init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS) on Posix)），该脚本会调用所有的其它脚本。

根据 PX4 运行的操作系统将本文后续内容分成了如下各小节。

## Posix (Linux/MacOS)

在 Posix 操作系统上，系统的 shell 将会作为脚本文件的解释器（例如， 在 Ubuntu 中 /bin/sh 与 Dash 建立了符号链接）。 为了使 PX4 可以在 Posix 中正常运行，需要做到以下几点：

- PX4 的各个模块需要看起来像系统的单个可执行文件， 这一点可以通过创建符号链接坐到。 每一个模块都根据命名规则： `px4-<module> -> px4` 在编译文件夹 `bin` 下创建了相应的符号链接。 在执行命令时，系统将检查命令的二进制路径 (`argv[0]`)，如果系统发现该命令是 PX4 的一个模块（命令名称以 `px4-` 起头），那么系统将会把这个命令发送给 PX4 主实例（见下文）。 > **Tip** 使用 `px4-` 这个前缀是为了避免 PX4 的命令名称与操作系统的命令名称相冲突（例如，`shutdown` 命令），同时此举也方便了在输入 `px4-<TAB>` 之后使用 tab 进行命令的自动填充。
- Shell 需要知道在那里可以找到上述符号链接。 为此，在运行启动脚本前会将包含符号链接文件的 `bin` 目录添加至操作系统的 `PATH` 环境变量中。
- Shell 将每个模块作为一个新的 (客户端) 进程进行启动， 每个客户端进程都需要与 PX4 主实例（服务器）进行通讯，在该实例中实际的模块以线程的形式运行。 该过程通过 [UNIX socket](http://man7.org/linux/man-pages/man7/unix.7.html) 完成实现。 服务器侦听一个 socket，然后客户端将连接该 socket 并通过它发送指令。 服务器收到客户端的指令后将指令运行的输出结果及返回代码重新发送给客户端。
- 启动脚本直接调用各模块，例如 `commander start`, 而不使用 `px4-` 这个前缀。 这一点可以通过设置别名（aliase）来实现：`bin/px4-alias.sh` 文件会给每一个模块以 `alias <module>=px4-<module>` 的形式设置好模块的别名。
- `rcS` 脚本由 PX4 主实例调用执行。 该脚本并不开启任何模块，它仅仅首先更新 `PATH` 环境变量然后以 `rcS` 文件作为值参数开启操作系统的 shell 。
- 除此之外，在进行多飞行器仿真时还可以启动多个服务器实例。 客户端可通过 `--instance` 选择服务器实例。 该实例可通过 `$px4_instance` 变量在脚本中使用。

当 PX4 在操作系统上处于运行状态时可以从任意终端直接运行各个模块。 例如：

    cd <Firmware>/build/px4_sitl_default/bin
    ./px4-commander takeoff
    ./px4-listener sensor_accel
    

### 动态模块

通常，所有模块都被编入一个 PX4 可执行程序。 然而，在Posix上，可以将模块编译成单独的文件，可以使用 `dyn` 命令加载到 PX4。

    dyn ./test.px4mod
    

## NuttX

NuttX 有一个内置的 shell 解释器 ([NSH](http://nuttx.org/Documentation/NuttShell.html))，因此可以直接执行启动脚本。

### 调试系统启动

软件组件的失效不会中止 PX4 系统的启动， 可以在启动脚本中使用 `set +e` 来控制。

可以通过连接 [system console](../debug/system_console.md) 并通过板载电源循环来调试引导顺序。 由此生成的启动引导日志文件中包含了引导序列的详细信息，同时也应包含了解释启动中止的线索。

#### 启动失败的常见原因

- 对于自定义的应用程序：系统用尽了 RAM 资源。 运行 `free` 命令以查看可用 RAM 的大小。
- 引发堆栈跟踪的软件故障或者断言。

### 替换系统的启动文件

在大多数情况下自定义默认启动项是更好的做法，实现方法见下文。 如果需要替换整个引导文件，请创建文件： `/fs/microsd/etc/rc.txt` ，该文件位于 microSD 卡的根目录下的 `etc` 文件夹下。 如果此文件存在，系统中的任何内容都不会自动启动。

### 自定义系统的启动文件

自定义系统启动的最佳方式是引入一个 [新机架配置](../airframes/adding_a_new_frame.md)。 如果只需要调整(例如开始一个更多的应用程序或仅仅使用一个不同的混音器)，在启动时可以使用特殊的钩子。

> **Caution** 系统的启动文件是 UNIX 系统文件，该文件要求以 UNIX 规范的 LF 作为行结束符。 在 Windows 平台上编辑系统的启动文件应该使用一个合适的文本编辑器。

主要有三类钩子。 请注意，microsd 卡的根文件夹已被路径 `/fs/microsd` 标识。

- /fs/microsd/etc/config.txt
- /fs/microsd/etc/extras.txt
- /fs/microsd/etc/mixers/NAME_OF_MIXER

#### 自定义配置（config.txt）

`config.txt` 文件可用于修改 shell 变量。 它是在主系统配置后加载的，*之前* 它已启动。

#### 启动额外的应用

`extras.txt` 可以在主系统启动后启动额外的应用程序。 通常，这些是有效载荷控制器或类似的可选自定义组件。

> **Caution**在系统启动文件中调用未知命令可能会导致系统引导失败。 通常情况下系统在引导失败后不会发送 mavlink 消息，所以在这种情况下请检查系统在控制台上输出的的错误消息。

下面的示例演示了如何启动自定义应用程序:

- 在 SD 卡上创建一个文件 `etc/extras.txt` ，该文件应包含如下内容： ```custom_app start```
- 搭配使用 `set +e` 和 `set -e` 可以将命令设置为可选命令：
    
        set +e
        optional_app start      # 即便 optional_app 未知或者失效也不会导致系统启动失败
        set -e
        
        mandatory_app start     # 如果 mandatory_app 未知或者失效则会导致系统启动中断
        

#### 启动自定义的混控器

默认情况下系统将从 `/etc/mixers` 文件夹下载入混控器。 如果在`/fs/microsd/etc/mixers`中存在同名文件，则该文件将被加载。 这允许自定义混音器文件，而无需重新编译Firmware。

##### 示例

下面的示例演示了如何添加一个自定义 aux 混控器：

- 在 SD 卡中创建文件 `etc/mixers/gimbal.aux.mix` ，并将你的混控器设定内容写入该文件内。
- 为了使用该混控器，再创建一个额外的文件 `etc/config.txt` ，该文件的内容如下： 
        set MIXER_AUX gimbal
        set PWM_AUX_OUT 1234
        set PWM_AUX_DISARMED 1500
        set PWM_AUX_MIN 1000
        set PWM_AUX_MAX 2000
        set PWM_AUX_RATE 50