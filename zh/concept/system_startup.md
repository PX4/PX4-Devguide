# 系统启动

PX4 系统的启动由 shell 脚本文件控制。 在 NuttX 平台上这些脚本文件位于 [ROMFS/px4fmu_common/init.d](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d) 文件夹下 - 该文件夹下的部分脚本文件也适用于 Posix (Linux/MacOS) 平台。 仅适用于 Posix 平台的启动脚本文件可以在 [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/Firmware/tree/master/ROMFS/px4fmu_common/init.d-posix) 文件夹下找到。

上述文件夹中以数字和下划线为文件名开头的脚本文件（例如，`10000_airplane`）都是封装好的机架构型配置文件。 这些文件在编译时会被导出至 `airframes.xml` 文件中，[QGroundControl](http://qgroundcontrol.com) 通过解析该 xml 文件得到可以在 UI 界面上进行选择的机架构型。 如何添加一个新的配置请参阅 [这里](../airframes/adding_a_new_frame.md)。

其它的文件则是系统常规启动逻辑的一部分。 在启动过程中第一个被系统执行的脚本文件是 [init.d/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d/rcS) （Posix 平台则为 [init.d-posix/rcS](https://github.com/PX4/Firmware/blob/master/ROMFS/px4fmu_common/init.d-posix/rcS) on Posix)），该脚本会调用所有的其它脚本。

根据 PX4 运行的操作系统将本文后续内容分成了如下各小节。

## Posix (Linux/MacOS)

在 Posix 操作系统上，系统的 shell 将会作为脚本文件的解释器（例如， 在 Ubuntu 中 /bin/sh 与 Dash 建立了符号链接）。 为了使 PX4 可以在 Posix 中正常运行，需要做到以下几点：

- PX4 的各个模块需要看起来像系统的单个可执行文件， 这一点可以通过创建符号链接坐到。 每一个模块都根据命名规则： `px4-<module> -> px4` 在编译文件夹 `bin` 下创建了相应的符号链接。 在执行命令时，系统将检查命令的二进制路径 (`argv[0]`)，如果系统发现该命令是 PX4 的一个模块（命令名称以 `px4-` 起头），那么系统将会把这个命令发送给 PX4 主实例（见下文）。 > **Tip** The `px4-` prefix is used to avoid conflicts with system commands (e.g. `shutdown`), and it also allows for simple tab completion by typing `px4-<TAB>`.
- The shell needs to know where to find the symbolic links. For that the `bin` directory with the symbolic links is added to the `PATH` variable right before executing the startup scripts.
- The shell starts each module as a new (client) process. Each client process needs to communicate with the main instance of px4 (the server), where the actual modules are running as threads. This is done through a [UNIX socket](http://man7.org/linux/man-pages/man7/unix.7.html). The server listens on a socket, to which clients can connect and send a command. The server then sends the output and return code back to the client.
- The startup scripts call the module directly, e.g. `commander start`, rather than using the `px4-` prefix. This works via aliases: for each module an alias in the form of `alias <module>=px4-<module>` is created in the file `bin/px4-alias.sh`.
- The `rcS` script is executed from the main px4 instance. It does not start any modules, but first updates the `PATH` variable and then simply runs a shell with the `rcS` file as argument.
- In addition to that, multiple server instances can be started for multi-vehicle simulations. A client selects the instance via `--instance`. The instance is available in the script via `$px4_instance` variable.

The modules can be executed from any terminal when PX4 is already running on a system. 例如：

    cd <Firmware>/build/px4_sitl_default/bin
    ./px4-commander takeoff
    ./px4-listener sensor_accel
    

## NuttX

NuttX has an integrated shell interpreter ([NSH](http://nuttx.org/Documentation/NuttShell.html)), and thus scripts can be executed directly.

### Debugging the System Boot

A failure of a driver of software component will not lead to an aborted boot. This is controlled via `set +e` in the startup script.

The boot sequence can be debugged by connecting the [system console](../debug/system_console.md) and power-cycling the board. The resulting boot log has detailed information about the boot sequence and should contain hints why the boot aborted.

#### Common boot failure causes

- For custom applications: The system was out of RAM. Run the `free` command to see the amount of free RAM.
- A software fault or assertion resulting in a stack trace

### Replacing the System Startup

In most cases customizing the default boot is the better approach, which is documented below. If the complete boot should be replaced, create a file `/fs/microsd/etc/rc.txt`, which is located in the `etc` folder on the microSD card. If this file is present nothing in the system will be auto-started.

### Customizing the System Startup

The best way to customize the system startup is to introduce a [new airframe configuration](../airframes/adding_a_new_frame.md). If only tweaks are wanted (like starting one more application or just using a different mixer) special hooks in the startup can be used.

> **Caution** The system boot files are UNIX FILES which require UNIX LINE ENDINGS. If editing on Windows use a suitable editor.

There are three main hooks. Note that the root folder of the microsd card is identified by the path `/fs/microsd`.

- /fs/microsd/etc/config.txt
- /fs/microsd/etc/extras.txt
- /fs/microsd/etc/mixers/NAME_OF_MIXER

#### Customizing the Configuration (config.txt)

The `config.txt` file can be used to modify shell variables. It is loaded after the main system has been configured and *before* it is booted.

#### Starting additional applications

The `extras.txt` can be used to start additional applications after the main system boot. Typically these would be payload controllers or similar optional custom components.

> **Caution** Calling an unknown command in system boot files may result in boot failure. Typically the system does not stream mavlink messages after boot failure, in this case check the error messages that are printed on the system console.

The following example shows how to start custom applications:

- Create a file on the SD card `etc/extras.txt` with this content: ```custom_app start```
- A command can be made optional by gating it with the `set +e` and `set -e` commands:
    
        set +e
        optional_app start      # Will not result in boot failure if optional_app is unknown or fails
        set -e
        
        mandatory_app start     # Will abort boot if mandatory_app is unknown or fails
        

#### Starting a custom mixer

By default the system loads the mixer from `/etc/mixers`. If a file with the same name exists in `/fs/microsd/etc/mixers` this file will be loaded instead. This allows to customize the mixer file without the need to recompile the Firmware.

##### Example

The following example shows how to add a custom aux mixer:

- Create a file on the SD card, `etc/mixers/gimbal.aux.mix` with your mixer content.
- Then to use it, create an additional file `etc/config.txt` with this content: 
        set MIXER_AUX gimbal
        set PWM_AUX_OUT 1234
        set PWM_AUX_DISARMED 1500
        set PWM_AUX_MIN 1000
        set PWM_AUX_MAX 2000
        set PWM_AUX_RATE 50