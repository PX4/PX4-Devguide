# MAVLink Shell

MAVLink Shell 是一个可以通过串口（USB、数传或基于WIFI的UDP/TCP链路）使用MAVLink协议访问的 *NSH 控制台* 。只适用于基于NuttX的系统，如：Pixhawk、Pixracer等。

它可用于启动系统指令与模块，并得到输出信息。 尽管它不能*直接*显示那些不是由它启动的模块的输出，但是可以间接的使用 `dmesg` 命令来查询。执行 `dmesg -f &` 可以打印出工作队列中其它模块和任务的输出信息。

> **提示** [QGroundControl MAVLink Console](#qgroundcontrol) 是访问控制台最方便的方法。 如果系统未能正常启动，则应使用[System Console](../debug/system_console.md)。

## 启用 Shell

### QGroundControl MAVLink Console {#qgroundcontrol}

访问 shell 的最简单方式是使用 [QGroundControl MAVLink Console](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_console.html) (见**Analyze View > Mavlink Console**)。

### mavlink_shell.py

You can also access the shell in a terminal using the **mavlink_shell.py** script:
1. Shut down *QGroundControl*.
1. Install dependencies:
   ```sh
   sudo pip3 install pymavlink pyserial
   ```
1. Open terminal (in Firmware directory) and start the shell:
   ```sh
   # 用于串口
   ./Tools/mavlink_shell.py /dev/ttyACM0
   ```
    ```sh
   # 用于 WiFi 连接
   ./Tools/mavlink_shell.py 0.0.0.0:14550
   ```

Use `mavlink_shell.py -h` to get a description of all available arguments.

## 使用 MAVLink Shell

For information see: [PX4 Consoles/Shells > Using Consoles/Shells](../debug/consoles.md#using_the_console).
