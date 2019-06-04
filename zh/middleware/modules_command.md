# 模块参考：命令（Command）

## bl_update

源码： [systemcmds/bl_update](https://github.com/PX4/Firmware/tree/master/src/systemcmds/bl_update)

用于从文件中刷新飞行控制器的 引导加载程序（bootloader ）

### 用法 {#bl_update_usage}

    bl_update [arguments...]
       setopt        设置选项字节（option bits）以解锁  FLASH (仅当其处于锁定状态时)
    
       &lt;file&gt;        Bootloader bin 文件
    

## config

源码： [systemcmds/config](https://github.com/PX4/Firmware/tree/master/src/systemcmds/config)

配置传感器的驱动（设定传感器采样 & 发布速率，量程等）

### 用法 {#config_usage}

    config &lt;command&gt; [arguments...]
     Commands:
    
    &lt;file:dev&gt; 参数通常是 /dev/{gyro,accel,mag}i 中的一个
       block         堵塞传感器话题的发布
         &lt;file:dev&gt;  传感器设备文件
    
       unblock       恢复传感器话题的发布
         &lt;file:dev&gt;  传感器设备文件
    
       sampling      设定传感器采样速率
         &lt;file:dev&gt; &lt;rate&gt; 传感器设备文件，采样速率 Hz
    
       rate          设定传感器数据发布速率
         &lt;file:dev&gt; &lt;rate&gt; 传感器设备文件，发布速率 Hz
    
       range         设定传感器测量量程
         &lt;file:dev&gt; &lt;rate&gt; 传感器设备文件和量程
    
       check         执行传感器自检（并打印自检信息）
         &lt;file:dev&gt;  传感器设备文件
    

## dumpfile

源码： [systemcmds/dumpfile](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dumpfile)

转储文件应用。 将文件大小及内容以二进制模式输出值标准输出设备（不使用 LF 替换 CR LF）。

### 用法 {#dumpfile_usage}

    dumpfile [arguments...]
         &lt;file&gt;      需要进行转储的文件
    

## dyn

源码：[systemcmds/dyn](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dyn)

### 描述

载入并运行一个未被编译至 PX4 二进制文件内的动态 PX4 模块。

### 示例

    dyn ./hello.px4mod start
    

### 用法 {#dyn_usage}

    dyn [arguments...]
         &lt;file&gt;      包含模块的文件
         [arguments...] 传递给模块的参数
    

## esc_calib

源码： [systemcmds/esc_calib](https://github.com/PX4/Firmware/tree/master/src/systemcmds/esc_calib)

ESC 校准工具。

校准流程（运行命令将会引导你完成此流程）：

- 移除螺旋桨，将 ESC 断电
- 停止姿态控制器： mc_att_control stop， fw_att_control stop
- 确保安全设置断开（Make sure safety is off）
- 运行这个命令

### 用法 {#esc_calib_usage}

    esc_calib [arguments...]
         [-d &lt;val&gt;]  选择 PWM 输出设备
                     取值 &lt;file:dev&gt;, 默认值： /dev/pwm_output0
         [-l &lt;val&gt;]  Low PWM 值，单位 us
                     默认值： 1000
         [-h &lt;val&gt;]  High PWM 值，单位 us
                     默认值：2000
         [-c &lt;val&gt;]  使用如下形式选取通道：1234 (1 位数字表示一个通道，
                     1=第一个通道)
         [-m &lt;val&gt;]  使用位掩码（bitmask）选取通道 0xF, 3)
         [-a]        Select all channels
    

## hardfault_log

源码： [systemcmds/hardfault_log](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log)

硬错误处理程序。

在启动脚本中用于处理硬错误。

### 用法 {#hardfault_log_usage}

    hardfault_log &lt;command&gt; [arguments...]
     Commands:
       check         检查是否存在未提交的硬错误（uncommited hardfault）
    
       rearm         抛下一个未提交的硬错误
    
       fault         生成一个硬错误 (该命令会导致系统崩溃:)
         [0|1]       硬错误类型： 0=除 0 错误, 1=断言错误（Assertion） (默认值=0)
    
       commit        讲一个未提交的硬错误写入 /fs/microsd/fault_%i.txt (然后
                     rearm但不 reset）
    
       count         读取重启计数器，计算一个未提交的硬错误引起的重启次数
                      (该结果将作为程序的退出代码返回)
    
       reset         重置重启计数器
    

## led_control

源码： [systemcmds/led_control](https://github.com/PX4/Firmware/tree/master/src/systemcmds/led_control)

### 描述

用于控制 & 测试 （外部） LED's 的命令行工具。

要使用该命令请确保有一个负责处理 led_control 的 uorb 主题处于运行状态。

有不同的优先级，例如，一个模块可设置一个低优先级的颜色，然后另一个模块可设置一个高优先级的闪烁 N 次的动作，LED 在完成闪烁后会自动返回较低优先级的状态。 也可使用 `reset` 命令来返回至一个更低的优先级。

### 示例

第一个 LED 闪烁蓝光 5 次：

    led_control blink -c blue -l 0 -n 5
    

### 用法 {#led_control_usage}

    led_control &lt;command&gt; [arguments...]
     Commands:
       test          运行一个测试范例
    
       on            点亮 LED
    
       off            熄灭 LED
    
       reset         重置 LED 优先级
    
       blink         闪烁 LED 灯 N 次
         [-n &lt;val&gt;]  闪烁次数
                     默认值： 3
         [-s &lt;val&gt;]  设定闪烁速度
                    取值： fast|normal|slow, 默认值：normal
    
       breathe       LED 持续淡入 & 淡出（呼吸效果）
    
       flash         以 1Hz 速度快速闪烁两次然后关闭 LED
    
    下述参数可用于上述除  'test' 命令之外的所有命令：
         [-c &lt;val&gt;]  color
                     取值： red|blue|green|yellow|purple|amber|cyan|white, 默认值：
                     white
         [-l &lt;val&gt;]  需要控制哪一个 LED： 0, 1, 2, ... (default=all)
         [-p <val>]  Priority
                     default: 2
    

## listener

源码： [systemcmds/topic_listener](https://github.com/PX4/Firmware/tree/master/src/systemcmds/topic_listener)

用于监听 uORB 主题并将数据输出在控制台上的工具。

The listener can be exited any time by pressing Ctrl+C, Esc, or Q.

### 用法 {#listener_usage}

    listener &lt;command&gt; [arguments...]
     Commands:
         &lt;topic_name&gt; uORB 主题名称
         [-i &lt;val&gt;]  主题实例
                     default: 0
         [-n &lt;val&gt;]  消息数量
                     default: 1
         [-r &lt;val&gt;]  订阅速率 (0 表示不限制)
                     default: 0
    

## mixer

Source: [systemcmds/mixer](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mixer)

### 描述

Load or append mixer files to the ESC driver.

Note that the driver must support the used ioctl's, which is the case on NuttX, but for example not on RPi.

### 用法 {#mixer_usage}

    mixer &lt;command&gt; [arguments...]
     Commands:
       load
         &lt;file:dev&gt; &lt;file&gt; 输出装置 （例如，/dev/pwm_output0）和混控器文件
       append
         &lt;file:dev&gt; &lt;file&gt; 输出装置 （例如，/dev/pwm_output0）和混控器文件
    

## motor_ramp

Source: [systemcmds/motor_ramp](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_ramp)

### 描述

Application to test motor ramp up.

Before starting, make sure to stop any running attitude controller:

    mc_att_control stop
    fw_att_control stop
    

When starting, a background task is started, runs for several seconds (as specified), then exits.

Note: this command currently only supports the `/dev/pwm_output0` output.

### 示例

    motor_ramp sine 1100 0.5
    

### 用法 {#motor_ramp_usage}

    motor_ramp [arguments...]
         ramp|sine|square mode
         &lt;min_pwm&gt; &lt;time&gt; [&lt;max_pwm&gt;] pwm value in us, time in sec
    
    警告：电机将加速到最大速度！
    

## motor_test

Source: [systemcmds/motor_test](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_test)

Utility to test motors.

Note: this can only be used for drivers which support the motor_test uorb topic (currently uavcan and tap_esc)

### 用法 {#motor_test_usage}

    motor_test <command> [arguments...]
     Commands:
       test          Set motor(s) to a specific output value
         [-m <val>]  Motor to test (0...7, all if not specified)
         [-p <val>]  Power (0...100)
                     default: 0
    
       stop          Stop all motors
    
       iterate       Iterate all motors starting and stopping one after the other
    

## mtd

Source: [systemcmds/mtd](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mtd)

Utility to mount and test partitions (based on FRAM/EEPROM storage as defined by the board)

### 用法 {#mtd_usage}

    mtd &lt;command&gt; [arguments...]
     Commands:
       status        打印状态信息
    
       start         挂在分区
    
       readtest      进行读取测试
    
       rwtest        进行读写测试
    
       erase         擦除分区
    
     'start', 'readtest', 'rwtest' 和 'erase' 命令有如下可选参数：
         [&lt;partition_name1&gt; [&lt;partition_name2&gt; ...]] 分区名称
                     （例如，/fs/mtd_params），如未指定分区名称则可使用系统默认值。
    

## nshterm

Source: [systemcmds/nshterm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/nshterm)

Start an NSH shell on a given port.

This was previously used to start a shell on the USB serial port. Now there runs mavlink, and it is possible to use a shell over mavlink.

### 用法 {#nshterm_usage}

    nshterm [arguments...]
         &lt;file:dev&gt;  指定 shell 从哪个设备上运行 （例如，/dev/ttyACM0）
    

## param

Source: [systemcmds/param](https://github.com/PX4/Firmware/tree/master/src/systemcmds/param)

### 描述

Command to access and manipulate parameters via shell or script.

This is used for example in the startup script to set airframe-specific parameters.

Parameters are automatically saved when changed, eg. with `param set`. They are typically stored to FRAM or to the SD card. `param select` can be used to change the storage location for subsequent saves (this will need to be (re-)configured on every boot).

If the FLASH-based backend is enabled (which is done at compile time, e.g. for the Intel Aero or Omnibus), `param select` has no effect and the default is always the FLASH backend. However `param save/load <file>` can still be used to write to/read from files.

Each parameter has a 'used' flag, which is set when it's read during boot. It is used to only show relevant parameters to a ground control station.

### 示例

Change the airframe and make sure the airframe's default parameters are loaded:

    param set SYS_AUTOSTART 4001
    param set SYS_AUTOCONFIG 1
    reboot
    

### 用法 {#param_usage}

    param <command> [arguments...]
     Commands:
       load          Load params from a file (overwrite all)
         [<file>]    File name (use default if not given)
    
       import        Import params from a file
         [<file>]    File name (use default if not given)
    
       save          Save params to a file
         [<file>]    File name (use default if not given)
    
       select        Select default file
         [<file>]    File name (use <root>/eeprom/parameters if not given)
    
       show          Show parameter values
         [-a]        Show all parameters (not just used)
         [-c]        Show only changed and used params
         [-q]        quiet mode, print only param value (name needs to be exact)
         [<filter>]  Filter by param name (wildcard at end allowed, eg. sys_*)
    
       status        Print status of parameter system
    
       set           Set parameter to a value
         <param_name> <value> Parameter name and value to set
         [fail]      If provided, let the command fail if param is not found
    
       compare       Compare a param with a value. 如果相等则命令成功。
         &lt;param_name&gt; &lt;value&gt; 参数名称和进行对比的值
    
       greater       将一个参数与一个数值进行比较。 如果参数比该值要大则命令成功
         &lt;param_name&gt; &lt;value&gt; P参数名称和进行对比的值
    
       touch         讲一个参数表以为已使用 (used)
         [&lt;param_name1&gt; [&lt;param_name2&gt;]] 参数名称 (一个或者多个)
    
       reset         将参数重置为默认值
         [&lt;exclude1&gt; [&lt;exclude2&gt;]] 不重置相匹配的参数 (允许尾端的通配符)
    
       reset_nostart 将 SYS_AUTOSTART 和 SYS_AUTOCONFIG 之外的所有参数重置为默认值
         [&lt;exclude1&gt; [&lt;exclude2&gt;]] 不重置相匹配的参数 (允许尾端的通配符)
    
       index         显示指定索引位置的参数的值
         &lt;index&gt;     Index: 一个整数 >= 0
    
       index_used    显示指定索引位置的已使用参数的值
         &lt;index&gt;     Index: 一个整数 >= 0
    
       find          显示一个参数的索引值
         &lt;param&gt;     参数名称
    

## perf

Source: [systemcmds/perf](https://github.com/PX4/Firmware/tree/master/src/systemcmds/perf)

Tool to print performance counters

### 用法 {#perf_usage}

    perf [arguments...]
       reset         重置所有计数器
    
       latency       打印 HRT 计时器延时柱状体
    
    如未指定任何参数则打印所有计数器的性能表现。
    

## pwm

Source: [systemcmds/pwm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/pwm)

### 描述

This command is used to configure PWM outputs for servo and ESC control.

The default device `/dev/pwm_output0` are the Main channels, AUX channels are on `/dev/pwm_output1` (`-d` parameter).

It is used in the startup script to make sure the PWM parameters (`PWM_*`) are applied (or the ones provided by the airframe config if specified). `pwm info` shows the current settings (the trim value is an offset and configured with `PWM_MAIN_TRIMx` and `PWM_AUX_TRIMx`).

The disarmed value should be set such that the motors don't spin (it's also used for the kill switch), at the minimum value they should spin.

Channels are assigned to a group. Due to hardware limitations, the update rate can only be set per group. Use `pwm info` to display the groups. If the `-c` argument is used, all channels of any included group must be included.

The parameters `-p` and `-r` can be set to a parameter instead of specifying an integer: use -p p:PWM_MIN for example.

Note that in OneShot mode, the PWM range [1000, 2000] is automatically mapped to [125, 250].

### 示例

Set the PWM rate for all channels to 400 Hz:

    pwm rate -a -r 400
    

Test the outputs of eg. channels 1 and 3, and set the PWM value to 1200 us:

    pwm arm
    pwm test -c 13 -p 1200
    

### 用法 {#pwm_usage}

    pwm &lt;command&gt; [arguments...]
     Commands:
       arm           解锁模式输出
       disarm        锁定模式输出
    
       info          打印当前所有通道的设定
    
       forcefail     强制进入故障保护（Failsafe） 模式。 PWM 输出将被设置为故障保护值。
         on|off      开启或关闭
    
       terminatefail 启用 Termination Failsafe 模式。 该设定为真时所有故障保护都是不可恢复的（即便满足恢复条件）。
         on|off      开启或关闭
    
       rate          配置 PWM 速率
         -r &lt;val&gt;    PWM 速率，单位为 Hz (0 = Oneshot，否则该取值应处于 50 到 400Hz之间)
    
       oneshot       配置 Oneshot125 (速率被设为 0)
    
       failsafe      设定故障保护模式的 PWM 值
    
       disarmed      设定锁定模式 PWM 值
    
       min           设定最小 PWM 值
    
       max           设定最大 PWM 值
    
       test          将输出设定为某一特定值直到按键 'q' 或 'c' 或 'ctrl-c'
                     被按下
    
       steps         从 0 到 100% 运行 5 次阶跃
    
      'failsafe', 'disarmed', 'min', 'max' 和 'test' 命令都需要指定一个 PWM值：
         -p &lt;val&gt;    PWM 值 （例如，1100）
    
    'rate', 'oneshot', 'failsafe', 'disarmed', 'min', 'max', 'test'
     和 'steps' 命令还额外需要使用如下命令来指定进行设定的控制通道：
         [-c &lt;val&gt;]  使用如下形式进行通道的选取: 1234 (1 个数字表示一个通道，
                     1=第一个通道)
         [-m &lt;val&gt;]  使用位掩码（bitmask）选取通道 0xF, 3)
         [-g <val>]  Select channels by group (eg. （例如，0, 1, 2. use 'pwm info' to show
                     groups)
         [-a]        Select all channels
    
     These parameters apply to all commands:
         [-d <val>]  Select PWM output device
                     values: <file:dev>, default: /dev/pwm_output0
         [-v]        Verbose output
         [-e]        Exit with 1 instead of 0 on error
    

## reboot

Source: [systemcmds/reboot](https://github.com/PX4/Firmware/tree/master/src/systemcmds/reboot)

Reboot the system

### 用法 {#reboot_usage}

    reboot [arguments...]
         [-b]        重启至 bootloader
         [lock|unlock] 锁定/释放停机锁定（shutdown lock） (用于测试目的)
    

## sd_bench

Source: [systemcmds/sd_bench](https://github.com/PX4/Firmware/tree/master/src/systemcmds/sd_bench)

Test the speed of an SD Card

### 用法 {#sd_bench_usage}

    sd_bench [arguments...]
         [-b &lt;val&gt;]  每次读/写操作的块的大小
                     默认值： 4096
         [-r &lt;val&gt;]  运行次数
                     默认值： 5
         [-d &lt;val&gt;]  每次运行的持续时间，单位为 ms
                     默认值： 2000
         [-s]        完成每个块之后调用 fsync （默认值=每次运行结束时）
    

## top

Source: [systemcmds/top](https://github.com/PX4/Firmware/tree/master/src/systemcmds/top)

Monitor running processes and their CPU, stack usage, priority and state

### 用法 {#top_usage}

    top [arguments...]
       once          仅打印一次负载情况
    

## usb_connected

Source: [systemcmds/usb_connected](https://github.com/PX4/Firmware/tree/master/src/systemcmds/usb_connected)

Utility to check if USB is connected. Was previously used in startup scripts. A return value of 0 means USB is connected, 1 otherwise.

### 用法 {#usb_connected_usage}

    usb_connected [arguments...]
    

## ver

Source: [systemcmds/ver](https://github.com/PX4/Firmware/tree/master/src/systemcmds/ver)

Tool to print various version information

### 用法 {#ver_usage}

    ver &lt;command&gt; [arguments...]
     Commands:
       hw            硬件构架
    
       mcu           MCU 信息
    
       git           git 版本信息
    
       bdate         构建日期和时间
       gcc           编译器信息
    
       bdate         构建日期和时间
    
       px4guid       PX4 GUID
    
       uri           构建 URI
    
       all           打印所有版本
    
       hwcmp         比较硬件版本 (相符时返回 0)
         &lt;hw&gt; [&lt;hw2&gt;] 需要进行比较的硬件 （例如，PX4_FMU_V4）。 如果指定了多种硬件类型将执行或比较（OR comparison）
    
       hwtypecmp     比较硬件类型（匹配则返回 0 ）
         &lt;hwtype&gt; [&lt;hwtype2&gt;] 需要进行比较的硬件类型 （例如 V2） 如果指定了多种硬件类型将执行或比较（OR comparison）