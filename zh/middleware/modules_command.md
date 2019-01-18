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
         [-m &lt;val&gt;]  使用位掩码（bitmask）选取通道 （例如，0xF, 3）
                     默认值： 0
         [-a]        选择所有通道
    

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
         [-l &lt;val&gt;]  需要控制哪一个 LED： 0, 1, 2, ... (默认=all)
                     默认值：-1
         [-p &lt;val&gt;]  优先级
                     默认值：2
    

## listener

源码： [systemcmds/topic_listener](https://github.com/PX4/Firmware/tree/master/src/systemcmds/topic_listener)

用于监听 uORB 主题并将数据输出在控制台上的工具。

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

源码： [systemcmds/mixer](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mixer)

### 描述

将混控器文件加载或者附加到 ESC 驱动中。

需要注意的是驱动必须支持这个命令使用的 ioctl ，这一点在 Nuttx 上是成立的，但在其它平台上就不一定成立，如 RPI。

### 用法 {#mixer_usage}

    mixer &lt;command&gt; [arguments...]
     Commands:
       load
         &lt;file:dev&gt; &lt;file&gt; 输出装置 （例如，/dev/pwm_output0）和混控器文件
       append
         &lt;file:dev&gt; &lt;file&gt; 输出装置 （例如，/dev/pwm_output0）和混控器文件
    

## motor_ramp

源码： [systemcmds/motor_ramp](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_ramp)

### 描述

用于测试电机的加速。

在开始之前需要确保停止所有姿态控制器的运行。

    mc_att_control stop
    fw_att_control stop
    

命令开始后将开启一个后台任务，该任务会持续若干秒（根据设定值）然后退出。

Note: 该命令目前只支持 `/dev/pwm_output0` 输出。

### 示例

    motor_ramp sine 1100 0.5
    

### 用法 {#motor_ramp_usage}

    motor_ramp [arguments...]
         ramp|sine|square mode
         &lt;min_pwm&gt; &lt;time&gt; [&lt;max_pwm&gt;] pwm value in us, time in sec
    
    警告：电机将加速到最大速度！
    

## motor_test

源码： [systemcmds/motor_test](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_test)

电机测试工具。

Note: 该命令只能用于支持 motor_test uorb 主题的驱动（目前仅有 uavcan 和 tap_esc）。

### 用法 {#motor_test_usage}

    motor_test &lt;command&gt; [arguments...]
     Commands:
       test          设定电机为某一特定输出值
         [-m &lt;val&gt;]  需要进行测试的电机 (0...7, 如未指定则为所有电机)
                     默认值： -1
         [-p &lt;val&gt;]  功率 (0...100)
                     默认值： 0
    
       stop          停止所有电机
    
       iterate       遍历测试所有电机，完成一个电机的启动和停止后继续进行下一个电机的测试
    

## mtd

源码： [systemcmds/mtd](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mtd)

用于加载和测试分区的工具（由飞控板定义的 FRAM/EEPROM 存储）

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

源码： [systemcmds/nshterm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/nshterm)

在指定端口启动一个 NSH shell

该命令此前被用于在 USB 串口端口开启一个 shell， 现在它将在那个端口运行 MAVLink，而且可以通过 MAVLink 来使用一个 shell。

### 用法 {#nshterm_usage}

    nshterm [arguments...]
         &lt;file:dev&gt;  指定 shell 从哪个设备上运行 （例如，/dev/ttyACM0）
    

## param

源码： [systemcmds/param](https://github.com/PX4/Firmware/tree/master/src/systemcmds/param)

### 描述

在 shell 或者脚本中获取参数并对其进行操作的命令。

例如，在启动脚本中使用此命令来设置特定于机型的参数。

例如， 使用 `param set` 可以在对参数进行修改后自动进行保存。 这些参数通常被存储在 FRAM 或者 SD 卡中。 `param select` 可用于更改后续参数保存的存储位置（这一选项在每次启动时都需要重新进行配置）。

如果启用了基于 FLASH 的后端（例如， Intel Aero 或 Omnibus 在编译时完成该操作的相关设定 ），`param select` 不会产生任何作用，默认值将始终为 FLASH 后端。 然而，仍可使用 `param save/load &lt;file&gt;` 从文件中读取/写入参数。

每个参数都有一个 "已使用" 的标志位，如在启动过程中该参数被读取了那个该标志位将会被设置。 它只是用于向地面控制站显示有关联的参数。

### 示例

更改机型，并确保机型的默认参数被加载了：

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

源码： [systemcmds/perf](https://github.com/PX4/Firmware/tree/master/src/systemcmds/perf)

用于打印计数器性能的工具。

### 用法 {#perf_usage}

    perf [arguments...]
       reset         重置所有计数器
    
       latency       打印 HRT 计时器延时柱状体
    
    如未指定任何参数则打印所有计数器的性能表现。
    

## pwm

源码： [systemcmds/pwm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/pwm)

### 描述

此命令用于配置舵机和 ESC 的 PWM 控制输出。

默认设备是主通道的 `/dev/pwm_output0` ，AUX 辅助通道位于 `/dev/pwm_output1` (需要搭配 `-d` 参数)。

它在启动脚本中用于确保应用了 PWM 参数 (`PWM_*`) （当机型配置中指定了参数的情况下将改用由机型配置提供的参数）。 `pwm info` 用于显示当前的设定 (配平值是一个偏移量，可使用 `PWM_MAIN_TRIMx` 和 `PWM_AUX_TRIMx` 进行设置)。

锁定值（disarmed value）的设置应保证电机不会转动（该取值也被应用于 kill switch），最小值（minimum value）的设定应保证电机会转动。

通道被分配到一个组。 由于硬件限制, 只能为每个组设置更新速率。 使用 `pwm info` 显示所有的组。 如果使用了 `-c` 参数, 则参数后面必须跟上包含的分组中的所有通道。

参数 `-p` 和 `-r` 可设置为一个参数变量而不是一个指定的证书：例如， -p p:PWM_MIN 。

注意，在 OneShot 模式下， PWM 范围 [1000, 2000] 会被自动映射到 [125, 250] 。

### 示例

将所有通道的 PWM 速率设置为 400 Hz:

    pwm rate -a -r 400
    

测试 通道的输出，例如通道1和通道3，并将 PWM 值设置为 1200us：

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
         [-m &lt;val&gt;]  使用位掩码（bitmask）选取通道 （例如，0xF, 3）
                     默认值： 0
         [-g &lt;val&gt;]  使用分组进行通道的选取 （例如，0, 1, 2. 使用 'pwm info' 可以显示所有分组）
                     默认值： 0
         [-a]        选取所有通道
    
    下面的参数适用于上述所有命令：
         [-d &lt;val&gt;]  选择 PWM 输出设备
                     取值： &lt;file:dev&gt;, 默认值： /dev/pwm_output0
         [-v]        详细输出
         [-e]        遇到错误退出时返回 1 而不是 0
    

## reboot

源码： [systemcmds/reboot](https://github.com/PX4/Firmware/tree/master/src/systemcmds/reboot)

重启系统

### 用法 {#reboot_usage}

    reboot [arguments...]
         [-b]        重启至 bootloader
         [lock|unlock] 锁定/释放停机锁定（shutdown lock） (用于测试目的)
    

## sd_bench

源码： [systemcmds/sd_bench](https://github.com/PX4/Firmware/tree/master/src/systemcmds/sd_bench)

测试 SD 卡的速度

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

源码：[systemcmds/top](https://github.com/PX4/Firmware/tree/master/src/systemcmds/top)

监控运行的进程机器 CPU、栈堆使用情况和优先级、运行状态。

### 用法 {#top_usage}

    top [arguments...]
       once          仅打印一次负载情况
    

## usb_connected

源码： [systemcmds/usb_connected](https://github.com/PX4/Firmware/tree/master/src/systemcmds/usb_connected)

检查 USB 是否已连接的工具。 此前曾在启动脚本中使用过， 返回值为 0 表示 USB 已连接，否则返回 1 。

### 用法 {#usb_connected_usage}

    usb_connected [arguments...]
    

## ver

源码： [systemcmds/ver](https://github.com/PX4/Firmware/tree/master/src/systemcmds/ver)

用于打印各种版本信息的工具。

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