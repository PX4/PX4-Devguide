# 模块参考：命令
## bl_update
源代码: [systemcmds/bl_update](https://github.com/PX4/Firmware/tree/master/src/systemcmds/bl_update)

从文件烧写引导程序的实用程序
### 用法
```
bl_update [参数...]
   setopt        设置可选位来解锁FLASH（仅在锁定状态时需要）

   <file>        引导程序的二进制文件
```
## config
源代码: [systemcmds/config](https://github.com/PX4/Firmware/tree/master/src/systemcmds/config)

配置传感器驱动器（采样率、发布频率以及范围等等）
### 用法
```
config <命令> [参数...]
 命令:

<file:dev> 参数通常是/dev/{gyro,accel,mag}i之一
   block         锁定传感器主题发布
     <file:dev>  传感器驱动文件

   unblock       解锁传感器主题发布
     <file:dev>  传感器驱动文件

   sampling      设置传感器采样率
     <file:dev> <rate> 传感器驱动文件和采样频率，单位Hz

   rate          设置传感器发布频率
     <file:dev> <rate> 传感器驱动文件和发布频率，单位Hz

   range         设置传感器测量范围
     <file:dev> <rate> 传感器驱动文件和范围

   check         执行传感器自检（并打印信息）
     <file:dev>  传感器驱动文件
```
## dumpfile
源代码: [systemcmds/dumpfile](https://github.com/PX4/Firmware/tree/master/src/systemcmds/dumpfile)

转储文件实用程序。 以二进制模式打印文件大小和内容（不要用CR LF替换LF）到stdout。
### 用法
```
dumpfile [参数...]
     <file>      转储文件
```
## esc_calib
源代码: [systemcmds/esc_calib](https://github.com/PX4/Firmware/tree/master/src/systemcmds/esc_calib)

电调校准工具

校准流程（运行此命令将引导完成）：
- 移除螺旋桨，电调断电
- 关闭姿态控制器：mc_att_control stop，fw_att_control stop
- 确保安全开关关闭
- 运行此命令

### 用法
```
esc_calib [参数...]
     [-d <val>]  选择PWM输出设备
                 可选: <file:dev>, 缺省: /dev/pwm_output0
     [-l <val>]  最低PWM值，单位，us
                 缺省: 1000
     [-h <val>]  最高PWM值，单位，us
                 缺省: 2000
     [-c <val>]  以这种形式选择通道：1234 (每一位代表一个通道，1为第一通道)
     [-m <val>]  通过位掩码的形式选择通道(例如，0xF为第三通道))
                 缺省: 0
     [-a]        选择所有通道
```
## hardfault_log
源代码: [systemcmds/hardfault_log](https://github.com/PX4/Firmware/tree/master/src/systemcmds/hardfault_log)

硬故障实用程序 

在启动脚本中使用来处理硬故障

### 用法
```
hardfault_log <命令> [参数...]
 命令:
   check         检查是否存在未提交的硬故障

   rearm         抛掉一个未提交的硬故障

   fault         抛出一个硬故障 (这个命令会让系统崩溃：)
     [0|1]       硬故障类型: 0=除零, 1=断言 (缺省值为0)

   commit        将未提交的硬故障写入/fs/microsd/fault_%i.txt
                 (并执行rearm，但不会执行reset)

   count         读取重启计数器，统计一个未提交的硬故障的重启次数
                （作为程序的退出码返回）

   reset         重置重启计数器
```
## led_control
源代码: [systemcmds/led_control](https://github.com/PX4/Firmware/tree/master/src/systemcmds/led_control)

### 说明
控制和测试（外部）LED的命令行工具。

要使用它，请确保有一个处理led_control uorb主题的驱动程序运行。

可以设置不同的优先级，例如一个模块可以以低优先级设置颜色，另一个模块可以以高优先级闪烁N次，那么LED将在闪烁后自动返回到较低优先级的状态。 `reset`命令也可以用于返回较低的优先级。

### 示例
第一个LED以蓝色闪烁5次
```
led_control blink -c blue -l 0 -n 5
```


### 用法
```
led_control <命令> [参数...]
 命令:
   test          运行测试模式

   on            点亮LED

   off           熄灭LED

   reset         重置LED的优先级

   blink         闪烁LED
     [-n <val>]  闪烁次数
                 缺省: 3
     [-s <val>]  设置闪烁速度
                 可选: fast|normal|slow, 默认: normal

   breathe       持续地淡入淡出（呼吸灯）

 以下参数适用于除'test'之外的所有上述命令：
     [-c <val>]  颜色
                 可选: red|blue|green|yellow|purple|amber|cyan|white
                 缺省: white
     [-l <val>]  指定控制的LED: 0, 1, 2, ... 
                 缺省: -1(所有)
     [-p <val>]  优先级
                 缺省: 2
```
## listener
源代码: [systemcmds/topic_listener](https://github.com/PX4/Firmware/tree/master/src/systemcmds/topic_listener)

用于监听uORB主题并将数据打印到控制台的实用程序。

限制：只能监听一个主题的第一个实例。


### 用法
```
listener [参数...]
     <topic_name> [<num_msgs>] uORB主题名称以及消息数目(可选，缺省为1)
```
## mixer
源代码: [systemcmds/mixer](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mixer)

### 说明
加载或附加混控器文件到ESC驱动程序。

注意，驱动程序必须支持所用的ioctl函数，例如NuttX支持，但RPi不支持。
### 用法
```
mixer <命令> [参数...]
 命令:
   load
     <file:dev> <file> 输出设备(例如：/dev/pwm_output0)和混控器文件

   append
     <file:dev> <file> 输出设备(例如：/dev/pwm_output0)和混控器文件
```
## motor_ramp
源代码: [systemcmds/motor_ramp](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_ramp)


### 说明
测试电机加速的应用

开始前，请务必关闭所有运行中的姿态控制器：
```
mc_att_control stop
fw_att_control stop
```

启动时，作为后台任务启动，运行指定时间，然后退出。

注意：此命令目前只支持`/dev/pwm_output0`输出。

### 示例
```
motor_ramp sine 1100 0.5
```

### 用法
```
motor_ramp [参数...]
     ramp|sine|square 模式
     <min_pwm> <time> [<max_pwm>] pwm值单位为us，时间单位为sec

 警告：电机将加速到全速！
```
## motor_test
源代码: [systemcmds/motor_test](https://github.com/PX4/Firmware/tree/master/src/systemcmds/motor_test)

测试电机的实用程序

注意：这只能用于支持motor_test uorb主题的驱动程序（目前有uavcan和tap_esc）
### 用法
```
motor_test <命令> [参数...]
 命令:
   test          设置电机到指定输出值
     [-m <val>]  待测试电机(0...7，如不指定，则为全部)
                 缺省: -1
     [-p <val>]  以百分比设置输出值 (0...100)
                 缺省: 0

   stop          停止所有电机

   iterate       依次启动并停止所有电机
```
## mtd
源代码: [systemcmds/mtd](https://github.com/PX4/Firmware/tree/master/src/systemcmds/mtd)

挂载并测试分区（基于飞控板定义的FRAM / EEPROM存储）的实用程序
### 用法
```
mtd <命令> [参数...]
 命令:
   status        打印状态信息

   start         挂载分区

   readtest      执行读取测试

   rwtest        执行读写测试

   erase         擦除分区

 命令 'start'、'readtest'、'rwtest'和'erase'有可选参数： 
     [<partition_name1> [<partition_name2> ...]] 
     分区名称 (例如，/fs/mtd_params), 如果未指定的话，该参数为系统默认值
```
## nshterm
源代码: [systemcmds/nshterm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/nshterm)

在给定端口上启动NSH shell。

以前用于在USB串行端口上启动shell，现在运行mavlink，可以在mavlink上使用shell。
### 用法
```
nshterm [参数...]
     <file:dev>  运行shell的设备(例如，/dev/ttyACM0)
```
## param
源代码: [systemcmds/param](https://github.com/PX4/Firmware/tree/master/src/systemcmds/param)

### 说明
通过shell或脚本访问和操作参数的命令。

例如，用于在启动脚本中设置与机型相关的参数。

当参数更改时可以自动保存，例如，`param set`。参数通常存储在FRAM或SD卡中。 `param select`可以用于更改存储位置以供后续保存（需要在每次启动时重新配置）。

每个参数都有一个'used'标志，它在引导过程中被读取时设置。 它只用于向地面控制站显示相关参数。

### 示例
更改机型并确保机型的默认参数被加载：
```
param set SYS_AUTOSTART 4001
param set SYS_AUTOCONFIG 1
reboot
```

### 用法
```
param <命令> [参数...]
 命令:
   load          从文件中载入参数（覆盖所有）
     [<file>]    文件名 (如未给出，则使用缺省值)

   import        从文件中导入参数
     [<file>]    文件名 (如未给出，则使用缺省值)

   save          保存参数到文件
     [<file>]    文件名 (如未给出，则使用缺省值)

   select        选择缺省文件
     [<file>]    文件名 (如未给出，则使用<root>/eeprom/parameters)

   show          显示参数值
     [-c]        仅显示有变动的参数
     [<filter>]  参数名过滤器(允许在参数名的最后使用通配符，例如sys_*)

   set           设置参数
     <param_name> <value> 参数名和参数值
     [fail]      如果设置了该参数，那么如果没有找到参数，则命令失败

   compare       比较参数，如果相等，则命令成功
     <param_name> <value> 参数名和待比较的值

   greater       比较参数，如果参数大于待比较值，则命令成功
     <param_name> <value> 参数名和待比较的值

   reset         重置参数到缺省值
     [<exclude1> [<exclude2>]] 不重置匹配到的参数(允许在参数名的最后使用通配符)

   reset_nostart 重置除过SYS_AUTOSTART和SYS_AUTOCONFIG以外的参数到缺省值 
     [<exclude1> [<exclude2>]] 不重置匹配到的参数(允许在参数名的最后使用通配符)

   index         显示给定序号的参数
     <index>     序号: 大于等于0的整数

   index_used    显示给定序号的已用参数
     <index>     序号: 大于等于0的整数

   find          显示给定参数的序号
     <param>     参数名
```
## perf
源代码: [systemcmds/perf](https://github.com/PX4/Firmware/tree/master/src/systemcmds/perf)

打印性能计数器的工具
### 用法
```
perf [参数...]
   reset         重置所有计数器

   latency       打印HRT定时器延迟直方图

 如果没有给出，则打印所有性能计数器
```
## pwm
源代码: [systemcmds/pwm](https://github.com/PX4/Firmware/tree/master/src/systemcmds/pwm)

### 说明
该命令用于配置舵机和电调的PWM输出。

默认设备`/dev/pwm_output0`是主通道，辅助通道为`/dev/pwm_output1`（`-d`参数指定）。

在启动脚本中使用来确保PWM参数(`PWM_*`)被正确配置（如果指定机型配置，则由机型配置文件提供PWM参数）。

 `pwm info`显示当前设置（修正值是一个偏移量并由`PWM_MAIN_TRIMx`和`PWM_AUX_TRIMx`配置）。

应该设置锁定值，使得电机在最小值时不会旋转（同样用于终止开关）。

通道被分配成组。由于硬件限制，更新速率只能以组为单位设置。使用`pwm info`显示组。如果使用`-c`参数，则必须包括该组的所有通道。

可以将参数`-p`和`-r`设置为参数，而不是指定一个整数：例如，使用-p p:PWM_MIN。

注意，在OneShot模式下，PWM范围[1000, 2000]自动映射到[125, 250]。

### 示例
设置所有通道的PWM速率为400Hz：
```
pwm rate -a -r 400
```

测试例如通道1和通道3的输出，将PWM值设置为1200 us：
```
pwm arm
pwm test -c 13 -p 1200
```


### 用法
```
pwm <命令> [参数...]
 命令:
   arm           解锁输出

   disarm        锁定输出

   info          打印所有通道的当前配置信息

   forcefail     强制切换Failsafe模式
     on|off      打开|关闭

   terminatefail 强制切换Termination Failsafe模式
     on|off      打开|关闭

   rate          配置PWM速率
     -r <val>    PWM速率单位为Hz(0为Oneshot模式，其它50到400Hz)

   oneshot       配置Oneshot125 (速率设置为0)

   failsafe      设置Failsafe模式的PWM值

   disarmed      设置锁定PWM值

   min           设置最小PWM值

   max           设置最大PWM值

   test          设置输出为指定值直到'q'或'c'或'ctrl-c'被按下

   steps         从0到100%分5步运行

 命令'failsafe'、'disarmed'、'min'、'max'和'test'需要指定PWM值：
     -p <val>    PWM值 (例如1100)

 命令'rate'、'oneshot'、'failsafe'、'disarmed'、'min'、'max'、'test'和'steps'
 需要额外以下列参数中的一个指定通道：
     [-c <val>]  以这种形式选择通道：1234 (每一位代表一个通道，
                 1代表第一个通道)
     [-m <val>]  以位掩码的形式选择通道(例如，0xF, 3)
                 缺省: 0
     [-g <val>]  通过组来选择通道(例如，0, 1, 2. 使用'pwm info'显示组)
                 缺省: 0
     [-a]        选择所有通道

 这些参数可用于所有命令：
     [-d <val>]  选择PWM输出设备
                 可选: <file:dev>, 缺省: /dev/pwm_output0
     [-v]        详细输出
     [-e]        发生错误时以1退出，而不是0
```
## reboot
源代码: [systemcmds/reboot](https://github.com/PX4/Firmware/tree/master/src/systemcmds/reboot)

重启系统
### 用法
```
reboot [参数...]
     [-b]        重启进入引导程序
```
## sd_bench
源代码: [systemcmds/sd_bench](https://github.com/PX4/Firmware/tree/master/src/systemcmds/sd_bench)

测试SD卡的速度
### 用法
```
sd_bench [参数...]
     [-b <val>]  一次读写的块大小
                 缺省: 4096
     [-r <val>]  运行次数
                 缺省: 5
     [-d <val>]  运行持续时间，单位：ms
                 缺省: 2000
     [-s]        每块运行结束后调用fsync(缺省，每次运行完成后调用)
```
## top
源代码: [systemcmds/top](https://github.com/PX4/Firmware/tree/master/src/systemcmds/top)

监控运行进程及其CPU占用，堆栈使用情况，优先级和状态
### 用法
```
top [参数...]
   once          仅仅打印一次
```
## usb_connected
源代码: [systemcmds/usb_connected](https://github.com/PX4/Firmware/tree/master/src/systemcmds/usb_connected)

检查USB是否连接的实用程序。以前在启动脚本中使用。 返回0表示USB已连接，否则为1。
### 用法
```
usb_connected [参数...]
```
## ver
源代码: [systemcmds/ver](https://github.com/PX4/Firmware/tree/master/src/systemcmds/ver)

打印各种版本信息的工具
### 用法
```
ver <命令> [参数...]
 命令:
   hw            硬件架构

   mcu           MCU信息

   git           git版本信息

   bdate         构建日期和时间

   gcc           编译器信息

   uid           UUID

   mfguid        制造商UUID

   uri           构建URI

   all           打印所有版本信息

   hwcmp         比较硬件版本(匹配的话返回0)
     <hw>        待比较的硬件(例如，PX4FMU_V4)
```
