# 模块参考：驱动

## batt_smbus

源码：[drivers/batt_smbus](https://github.com/PX4/Firmware/tree/master/src/drivers/batt_smbus)

### 描述

针对 BQ40Z50 电池管理 IC 的智能电池驱动。

### 示例

写入内存以对参数进行设置， 地址，字节数，byte0, ... , byteN：

    batt_smbus -X write_flash 19069 2 27 0
    

### 用法 {#batt_smbus_usage}

    batt_smbus &lt;command&gt; [arguments...]
     Commands:
       start
         [-X &lt;val&gt;]  ullpt
                     默认值：BATT_SMBUS_BUS_I2C_EXTERNAL
         [-T &lt;val&gt;]  ullpt
                     默认值： BATT_SMBUS_BUS_I2C_EXTERNAL1
         [-R &lt;val&gt;]  ullpt
                     默认值： BATT_SMBUS_BUS_I2C_EXTERNAL2
         [-I &lt;val&gt;]  ullpt
                     默认值： BATT_SMBUS_BUS_I2C_INTERNAL
         [-A &lt;val&gt;]  ullpt
                     默认值： BATT_SMBUS_BUS_ALL
    
       man_info      打印制造商信息
    
       report        打印最近一次的报告。
    
       unseal        解锁设备内存以启用 write_flash 命令
    
       seal          锁定设备内存以使 write_flash 命令无效。
    
       suspend       将驱动从重新安排循环挂起。
    
       resume        将驱动从挂起中恢复。
    
       write_flash   想内存中写入。 运行此命令前必须使用 unseal 命令解除内存设备的锁定。
         [address]   开始写入的地址
         [number of bytes] 需要写入的字节数。
         [data[0]...data[n]] 数据的各个字节，以空格分开。
    

## fmu

源码：[drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)

### 描述

该模快用于操纵输出和读取输入针脚。 对于没有单独 IO 芯片的飞控板 （例如，Pixracer），它将使用主通道。 对于哪些有单独 IO 芯片的飞控板 （例如，Pixhawk），该模块将使用辅助（AUX）通道，主通道使用 px4io 驱动。

该模块监听 actuator_controls 主题，执行混控并写入 PWM 输出。

该模块使用 mode_* 命令进行配置。 该命令会设定驱动将占用最开始的哪些 N 个针脚。 例如，通过使用 mode_pwm4，引脚 5 和 6 可被分别被相机触发驱动或者 PWM 测距仪驱动使用。 此外， fmu 还可以以某种捕获模式启动，然后驱动可以使用 ioctl 调用注册一个捕获回调函数（callback）。

### 实现

默认情况下模块以工作队列的形式运行以降低内存占用。 它也可以在它自己独有的线程中运行以降低延时，使用 -t 启动标志进行指定即可。 以工作队列的形式运行时，该模块将以一个固定的频率进行调度，且 PWM 速率会限制着 actuator_controls 主题的更新速率。 以自己独有线程形式运行时，该模块会轮询 actuator_controls 主题。 此外， PWM 速率会影响更低级别的的 IO 计时器的速率。

### 示例

通常使用如下命令：

    fmu mode_pwm
    

来驱动所有可以的引脚。

捕获输入（上升沿和下降沿）并在控制台上打印出来：以一种捕获模式启动 fmu：

    fmu mode_pwm3cap1
    

该命令将启用第 4 引脚上的捕获。 然后执行：

    fmu test
    

使用 `pwm` 命令进行进一步的配置 (PWM 速率，级别, ...)，然后使用 `mixer` 命令来加载混控器文件。

### 用法 {#fmu_usage}

    fmu &lt;command&gt; [arguments...]
     Commands:
       start         启动任务 (不执行任何模式设置，不使用任何 mode_* 命令)
         [-t]        以单独任务的形式运行，而不是工作队列
    
     所有 mode_* 命令都启动 fum，如果 fmu 处于非运行状态的话
    
       mode_gpio
    
       mode_pwm      选择所有可用的针脚为 PWM
    
       mode_pwm8
    
       mode_pwm6
    
       mode_pwm5
    
       mode_pwm5cap1
    
       mode_pwm4
    
       mode_pwm4cap1
    
       mode_pwm3
    
       mode_pwm3cap1
    
       mode_pwm2
    
       mode_pwm2cap2
    
       mode_pwm1
    
       sensor_reset  执行一次传感器重置 (SPI 总线)
         [&lt;ms&gt;]      重置和重新启用之间的延迟时间，单位 ms
    
       peripheral_reset 重置飞控板外围设备
         [&lt;ms&gt;]      重置和重新启用之间的延迟时间，单位 ms
    
       i2c           配置 I2C 时钟速率（clock rate）
         &lt;bus_id&gt; &lt;rate&gt; 指定总线 id (>=0) 和速率，单位 Hz
    
       test          测试输入和输出
    
       fake          解锁并发送一个执行器控制信号
         &lt;roll&gt; &lt;pitch&gt; &lt;yaw&gt; &lt;thrust&gt; 位于区间 [-100, 100] 内的控制值
    
       stop
    
       status        打印状态信息
    

## gps

源码：[drivers/gps](https://github.com/PX4/Firmware/tree/master/src/drivers/gps)

### 描述

GPS 驱动模块负责处理与设备的通信并且将位置信息通过 uORB 发布出去。 它支持多个协议 (设备供应商)，默认情况下会自动选择正确的协议。

模块支持一个辅助（secondary） GPS 设备，可使用 `-e` 参数进行指定。 辅助 GPS 的位置信息会在第二个 uORB 主题实例上发布，但目前为止系统的其它部分暂未使用该数据（但该数据会被记录下来，以方便进行对比）。

### 实现

每个设备都有一个线程轮询数据。 GPS 协议类是通过回调来实现的，这使得可以在其它项目中使用该协议类 （例如，QGroundControl 也是用了 GPS 协议类）。

### 示例

进行测试时能提供虚假的 GPS 信号是非常有用的（它可以告知系统当前已经获得了一个有效的位置）。

    gps stop
    gps start -f
    

启动 2 个 GPS 设备 (主 GPS 在 /dev/ttyS3 ，辅助 GPS 在 /dev/ttyS4)： gps start -d /dev/ttyS3 -e /dev/ttyS4

### 用法 {#gps_usage}

    gps &lt;command&gt; [arguments...]
     Commands:
       start
         [-d &lt;val&gt;]  GPS 设别
                     取值： &lt;file:dev&gt;, 默认值： /dev/ttyS3
         [-b &lt;val&gt;]  波特率 (也可设为 p:&lt;param_name&gt;)
                     默认值： 0
         [-e &lt;val&gt;]  可选的辅助 GPS 设备
                     取值：&lt;file:dev&gt;
         [-g &lt;val&gt;]  波特率 (辅助 GPS, 也可设为 p:&lt;param_name&gt;)
                     默认值： 0
         [-f]        虚拟 GPS 信号 (对进行测试非常有用)
         [-s]        开启公布卫星信息
         [-i &lt;val&gt;]  GPS 接口
                     取值： spi|uart, 默认值： uart
         [-p &lt;val&gt;]  GPS 协议 (默认值=auto select)
                     取值： ubx|mtk|ash
    
       stop
    
       status        打印状态信息
    

## pga460

源码：[drivers/distance_sensor/pga460](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/pga460)

### 描述

超声笔测距仪驱动，负责处理与设备的用心并通过 uORB 将距离信息发布出去。

### 实现

此驱动以 NuttX 任务的形式实现。 选择这个实现方式是阴虚需要通过 UART 对消息进行轮询，而工作队列并不支持这一操作。 驱动在运行时将持续获取测距仪的测量值。 应用了一个简单的检测错误读数的算法以发布出去的数据的质量， 若驱动认为传感器数据无效或者不稳定，那么驱动将不会将数据发布出去。

### 用法 {#pga460_usage}

    pga460 &lt;command&gt; [arguments...]
     Commands:
       start &lt;device_path&gt;
         [device_path] pga460 传感器设备地址 (例如： /dev/ttyS6
    
       status
    
       stop
    
       help
    

## pwm_out_sim

源码：[drivers/pwm_out_sim](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out_sim)

### 描述

针对仿真模拟的 PWM 输出的驱动。

该模块唯一的功能是取 `actuator_control` uORB 消息作为输入，根据任意已经加载了的混控器对输入进行混合然后将结果输出至 `actuator_output` uORB 主题。

该模块在 SITL 和 HITL 仿真中使用。

### 用法 {#pwm_out_sim_usage}

    pwm_out_sim &lt;command&gt; [arguments...]
     Commands:
       start         以 mode_pwm16 模式运行此任务
    
     以下所有 mode_* 命令都会启动 pwm sim ，如果它不处于运行状态的话
    
       mode_pwm      使用 8 个 PWM 输出
    
       mode_pwm16    使用 16 个 PWM 输出
    
       stop
    
       status        打印输出信息
    

## rc_input

源码：[drivers/rc_input](https://github.com/PX4/Firmware/tree/master/src/drivers/rc_input)

### 描述

本模块自动选择合适的方法对 RC 输入进行解析， 受支持的方法有：

- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS Crossfire (CRSF)

### 实现

默认情况下模块以工作队列的形式运行以降低内存占用。 它也可以在它自己独有的线程中运行以降低延时，使用 -t 启动标志进行指定即可。 以工作队列的方式运行时，它将以固定的频率进行调度。

### 用法 {#rc_input_usage}

    rc_input &lt;command&gt; [arguments...]
     Commands:
       start         启动任务 (不执行任何模式设置，不使用任何 mode_* 命令)
         [-t]        以单独任务的形式运行，而不是工作队列
    
       bind          发送一个 DSM 绑定指令 (模块必须处于运行状态)
    
       stop
    
       status        打印状态信息
    

## sf1xx

源码：[drivers/distance_sensor/sf1xx](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/sf1xx)

### 描述

针对 Lightware SFxx 系列 LIDAR 测距仪的 I2C 总线驱动： SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20。

设置/使用 信息： https://docs.px4.io/en/sensor/sfxx_lidar.html

### 示例

尝试在任意总线上启动驱动（在第一个发现传感器的总线上启动）。

    sf1xx start -a
    

停止驱动程序的运行

    sf1xx stop
    

### 用法 {#sf1xx_usage}

    sf1xx &lt;command&gt; [arguments...]
     Commands:
       start         启动驱动
         [-a]        尝试在所有 I2C 总线上启动驱动
         [-b &lt;val&gt;]  在指定 I2C 总线上启动驱动
                     默认值： 1
         [-R &lt;val&gt;]  传感器朝向 - 默认为朝下（downward facing）
                     默认值： 25
    
       stop          停止驱动
    
       test          测试驱动 (基础功能测试)
    
       reset         重置驱动
    
       info          打印驱动信息
    

## tap_esc

源码：[drivers/tap_esc](https://github.com/PX4/Firmware/tree/master/src/drivers/tap_esc)

### 描述

本模块通过 UART 控制 TAP_ESC 硬件。 它监听着 actuator_controls主题，执行混控并写入 PWM 输出。

### 实现

目前，该模块仅以线程方式进行了实现，这就意味着它在自己的线程中运行，而不是在工作队列中运行。

### 示例

模块命令通常以如下内容作为开头： tap_esc start -d /dev/ttyS2 -n <1-8>

### 用法 {#tap_esc_usage}

    tap_esc &lt;command&gt; [arguments...]
     Commands:
       start         启动任务
         [-d &lt;val&gt;]  用于与 ESC 进行沟通的设备
                     取值： &lt;device&gt;
         [-n &lt;val&gt;]  ESC 数量
                     默认值： 4
    

## vmount

源码：[modules/vmount](https://github.com/PX4/Firmware/tree/master/src/modules/vmount)

### 描述

载荷（云台）控制驱动， 该模块将多种不同的输入手段（例如， RC 信号和 MAVLink 信号）映射到 一个配置好的输出端口上 （例如，辅助 AUX 通道或者 MAVLink）。

该模块的使用方式在[gimbal_control](https://dev.px4.io/en/advanced/gimbal_control.html) 页面有记载。

### 实现

除了一个针对输入和输出的通用基类外，每个方法都是通过各自独有的类实现的。 各个类通过一个由 `ControlData` 数据结构定义的 API 实现相互的连接。 此举确保了每一种输入方法都可以在每一个输出方法中使用，且只需要很少的工作量就可以添加一个新的输入/输出。

### 示例

通过设定一个固定的偏航角来测试输出功能（其它轴的角度设为0）：

    vmount stop
    vmount test yaw 30
    

### 用法 {#vmount_usage}

    vmount &lt;command&gt; [arguments...]
     Commands:
       start
    
       test          测试输出：将某个轴的角度设定为一个定值 (vmount 不能处于运行状态)
         roll|pitch|yaw &lt;angle&gt; 指定一个旋转轴和旋转角度，单位 °
    
       stop
    
       status        打印状态信息