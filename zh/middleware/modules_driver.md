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

Ultrasonic range finder driver that handles the communication with the device and publishes the distance via uORB.

### 实现

This driver is implented as a NuttX task. This Implementation was chosen due to the need for polling on a message via UART, which is not supported in the work_queue. This driver continuously takes range measurements while it is running. A simple algorithm to detect false readings is implemented at the driver levelin an attemptto improve the quality of data that is being published. The driver will not publish data at all if it deems the sensor data to be invalid or unstable.

### 用法 {#pga460_usage}

    pga460 <command> [arguments...]
     Commands:
       start <device_path>
         [device_path] The pga460 sensor device path, (e.g: /dev/ttyS6
    
       status
    
       stop
    
       help
    

## pwm_out_sim

源码：[drivers/pwm_out_sim](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out_sim)

### 描述

Driver for simulated PWM outputs.

Its only function is to take `actuator_control` uORB messages, mix them with any loaded mixer and output the result to the `actuator_output` uORB topic.

It is used in SITL and HITL.

### 用法 {#pwm_out_sim_usage}

    pwm_out_sim <command> [arguments...]
     Commands:
       start         Start the task in mode_pwm16
    
     All of the mode_* commands will start the pwm sim if not running already
    
       mode_pwm      use 8 PWM outputs
    
       mode_pwm16    use 16 PWM outputs
    
       stop
    
       status        print status info
    

## rc_input

源码：[drivers/rc_input](https://github.com/PX4/Firmware/tree/master/src/drivers/rc_input)

### 描述

This module does the RC input parsing and auto-selecting the method. Supported methods are:

- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS Crossfire (CRSF)

### 实现

By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread, specified via start flag -t, to reduce latency. When running on the work queue, it schedules at a fixed frequency.

### 用法 {#rc_input_usage}

    rc_input <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
         [-t]        Run as separate task instead of the work queue
    
       bind          Send a DSM bind command (module must be running)
    
       stop
    
       status        print status info
    

## sf1xx

源码：[drivers/distance_sensor/sf1xx](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/sf1xx)

### 描述

I2C bus driver for Lightware SFxx series LIDAR rangefinders: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/en/sensor/sfxx_lidar.html

### 示例

Attempt to start driver on any bus (start on bus where first sensor found).

    sf1xx start -a
    

Stop driver

    sf1xx stop
    

### 用法 {#sf1xx_usage}

    sf1xx <command> [arguments...]
     Commands:
       start         Start driver
         [-a]        Attempt to start driver on all I2C buses
         [-b <val>]  Start driver on specific I2C bus
                     default: 1
         [-R <val>]  Sensor rotation - downward facing by default
                     default: 25
    
       stop          Stop driver
    
       test          Test driver (basic functional tests)
    
       reset         Reset driver
    
       info          Print driver information
    

## tap_esc

源码：[drivers/tap_esc](https://github.com/PX4/Firmware/tree/master/src/drivers/tap_esc)

### 描述

This module controls the TAP_ESC hardware via UART. It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### 实现

Currently the module is implementd as a threaded version only, meaning that it runs in its own thread instead of on the work queue.

### 示例

The module is typically started with: tap_esc start -d /dev/ttyS2 -n <1-8>

### 用法 {#tap_esc_usage}

    tap_esc <command> [arguments...]
     Commands:
       start         Start the task
         [-d <val>]  Device used to talk to ESCs
                     values: <device>
         [-n <val>]  Number of ESCs
                     default: 4
    

## vmount

源码：[modules/vmount](https://github.com/PX4/Firmware/tree/master/src/modules/vmount)

### 描述

Mount (Gimbal) control driver. It maps several different input methods (eg. RC or MAVLink) to a configured output (eg. AUX channels or MAVLink).

Documentation how to use it is on the [gimbal_control](https://dev.px4.io/en/advanced/gimbal_control.html) page.

### 实现

Each method is implemented in its own class, and there is a common base class for inputs and outputs. They are connected via an API, defined by the `ControlData` data structure. This makes sure that each input method can be used with each output method and new inputs/outputs can be added with minimal effort.

### 示例

Test the output by setting a fixed yaw angle (and the other axes to 0):

    vmount stop
    vmount test yaw 30
    

### 用法 {#vmount_usage}

    vmount <command> [arguments...]
     Commands:
       start
    
       test          Test the output: set a fixed angle for one axis (vmount must
                     not be running)
         roll|pitch|yaw <angle> Specify an axis and an angle in degrees
    
       stop
    
       status        print status info