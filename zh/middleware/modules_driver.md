# 模块参考：驱动

子分类

- [距离传感器](modules_driver_distance_sensor.md)

## atxxxx

源码位置: [drivers/osd/atxxxx](https://github.com/PX4/Firmware/tree/master/src/drivers/osd/atxxxx)

### 描述

例如，安装在OmnibusF4SD板子上的用于OSD驱动的ATXXXX芯片

他可以通过OSD_ATXXXX_CFG这个参数来使能

### 用法 {#atxxxx_usage}

    atxxxx <command> [arguments...]
     Commands:
       start         Start the driver
         [-b <val>]  SPI bus (default: use board-specific bus)
    
       stop
    
       status        print status info
    

## batt_smbus

源码位置: [drivers/batt_smbus](https://github.com/PX4/Firmware/tree/master/src/drivers/batt_smbus)

### 描述

用于智能电池的BQ40Z50电量统计芯片

### 示例

通过写入flash来设置它的参数。 地址，字节数，字节0，... ，字节N

    batt_smbus -X write_flash 19069 2 27 0
    

### 用法 {#batt_smbus_usage}

    batt_smbus <命令> [参数...]
     命令列表:
       start
         [-X]        BATT_SMBUS_BUS_I2C_EXTERNAL
         [-T]        BATT_SMBUS_BUS_I2C_EXTERNAL1
         [-R]        BATT_SMBUS_BUS_I2C_EXTERNAL2
         [-I]        BATT_SMBUS_BUS_I2C_INTERNAL
         [-A]        BATT_SMBUS_BUS_ALL
    
       man_info      打印厂商信息
    
       unseal        解锁设备的flash来使能 write_flash 命令
    
       seal          锁住设备的flash来失能 write_flash 命令.
    
       suspend       从调度循环中挂起该设备
    
       resume        将该设备从挂起状态恢复
    
       write_flash   写入flash。 必须先通过unseal 命令来解锁flash。
         [address]   写入的起始地址
         [number of bytes] 需要写入的字节数
         [data[0]...data[n]] 具体的字节数据，使用空格隔开
    
       stop          停止设备
    
       status        打印状态信息
    

## dshot

源码位置: [drivers/dshot](https://github.com/PX4/Firmware/tree/master/src/drivers/dshot)

### 描述

这是DShot输出的驱动。 它跟fmu的驱动很相似，可以简单地替换掉，来实现使用DShot与调速器通讯而不是PWM。

它支持：

- DShot150, DShot300, DShot600, DShot1200
- 通过独立的串口遥控，并且发布esc_status消息
- 通过命令行接口发送 DShot 命令

### 示例

设置电机1永久反向：

    dshot reverse -m 1
    dshot save -m 1
    

保存之后，设置的反向之后的转向将被认为是正常时候的转向， 所以如果需要再次反转方向只需要再次重复相同的命令。

### 用法 {#dshot_usage}

    dshot <命令> [参数...]
     命令列表:
       start         启动任务 (不带任何模式集, 使用mode_*类的命令)，所有的 mode_*类的命令都会启动这个模块，如果这个模块还没有启动的话
    
       mode_gpio
    
       mode_pwm      选择所有可能的PWM引脚
    
       mode_pwm8
    
       mode_pwm6
    
       mode_pwm5
    
       mode_pwm5cap1
    
       mode_pwm4
    
       mode_pwm4cap1
    
       mode_pwm4cap2
    
       mode_pwm3
    
       mode_pwm3cap1
    
       mode_pwm2
    
       mode_pwm2cap2
    
       mode_pwm1
    
       telemetry     在某个串口上使能遥控功能
         <device>    UART 设备节点
    
       reverse       反转马达方向
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       normal        转到默认的方向
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       save          保存当前的设置
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       3d_on         使能3D模式
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       3d_off        关闭3D模式
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       beep1         发送蜂鸣模式 1
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       beep2         发送蜂鸣模式 2
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       beep3         发送蜂鸣模式 3
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       beep4         发送蜂鸣模式 4
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       beep5         发送蜂鸣模式 5
         [-m <val>]  马达的编号 (从1开始，默认全部)
    
       esc_info      请求马达的信息
         -m <val>    马达的编号 (从1开始)
    
       stop
    
       status        打印状态信息
    

## fmu

源码位置: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)

### 描述

该模块负责驱动输出引脚或者读取输入引脚。 对于没有独立 IO 芯片的飞控板（例如 Pixracer), 它使用主通道。 On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy. By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder driver. Alternatively, the fmu can be started in one of the capture modes, and then drivers can register a capture callback with ioctl calls.

### 实现

By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### 示例

It is typically started with:

    fmu mode_pwm
    

To drive all available pins.

Capture input (rising and falling edges) and print on the console: start the fmu in one of the capture modes:

    fmu mode_pwm3cap1
    

This will enable capturing on the 4th pin. Then do:

    fmu test
    

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load mixer files.

### Usage {#fmu_usage}

    fmu <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
    
     All of the mode_* commands will start the fmu if not running already
    
       mode_gpio
    
       mode_pwm      Select all available pins as PWM
    
       mode_pwm8
    
       mode_pwm6
    
       mode_pwm5
    
       mode_pwm5cap1
    
       mode_pwm4
    
       mode_pwm4cap1
    
       mode_pwm4cap2
    
       mode_pwm3
    
       mode_pwm3cap1
    
       mode_pwm2
    
       mode_pwm2cap2
    
       mode_pwm1
    
       sensor_reset  Do a sensor reset (SPI bus)
         [<ms>]      Delay time in ms between reset and re-enabling
    
       peripheral_reset Reset board peripherals
         [<ms>]      Delay time in ms between reset and re-enabling
    
       i2c           Configure I2C clock rate
         <bus_id> <rate> Specify the bus id (>=0) and rate in Hz
    
       test          Test inputs and outputs
    
       fake          Arm and send an actuator controls command
         <roll> <pitch> <yaw> <thrust> Control values in range [-100, 100]
    
       stop
    
       status        print status info
    

## gps

Source: [drivers/gps](https://github.com/PX4/Firmware/tree/master/src/drivers/gps)

### Description

GPS driver module that handles the communication with the device and publishes the position via uORB. It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published on the second uORB topic instance, but it's currently not used by the rest of the system (however the data will be logged, so that it can be used for comparisons).

### Implementation

There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks so that they can be used in other projects as well (eg. QGroundControl uses them too).

### Examples

For testing it can be useful to fake a GPS signal (it will signal the system that it has a valid position):

    gps stop
    gps start -f
    

Starting 2 GPS devices (the main GPS on /dev/ttyS3 and the secondary on /dev/ttyS4):

    gps start -d /dev/ttyS3 -e /dev/ttyS4
    

Initiate warm restart of GPS device

    gps reset warm
    

### Usage {#gps_usage}

    gps <command> [arguments...]
     Commands:
       start
         [-d <val>]  GPS device
                     values: <file:dev>, default: /dev/ttyS3
         [-b <val>]  Baudrate (can also be p:<param_name>)
                     default: 0
         [-e <val>]  Optional secondary GPS device
                     values: <file:dev>
         [-g <val>]  Baudrate (secondary GPS, can also be p:<param_name>)
                     default: 0
         [-f]        Fake a GPS signal (useful for testing)
         [-s]        Enable publication of satellite info
         [-i <val>]  GPS interface
                     values: spi|uart, default: uart
         [-p <val>]  GPS Protocol (default=auto select)
                     values: ubx|mtk|ash|eml
    
       stop
    
       status        print status info
    
       reset         Reset GPS device
         cold|warm|hot Specify reset type
    

## ina226

Source: [drivers/power_monitor/ina226](https://github.com/PX4/Firmware/tree/master/src/drivers/power_monitor/ina226)

### 描述

Driver for the INA226 power monitor.

Multiple instances of this driver can run simultaneously, if each instance has a separate bus OR I2C address.

For example, one instance can run on Bus 2, address 0x41, and one can run on Bus 2, address 0x43.

If the INA226 module is not powered, then by default, initialization of the driver will fail. To change this, use the -f flag. If this flag is set, then if initialization fails, the driver will keep trying to initialize again every 0.5 seconds. With this flag set, you can plug in a battery after the driver starts, and it will work. Without this flag set, the battery must be plugged in before starting the driver.

### Usage {#ina226_usage}

    ina226 <command> [arguments...]
     Commands:
       start         Start a new instance of the driver
         [-a]        If set, try to start the driver on each availabe I2C bus until
                     a module is found
         [-f]        If initialization fails, keep retrying periodically. Ignored if
                     the -a flag is set. See full driver documentation for more info
         [-b <val>]  I2C bus (default: use board-specific bus)
                     default: 0
         [-d <val>]  I2C Address (Start with '0x' for hexadecimal)
                     default: 65
         [-t <val>]  Which battery calibration values should be used (1 or 2)
                     default: 1
    
       stop          Stop one instance of the driver
         [-b <val>]  I2C bus (default: use board-specific bus)
                     default: 0
         [-d <val>]  I2C Address (Start with '0x' for hexadecimal)
                     default: 65
    
       status        Status of every instance of the driver
    
       info          Status of every instance of the driver
    

## pga460

Source: [drivers/distance_sensor/pga460](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/pga460)

### Description

Ultrasonic range finder driver that handles the communication with the device and publishes the distance via uORB.

### Implementation

This driver is implented as a NuttX task. This Implementation was chosen due to the need for polling on a message via UART, which is not supported in the work_queue. This driver continuously takes range measurements while it is running. A simple algorithm to detect false readings is implemented at the driver levelin an attemptto improve the quality of data that is being published. The driver will not publish data at all if it deems the sensor data to be invalid or unstable.

### Usage {#pga460_usage}

    pga460 <command> [arguments...]
     Commands:
       start <device_path>
         [device_path] The pga460 sensor device path, (e.g: /dev/ttyS6
    
       status
    
       stop
    
       help
    

## pwm_out_sim

Source: [drivers/pwm_out_sim](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out_sim)

### Description

Driver for simulated PWM outputs.

Its only function is to take `actuator_control` uORB messages, mix them with any loaded mixer and output the result to the `actuator_output` uORB topic.

It is used in SITL and HITL.

### Usage {#pwm_out_sim_usage}

    pwm_out_sim <command> [arguments...]
     Commands:
       start         Start the module
         [-m <val>]  Mode
                     values: hil|sim, default: sim
    
       stop
    
       status        print status info
    

## rc_input

Source: [drivers/rc_input](https://github.com/PX4/Firmware/tree/master/src/drivers/rc_input)

### Description

This module does the RC input parsing and auto-selecting the method. Supported methods are:

- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS Crossfire (CRSF)

### Implementation

By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread, specified via start flag -t, to reduce latency. When running on the work queue, it schedules at a fixed frequency.

### Usage {#rc_input_usage}

    rc_input <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
         [-t]        Run as separate task instead of the work queue
         [-d <val>]  RC device
                     values: <file:dev>, default: /dev/ttyS3
    
       bind          Send a DSM bind command (module must be running)
    
       stop
    
       status        print status info
    

## roboclaw

Source: [drivers/roboclaw](https://github.com/PX4/Firmware/tree/master/src/drivers/roboclaw)

### Description

This driver communicates over UART with the [Roboclaw motor driver](http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf). It performs two tasks:

- Control the motors based on the `actuator_controls_0` UOrb topic.
- Read the wheel encoders and publish the raw data in the `wheel_encoders` UOrb topic

In order to use this driver, the Roboclaw should be put into Packet Serial mode (see the linked documentation), and your flight controller's UART port should be connected to the Roboclaw as shown in the documentation. For Pixhawk 4, use the `UART & I2C B` port, which corresponds to `/dev/ttyS3`.

### Implementation

The main loop of this module (Located in `RoboClaw.cpp::task_main()`) performs 2 tasks:

1. Write `actuator_controls_0` messages to the Roboclaw as they become available
2. Read encoder data from the Roboclaw at a constant, fixed rate.

Because of the latency of UART, this driver does not write every single `actuator_controls_0` message to the Roboclaw immediately. Instead, it is rate limited based on the parameter `RBCLW_WRITE_PER`.

On startup, this driver will attempt to read the status of the Roboclaw to verify that it is connected. If this fails, the driver terminates immediately.

### Examples

The command to start this driver is:

$ roboclaw start <device> <baud>

`<device>` is the name of the UART port. On the Pixhawk 4, this is `/dev/ttyS3`. `<baud>` is te baud rate.

All available commands are:

- `$ roboclaw start <device> <baud>`
- `$ roboclaw status`
- `$ roboclaw stop`

### Usage {#roboclaw_usage}

    roboclaw <command> [arguments...]
     Commands:
    

## safety_button

Source: [drivers/safety_button](https://github.com/PX4/Firmware/tree/master/src/drivers/safety_button)

### Description

This module is responsible for the safety button.

### Usage {#safety_button_usage}

    safety_button <command> [arguments...]
     Commands:
       start         Start the safety button driver
    

## tap_esc

Source: [drivers/tap_esc](https://github.com/PX4/Firmware/tree/master/src/drivers/tap_esc)

### Description

This module controls the TAP_ESC hardware via UART. It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation

Currently the module is implementd as a threaded version only, meaning that it runs in its own thread instead of on the work queue.

### Example

The module is typically started with: tap_esc start -d /dev/ttyS2 -n <1-8>

### Usage {#tap_esc_usage}

    tap_esc <command> [arguments...]
     Commands:
       start         Start the task
         [-d <val>]  Device used to talk to ESCs
                     values: <device>
         [-n <val>]  Number of ESCs
                     default: 4
    

## vmount

Source: [modules/vmount](https://github.com/PX4/Firmware/tree/master/src/modules/vmount)

### Description

Mount (Gimbal) control driver. It maps several different input methods (eg. RC or MAVLink) to a configured output (eg. AUX channels or MAVLink).

Documentation how to use it is on the [gimbal_control](https://dev.px4.io/en/advanced/gimbal_control.html) page.

### Implementation

Each method is implemented in its own class, and there is a common base class for inputs and outputs. They are connected via an API, defined by the `ControlData` data structure. This makes sure that each input method can be used with each output method and new inputs/outputs can be added with minimal effort.

### Examples

Test the output by setting a fixed yaw angle (and the other axes to 0):

    vmount stop
    vmount test yaw 30
    

### Usage {#vmount_usage}

    vmount <command> [arguments...]
     Commands:
       start
    
       test          Test the output: set a fixed angle for one axis (vmount must
                     not be running)
         roll|pitch|yaw <angle> Specify an axis and an angle in degrees
    
       stop
    
       status        print status info