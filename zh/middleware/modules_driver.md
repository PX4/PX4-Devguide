# 模块参考：驱动

Subcategories:

- [Distance Sensor](modules_driver_distance_sensor.md)

## batt_smbus

Source: [drivers/batt_smbus](https://github.com/PX4/Firmware/tree/master/src/drivers/batt_smbus)

### 描述

Smart battery driver for the BQ40Z50 fuel gauge IC.

### 示例

To write to flash to set parameters. address, number_of_bytes, byte0, ... , byteN

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

Source: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)

### 描述

This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip (eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy. By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder driver. Alternatively, the fmu can be started in one of the capture modes, and then drivers can register a capture callback with ioctl calls.

### 实现

By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread, specified via start flag -t, to reduce latency. When running on the work queue, it schedules at a fixed frequency, and the pwm rate limits the update rate of the actuator_controls topics. In case of running in its own thread, the module polls on the actuator_controls topic. Additionally the pwm rate defines the lower-level IO timer rates.

### 示例

It is typically started with:

    fmu mode_pwm
    

To drive all available pins.

Capture input (rising and falling edges) and print on the console: start the fmu in one of the capture modes:

    fmu mode_pwm3cap1
    

This will enable capturing on the 4th pin. Then do:

    fmu test
    

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load mixer files.

### 用法 {#fmu_usage}

    fmu <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
         [-t]        Run as separate task instead of the work queue
    
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

### 描述

GPS driver module that handles the communication with the device and publishes the position via uORB. It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published on the second uORB topic instance, but it's currently not used by the rest of the system (however the data will be logged, so that it can be used for comparisons).

### 实现

There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks so that they can be used in other projects as well (eg. QGroundControl uses them too).

### 示例

For testing it can be useful to fake a GPS signal (it will signal the system that it has a valid position):

    gps stop
    gps start -f
    

Starting 2 GPS devices (the main GPS on /dev/ttyS3 and the secondary on /dev/ttyS4): gps start -d /dev/ttyS3 -e /dev/ttyS4

### 用法 {#gps_usage}

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
    

## pga460

Source: [drivers/distance_sensor/pga460](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/pga460)

### 描述

Ultrasonic range finder driver that handles the communication with the device and publishes the distance via uORB.

### 实现

This driver is implented as a NuttX task. This Implementation was chosen due to the need for polling on a message via UART, which is not supported in the work_queue. This driver continuously takes range measurements while it is running. A simple algorithm to detect false readings is implemented at the driver levelin an attemptto improve the quality of data that is being published. The driver will not publish data at all if it deems the sensor data to be invalid or unstable.

### 用法 {#pga460_usage}

    pga460 &lt;command&gt; [arguments...]
     Commands:
       start &lt;device_path&gt;
         [device_path] pga460 传感器设备地址 (例如： /dev/ttyS6
    
       status
    
       stop
    
       help
    

## pwm_out_sim

Source: [drivers/pwm_out_sim](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out_sim)

### 描述

Driver for simulated PWM outputs.

Its only function is to take `actuator_control` uORB messages, mix them with any loaded mixer and output the result to the `actuator_output` uORB topic.

It is used in SITL and HITL.

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

Source: [drivers/rc_input](https://github.com/PX4/Firmware/tree/master/src/drivers/rc_input)

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

    rc_input &lt;command&gt; [arguments...]
     Commands:
       start         启动任务 (不执行任何模式设置，不使用任何 mode_* 命令)
         [-t]        以单独任务的形式运行，而不是工作队列
    
       bind          发送一个 DSM 绑定指令 (模块必须处于运行状态)
    
       stop
    
       status        打印状态信息
    

## tap_esc

Source: [drivers/tap_esc](https://github.com/PX4/Firmware/tree/master/src/drivers/tap_esc)

### 描述

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