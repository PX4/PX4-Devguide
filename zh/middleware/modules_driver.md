# 模块参考：驱动

子分类

- [距离传感器](modules_driver_distance_sensor.md)

## adc

Source: [drivers/adc](https://github.com/PX4/Firmware/tree/master/src/drivers/adc)

### 描述

ADC driver.

### Usage {#adc_usage}

    adc <command> [arguments...]
     Commands:
       start
    
       test
    
       stop
    
       status        print status info
    

## atxxxx

Source: [drivers/osd/atxxxx](https://github.com/PX4/Firmware/tree/master/src/drivers/osd/atxxxx)

### 描述

OSD driver for the ATXXXX chip that is mounted on the OmnibusF4SD board for example.

It can be enabled with the OSD_ATXXXX_CFG parameter.

### Usage {#atxxxx_usage}

    atxxxx <command> [arguments...]
     Commands:
       start         Start the driver
         [-b <val>]  SPI bus (default: use board-specific bus)
    
       stop
    
       status        print status info
    

## batt_smbus

Source: [drivers/batt_smbus](https://github.com/PX4/Firmware/tree/master/src/drivers/batt_smbus)

### Description

Smart battery driver for the BQ40Z50 fuel gauge IC.

### Examples

To write to flash to set parameters. address, number_of_bytes, byte0, ... , byteN

    batt_smbus -X write_flash 19069 2 27 0
    

### Usage {#batt_smbus_usage}

    batt_smbus <command> [arguments...]
     Commands:
       start
         [-X]        BATT_SMBUS_BUS_I2C_EXTERNAL
         [-T]        BATT_SMBUS_BUS_I2C_EXTERNAL1
         [-R]        BATT_SMBUS_BUS_I2C_EXTERNAL2
         [-I]        BATT_SMBUS_BUS_I2C_INTERNAL
         [-A]        BATT_SMBUS_BUS_ALL
    
       man_info      Prints manufacturer info.
    
       unseal        Unseals the devices flash memory to enable write_flash
                     commands.
    
       seal          Seals the devices flash memory to disbale write_flash commands.
    
       suspend       Suspends the driver from rescheduling the cycle.
    
       resume        Resumes the driver from suspension.
    
       write_flash   Writes to flash. The device must first be unsealed with the
                     unseal command.
         [address]   The address to start writing.
         [number of bytes] Number of bytes to send.
         [data[0]...data[n]] One byte of data at a time separated by spaces.
    
       stop
    
       status        print status info
    

## dshot

Source: [drivers/dshot](https://github.com/PX4/Firmware/tree/master/src/drivers/dshot)

### Description

This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement to use DShot as ESC communication protocol instead of PWM.

It supports:

- DShot150, DShot300, DShot600, DShot1200
- 通过独立的串口遥控，并且发布esc_status消息
- 通过命令行接口发送 DShot 命令

### Examples

Permanently reverse motor 1:

    dshot reverse -m 1
    dshot save -m 1
    

After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.

### Usage {#dshot_usage}

    dshot <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
    
     All of the mode_* commands will start the module if not running already
    
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
    
       telemetry     Enable Telemetry on a UART
         <device>    UART device
    
       reverse       Reverse motor direction
         [-m <val>]  Motor index (1-based, default=all)
    
       normal        Normal motor direction
         [-m <val>]  Motor index (1-based, default=all)
    
       save          Save current settings
         [-m <val>]  Motor index (1-based, default=all)
    
       3d_on         Enable 3D mode
         [-m <val>]  Motor index (1-based, default=all)
    
       3d_off        Disable 3D mode
         [-m <val>]  Motor index (1-based, default=all)
    
       beep1         Send Beep pattern 1
         [-m <val>]  Motor index (1-based, default=all)
    
       beep2         Send Beep pattern 2
         [-m <val>]  Motor index (1-based, default=all)
    
       beep3         Send Beep pattern 3
         [-m <val>]  Motor index (1-based, default=all)
    
       beep4         Send Beep pattern 4
         [-m <val>]  Motor index (1-based, default=all)
    
       beep5         Send Beep pattern 5
         [-m <val>]  Motor index (1-based, default=all)
    
       esc_info      Request ESC information
         -m <val>    Motor index (1-based)
    
       stop
    
       status        print status info
    

## fmu

Source: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)

### Description

This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip (eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy. By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder driver. Alternatively, the fmu can be started in one of the capture modes, and then drivers can register a capture callback with ioctl calls.

### Implementation

By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples

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

### Description

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
    

## pca9685_pwm_out

Source: [drivers/pca9685_pwm_out](https://github.com/PX4/Firmware/tree/master/src/drivers/pca9685_pwm_out)

### Description

This module is responsible for generate pwm pulse with PCA9685 chip.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation

This module depends on ModuleBase and OutputModuleInterface. IIC communication is based on CDev::I2C

### Examples

It is typically started with:

    pca9685_pwm_out start -a 64 -b 1
    

Use the `mixer` command to load mixer files. `mixer load /dev/pca9685 ROMFS/px4fmu_common/mixers/quad_x.main.mix`

### Usage {#pca9685_pwm_out_usage}

    pca9685_pwm_out <command> [arguments...]
     Commands:
       start         Start the task
         [-a <val>]  device address on this bus
                     default: 64
         [-b <val>]  bus that pca9685 is connected to
                     default: 1
    
       stop
    
       status        print status info
    

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

### Usage {#rc_input_usage}

    rc_input <command> [arguments...]
     Commands:
       start
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

This module is responsible for the safety button. Pressing the safety button 3 times quickly will trigger a GCS pairing request.

### Usage {#safety_button_usage}

    safety_button <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

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