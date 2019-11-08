# 模块参考：驱动

Subcategories:

- [Distance Sensor](modules_driver_distance_sensor.md)

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
         [-X <val>]  ullpt
                     default: BATT_SMBUS_BUS_I2C_EXTERNAL
         [-T <val>]  ullpt
                     default: BATT_SMBUS_BUS_I2C_EXTERNAL1
         [-R <val>]  ullpt
                     default: BATT_SMBUS_BUS_I2C_EXTERNAL2
         [-I <val>]  ullpt
                     default: BATT_SMBUS_BUS_I2C_INTERNAL
         [-A <val>]  ullpt
                     default: BATT_SMBUS_BUS_ALL
    
       man_info      Prints manufacturer info.
    
       report        Prints the last report.
    
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
    

## dshot

Source: [drivers/dshot](https://github.com/PX4/Firmware/tree/master/src/drivers/dshot)

### Description

This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement to use DShot as ESC communication protocol instead of PWM.

It supports:

- DShot150, DShot300, DShot600, DShot1200
- telemetry via separate UART and publishing as esc_status message
- sending DShot commands via CLI

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
    

## pga460

Source: [drivers/distance_sensor/pga460](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/pga460)

### 描述

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