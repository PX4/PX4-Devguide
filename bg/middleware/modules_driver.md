# Modules Reference: Driver

Subcategories:

- [Distance Sensor](modules_driver_distance_sensor.md)

## atxxxx

Source: [drivers/osd/atxxxx](https://github.com/PX4/Firmware/tree/master/src/drivers/osd/atxxxx)

### Description

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
    

## fmu

Source: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)

### Description

This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip (eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy. By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder driver. Alternatively, the fmu can be started in one of the capture modes, and then drivers can register a capture callback with ioctl calls.

### Implementation

By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread, specified via start flag -t, to reduce latency. When running on the work queue, it schedules at a fixed frequency, and the pwm rate limits the update rate of the actuator_controls topics. In case of running in its own thread, the module polls on the actuator_controls topic. Additionally the pwm rate defines the lower-level IO timer rates.

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

### Description

GPS driver module that handles the communication with the device and publishes the position via uORB. It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published on the second uORB topic instance, but it's currently not used by the rest of the system (however the data will be logged, so that it can be used for comparisons).

### Implementation

There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks so that they can be used in other projects as well (eg. QGroundControl uses them too).

### Examples

For testing it can be useful to fake a GPS signal (it will signal the system that it has a valid position):

    gps stop
    gps start -f
    

Starting 2 GPS devices (the main GPS on /dev/ttyS3 and the secondary on /dev/ttyS4): gps start -d /dev/ttyS3 -e /dev/ttyS4

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
       start         Start the task in mode_pwm16
    
     All of the mode_* commands will start the pwm sim if not running already
    
       mode_pwm      use 8 PWM outputs
    
       mode_pwm16    use 16 PWM outputs
    
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
    
       bind          Send a DSM bind command (module must be running)
    
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