# 모듈 참고: 드라이버

하위 분류:

- [Imu](modules_driver_imu.md)
- [Distance Sensor](modules_driver_distance_sensor.md)
- [Airspeed Sensor](modules_driver_airspeed_sensor.md)
- [Baro](modules_driver_baro.md)
- [Magnetometer](modules_driver_magnetometer.md)

## adc

Source: [drivers/adc](https://github.com/PX4/Firmware/tree/master/src/drivers/adc)

### Description

ADC driver.

### Usage {#adc_usage}

    adc <command> [arguments...]
     Commands:
       start
    
       test
    
       stop
    
       status        print status info
    

## atxxxx

소스 코드: [drivers/osd/atxxxx](https://github.com/PX4/Firmware/tree/master/src/drivers/osd/atxxxx)

### 설명

OmnibusF4SD 보드에 붙은것과 같은 ATXXXX 칩용 OSD 드라이버입니다.

OSD_ATXXXX_CFG 매개변수로 활성화할 수 있습니다.

### 사용법 {#atxxxx_usage}

    atxxxx <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
    
       stop
    
       status        print status info
    

## batt_smbus

소스 코드: [drivers/batt_smbus](https://github.com/PX4/Firmware/tree/master/src/drivers/batt_smbus)

### 설명

BQ40Z50 잔여량 측정 IC용 지능형 배터리 드라이버.

### 예제

다음 설정 매개변수를 플래시에 기록합니다: address, number_of_bytes, byte0, ... , byteN

    batt_smbus -X write_flash 19069 2 27 0
    

### 사용법 {#batt_smbus_usage}

    batt_smbus <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-a <val>]  I2C address
                     default: 11
    
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
    

## blinkm

소스 코드: [drivers/lights/blinkm](https://github.com/PX4/Firmware/tree/master/src/drivers/lights/blinkm)

### 사용법 {#blinkm_usage}

    blinkm <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-a <val>]  I2C address
                     default: 9
    
       systemstate
    
       ledoff
    
       list
    
       script
         -n <val>    Script file name
                     values: <file>
    
       stop
    
       status        print status info
    

## bst

소스 코드: [drivers/telemetry/bst](https://github.com/PX4/Firmware/tree/master/src/drivers/telemetry/bst)

### 사용법 {#bst_usage}

    bst <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-a <val>]  I2C address
                     default: 118
    
       stop
    
       status        print status info
    

## dshot

소스 코드: [drivers/dshot](https://github.com/PX4/Firmware/tree/master/src/drivers/dshot)

### 설명

This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement to use DShot as ESC communication protocol instead of PWM.

It supports:

- DShot150, DShot300, DShot600, DShot1200
- telemetry via separate UART and publishing as esc_status message
- sending DShot commands via CLI

### 예제

Permanently reverse motor 1:

    dshot reverse -m 1
    dshot save -m 1
    

After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.

### 사용법 {#dshot_usage}

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
    

## fake_magnetometer

소스 코드: [examples/fake_magnetometer](https://github.com/PX4/Firmware/tree/master/src/examples/fake_magnetometer)

### 설명

Publish the earth magnetic field as a fake magnetometer (sensor_mag). Requires vehicle_attitude and vehicle_gps_position.

### 사용법 {#fake_magnetometer_usage}

    fake_magnetometer <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## gps

소스 코드: [drivers/gps](https://github.com/PX4/Firmware/tree/master/src/drivers/gps)

### 설명

GPS driver module that handles the communication with the device and publishes the position via uORB. It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published on the second uORB topic instance, but it's currently not used by the rest of the system (however the data will be logged, so that it can be used for comparisons).

### 구현

There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks so that they can be used in other projects as well (eg. QGroundControl uses them too).

### 예제

For testing it can be useful to fake a GPS signal (it will signal the system that it has a valid position):

    gps stop
    gps start -f
    

Starting 2 GPS devices (the main GPS on /dev/ttyS3 and the secondary on /dev/ttyS4):

    gps start -d /dev/ttyS3 -e /dev/ttyS4
    

Initiate warm restart of GPS device

    gps reset warm
    

### 사용법 {#gps_usage}

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

소스 코드: [drivers/power_monitor/ina226](https://github.com/PX4/Firmware/tree/master/src/drivers/power_monitor/ina226)

### 설명

Driver for the INA226 power monitor.

Multiple instances of this driver can run simultaneously, if each instance has a separate bus OR I2C address.

For example, one instance can run on Bus 2, address 0x41, and one can run on Bus 2, address 0x43.

If the INA226 module is not powered, then by default, initialization of the driver will fail. To change this, use the -f flag. If this flag is set, then if initialization fails, the driver will keep trying to initialize again every 0.5 seconds. With this flag set, you can plug in a battery after the driver starts, and it will work. Without this flag set, the battery must be plugged in before starting the driver.

### Usage {#ina226_usage}

    ina226 <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-a <val>]  I2C address
                     default: 65
         [-k]        if initialization (probing) fails, keep retrying periodically
         [-t <val>]  battery index for calibration values (1 or 2)
                     default: 1
    
       stop
    
       status        print status info
    

## irlock

Source: [drivers/irlock](https://github.com/PX4/Firmware/tree/master/src/drivers/irlock)

### Usage {#irlock_usage}

    irlock <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-a <val>]  I2C address
                     default: 84
    
       stop
    
       status        print status info
    

## lsm303agr

Source: [drivers/magnetometer/lsm303agr](https://github.com/PX4/Firmware/tree/master/src/drivers/magnetometer/lsm303agr)

### Usage {#lsm303agr_usage}

    lsm303agr <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
         [-R <val>]  Rotation
                     default: 0
    
       stop
    
       status        print status info
    

## paw3902

Source: [drivers/optical_flow/paw3902](https://github.com/PX4/Firmware/tree/master/src/drivers/optical_flow/paw3902)

### Usage {#paw3902_usage}

    paw3902 <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
         [-R <val>]  Rotation
                     default: 0
    
       stop
    
       status        print status info
    

## pca9685

Source: [drivers/pca9685](https://github.com/PX4/Firmware/tree/master/src/drivers/pca9685)

### Usage {#pca9685_usage}

    pca9685 <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
    
       reset
    
       test          enter test mode
    
       stop
    
       status        print status info
    

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
    

## pcf8583

Source: [drivers/rpm/pcf8583](https://github.com/PX4/Firmware/tree/master/src/drivers/rpm/pcf8583)

### Usage {#pcf8583_usage}

    pcf8583 <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
    
       stop
    
       status        print status info
    

## pmw3901

Source: [drivers/optical_flow/pmw3901](https://github.com/PX4/Firmware/tree/master/src/drivers/optical_flow/pmw3901)

### Usage {#pmw3901_usage}

    pmw3901 <command> [arguments...]
     Commands:
       start
         [-s]        Internal SPI bus(es)
         [-S]        External SPI bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-c <val>]  chip-select index (for external SPI)
                     default: 1
         [-m <val>]  SPI mode
         [-f <val>]  bus frequency in kHz
         [-R <val>]  Rotation
                     default: 0
    
       stop
    
       status        print status info
    

## pwm_out

소스 코드: [drivers/pwm_out](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out)

### Description

This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip (eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy. By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder driver. Alternatively, pwm_out can be started in one of the capture modes, and then drivers can register a capture callback with ioctl calls.

### Implementation

By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples

It is typically started with:

    pwm_out mode_pwm
    

To drive all available pins.

Capture input (rising and falling edges) and print on the console: start pwm_out in one of the capture modes:

    pwm_out mode_pwm3cap1
    

This will enable capturing on the 4th pin. Then do:

    pwm_out test
    

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load mixer files.

### 사용법 {#pwm_out_usage}

    pwm_out <command> [arguments...]
     Commands:
       start         Start the task (without any mode set, use any of the mode_*
                     cmds)
    
     All of the mode_* commands will start pwm_out if not running already
    
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
    
       stop
    
       status        print status info
    

## pwm_out_sim

소스 코드: [drivers/pwm_out_sim](https://github.com/PX4/Firmware/tree/master/src/drivers/pwm_out_sim)

### 설명

Driver for simulated PWM outputs.

Its only function is to take `actuator_control` uORB messages, mix them with any loaded mixer and output the result to the `actuator_output` uORB topic.

It is used in SITL and HITL.

### 사용법 {#pwm_out_sim_usage}

    pwm_out_sim <command> [arguments...]
     Commands:
       start         Start the module
         [-m <val>]  Mode
                     values: hil|sim, default: sim
    
       stop
    
       status        print status info
    

## px4flow

소스 코드: [drivers/optical_flow/px4flow](https://github.com/PX4/Firmware/tree/master/src/drivers/optical_flow/px4flow)

### 사용법 {#px4flow_usage}

    px4flow <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-a <val>]  I2C address
                     default: 66
         [-R <val>]  Rotation (default=downwards)
                     default: 25
    
       stop
    
       status        print status info
    

## rc_input

소스 코드: [drivers/rc_input](https://github.com/PX4/Firmware/tree/master/src/drivers/rc_input)

### 설명

This module does the RC input parsing and auto-selecting the method. Supported methods are:

- PPM
- SBUS
- DSM
- SUMD
- ST24
- TBS Crossfire (CRSF)

### 사용법 {#rc_input_usage}

    rc_input <command> [arguments...]
     Commands:
       start
         [-d <val>]  RC device
                     values: <file:dev>, default: /dev/ttyS3
    
       bind          Send a DSM bind command (module must be running)
    
       stop
    
       status        print status info
    

## rgbled

소스 코드: [drivers/lights/rgbled_ncp5623c](https://github.com/PX4/Firmware/tree/master/src/drivers/lights/rgbled_ncp5623c)

### 사용법 {#rgbled_usage}

    rgbled <command> [arguments...]
     Commands:
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-a <val>]  I2C address
                     default: 57
    
       stop
    
       status        print status info
    

## roboclaw

소스 코드: [drivers/roboclaw](https://github.com/PX4/Firmware/tree/master/src/drivers/roboclaw)

### 설명

This driver communicates over UART with the [Roboclaw motor driver](http://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf). It performs two tasks:

- Control the motors based on the `actuator_controls_0` UOrb topic.
- Read the wheel encoders and publish the raw data in the `wheel_encoders` UOrb topic

In order to use this driver, the Roboclaw should be put into Packet Serial mode (see the linked documentation), and your flight controller's UART port should be connected to the Roboclaw as shown in the documentation. For Pixhawk 4, use the `UART & I2C B` port, which corresponds to `/dev/ttyS3`.

### 구현

The main loop of this module (Located in `RoboClaw.cpp::task_main()`) performs 2 tasks:

1. Write `actuator_controls_0` messages to the Roboclaw as they become available
2. Read encoder data from the Roboclaw at a constant, fixed rate.

Because of the latency of UART, this driver does not write every single `actuator_controls_0` message to the Roboclaw immediately. Instead, it is rate limited based on the parameter `RBCLW_WRITE_PER`.

On startup, this driver will attempt to read the status of the Roboclaw to verify that it is connected. If this fails, the driver terminates immediately.

### 예제

이 드라이버를 시작하는 명령은 다음과 같습니다:

$ roboclaw start <device> <baud>

`<device>`은(는) UART 포트 이름입니다. 픽스호크 4에서는 `/dev/ttyS3`입니다. `<baud>`은(는) 초당 비트 전송율입니다.

사용할 수 있는 명령어는 다음과 같습니다:

- `$ roboclaw start <device> <baud>`
- `$ roboclaw status`
- `$ roboclaw stop`

### 사용법 {#roboclaw_usage}

    roboclaw <command> [arguments...]
     Commands:
    

## safety_button

소스 코드: [drivers/safety_button](https://github.com/PX4/Firmware/tree/master/src/drivers/safety_button)

### 설명

This module is responsible for the safety button. Pressing the safety button 3 times quickly will trigger a GCS pairing request.

### 사용법 {#safety_button_usage}

    safety_button <command> [arguments...]
     Commands:
       start
    
       stop
    
       status        print status info
    

## tap_esc

소스 코드: [drivers/tap_esc](https://github.com/PX4/Firmware/tree/master/src/drivers/tap_esc)

### 설명

This module controls the TAP_ESC hardware via UART. It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### 구현

Currently the module is implementd as a threaded version only, meaning that it runs in its own thread instead of on the work queue.

### 예제

The module is typically started with: tap_esc start -d /dev/ttyS2 -n <1-8>

### 사용법 {#tap_esc_usage}

    tap_esc <command> [arguments...]
     Commands:
       start         Start the task
         [-d <val>]  Device used to talk to ESCs
                     values: <device>
         [-n <val>]  Number of ESCs
                     default: 4
    

## vmount

소스 코드: [modules/vmount](https://github.com/PX4/Firmware/tree/master/src/modules/vmount)

### 설명

Mount (Gimbal) control driver. It maps several different input methods (eg. RC or MAVLink) to a configured output (eg. AUX channels or MAVLink).

Documentation how to use it is on the [gimbal_control](https://dev.px4.io/master/en/advanced/gimbal_control.html) page.

### 구현

Each method is implemented in its own class, and there is a common base class for inputs and outputs. They are connected via an API, defined by the `ControlData` data structure. This makes sure that each input method can be used with each output method and new inputs/outputs can be added with minimal effort.

### 예제

Test the output by setting a fixed yaw angle (and the other axes to 0):

    vmount stop
    vmount test yaw 30
    

### 사용법 {#vmount_usage}

    vmount <command> [arguments...]
     Commands:
       start
    
       test          Test the output: set a fixed angle for one axis (vmount must
                     not be running)
         roll|pitch|yaw <angle> Specify an axis and an angle in degrees
    
       stop
    
       status        print status info
    

## voxlpm

소스 코드: [drivers/power_monitor/voxlpm](https://github.com/PX4/Firmware/tree/master/src/drivers/power_monitor/voxlpm)

### 사용법 {#voxlpm_usage}

    voxlpm [arguments...]
       start
         [-I]        Internal I2C bus(es)
         [-X]        External I2C bus(es)
         [-b <val>]  bus (board-specific internal (default=all) or n-th external
                     (default=1))
         [-f <val>]  bus frequency in kHz
         [-T <val>]  Type
                     values: VBATT|P5VDC, default: VBATT
    
       stop
    
       status        print status info