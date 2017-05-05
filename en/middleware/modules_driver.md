# Modules Reference: Driver
## fmu
Source: [drivers/px4fmu](https://github.com/PX4/Firmware/tree/master/src/drivers/px4fmu)


### Description
This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.
In addition it does the RC input parsing and auto-selecting the method. Supported methods are:
- PPM
- SBUS
- DSM
- SUMD
- ST24

The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy.
By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder
driver.

### Implementation
By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread,
specified via start flag -t, to reduce latency.
When running on the work queue, it schedules at a fixed frequency, and the pwm rate limits the update rate of
the actuator_controls topics. In case of running in its own thread, the module polls on the actuator_controls topic.
Additionally the pwm rate defines the lower-level IO timer rates.

### Examples
It is typically started with:
```
fmu mode_pwm
```
To drive all available pins.

Capture input (rising and falling edges) and print on the console: start the fmu in one of the capture modes:
```
fmu mode_pwm3cap1
```
This will enable capturing on the 4th pin. Then do:
```
fmu test
```

Use the `pwm` command for further configurations (PWM rate, levels, ...), and the `mixer` command to load
mixer files.

### Usage
```
fmu <command> [arguments...]
 Commands:
   start         Start the task (without any mode set, use any of the mode_*
                 cmds)
     [-t]        Run as separate task instead of the work queue

 All of the mode_* commands will start the fmu if not running already

   mode_gpio

   mode_rcin     Only do RC input, no PWM outputs

   mode_pwm      Select all available pins as PWM

   mode_pwm1

   mode_pwm4

   mode_pwm2

   mode_pwm3

   mode_pwm3cap1

   mode_pwm2cap2

   mode_serial

   mode_gpio_serial

   mode_pwm_serial

   mode_pwm_gpio

   bind          Send a DSM bind command (module must be running)

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
```
