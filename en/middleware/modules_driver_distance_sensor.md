# Modules Reference: Distance Sensor (Driver)
## leddar_one
Source: [drivers/distance_sensor/leddar_one](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/leddar_one)


### Description

Serial bus driver for the LeddarOne LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_LEDDAR1_CFG parameter.

Setup/usage information: https://docs.px4.io/en/sensor/leddar_one.html

### Examples

Attempt to start driver on a specified serial device.
```
leddar_one start -d /dev/ttyS1
```
Stop driver
```
leddar_one stop
```

### Usage {#leddar_one_usage}
```
leddar_one <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device
     [-r <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop          Stop driver

   test          Test driver (basic functional tests)
```
## sf1xx
Source: [drivers/distance_sensor/sf1xx](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/sf1xx)


### Description

I2C bus driver for Lightware SFxx series LIDAR rangefinders: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/en/sensor/sfxx_lidar.html

### Examples

Attempt to start driver on any bus (start on bus where first sensor found).
```
sf1xx start -a
```
Stop driver
```
sf1xx stop
```

### Usage {#sf1xx_usage}
```
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
```
## teraranger
Source: [drivers/distance_sensor/teraranger](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/teraranger)


### Description

I2C bus driver for TeraRanger rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_TRANGER.

Setup/usage information: https://docs.px4.io/en/sensor/rangefinders.html#teraranger-rangefinders

### Examples
Start driver on any bus (start on bus where first sensor found).
```
teraranger start -a
```
Start driver on specified bus
```
teraranger start -b 1
```
Stop driver
```
teraranger stop
```

### Usage {#teraranger_usage}
```
teraranger <command> [arguments...]
 Commands:
   start         Start driver
     [-a]        Attempt to start driver on all I2C buses (first one found)
     [-b <val>]  Start driver on specific I2C bus
                 default: 1
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop          Stop driver

   status        Print driver information
```
## tfmini
Source: [drivers/distance_sensor/tfmini](https://github.com/PX4/Firmware/tree/master/src/drivers/distance_sensor/tfmini)


### Description

Serial bus driver for the Benewake TFmini LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_TFMINI_CFG parameter.

Setup/usage information: https://docs.px4.io/en/sensor/tfmini.html

### Examples

Attempt to start driver on a specified serial device.
```
tfmini start -d /dev/ttyS1
```
Stop driver
```
tfmini stop
```

### Usage {#tfmini_usage}
```
tfmini <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   status        Driver status

   stop          Stop driver

   test          Test driver (basic functional tests)

   status        Print driver status
```
