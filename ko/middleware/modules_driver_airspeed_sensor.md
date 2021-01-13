!REDIRECT "https://docs.px4.io/master/ko/middleware/modules_driver_airspeed_sensor.html"

# 모듈 참고: 항속 센서(드라이버)
## ets_airspeed
Source: [drivers/differential_pressure/ets](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/ets)

<a id="ets_airspeed_usage"></a>

### Usage
```
ets_airspeed <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)

   stop

   status        print status info
```
## ms4525_airspeed
Source: [drivers/differential_pressure/ms4525](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/ms4525)

<a id="ms4525_airspeed_usage"></a>

### Usage
```
ms4525_airspeed
```
## ms5525_airspeed
Source: [drivers/differential_pressure/ms5525](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/ms5525)

<a id="ms5525_airspeed_usage"></a>

### Usage
```
ms5525_airspeed
```
## sdp3x_airspeed
Source: [drivers/differential_pressure/sdp3x](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/sdp3x)

<a id="sdp3x_airspeed_usage"></a>

### Usage
```
sdp3x_airspeed <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 33
     [-k]        if initialization (probing) fails, keep retrying periodically

   stop

   status        print status info
```
