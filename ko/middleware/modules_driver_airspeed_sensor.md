# 모듈 참고: 대기속도 센서(드라이버)
## ets_airspeed
소스 코드: [drivers/differential_pressure/ets](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/ets)

### 사용법 {#ets_airspeed_usage}
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
소스 코드: [drivers/differential_pressure/ms4525](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/ms4525)

### 사용법 {#ms4525_airspeed_usage}
```
ms4525_airspeed <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-T <val>]  Device type
                 values: 4525|4515, default: 4525

   stop

   status        print status info
```
## ms5525_airspeed
소스 코드: [drivers/differential_pressure/ms5525](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/ms5525)

### 사용법 {#ms5525_airspeed_usage}
```
ms5525_airspeed <command> [arguments...]
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
## sdp3x_airspeed
소스 코드: [drivers/differential_pressure/sdp3x](https://github.com/PX4/Firmware/tree/master/src/drivers/differential_pressure/sdp3x)

### 사용법 {#sdp3x_airspeed_usage}
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
