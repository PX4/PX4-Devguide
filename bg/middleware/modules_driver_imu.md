# Modules Reference: Imu (Driver)
## bmi055
Source: [drivers/imu/bmi055](https://github.com/PX4/Firmware/tree/master/src/drivers/imu/bmi055)

### Usage {#bmi055_usage}
```
bmi055 <command> [arguments...]
 Commands:
   start
     [-A]        Accel
     [-G]        Gyro
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

   regdump

   testerror

   stop

   status        print status info
```
## lsm9ds1
Source: [drivers/imu/st/lsm9ds1](https://github.com/PX4/Firmware/tree/master/src/drivers/imu/st/lsm9ds1)

### Usage {#lsm9ds1_usage}
```
lsm9ds1 <command> [arguments...]
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
```
## mpu6000
Source: [drivers/imu/mpu6000](https://github.com/PX4/Firmware/tree/master/src/drivers/imu/mpu6000)

### Usage {#mpu6000_usage}
```
mpu6000 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es"
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
     [-T <val>]  Device type
                 values: 6000|20608|20602|20689, default: 6000

   reset

   regdump

   factorytest

   testerror

   stop

   status        print status info
```
