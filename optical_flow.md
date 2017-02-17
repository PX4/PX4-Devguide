# Optical Flow on the Snapdragon Flight

The Snapdragon Flight board has a downward facing gray-scale camera which can be used for optical flow based position stabilization.

Besides a camera, optical flow requires a downward facing distance sensor. Here, the use of the TeraRanger One is discussed.


## TeraRanger One setup
To connect the TeraRanger One (TROne) to the Snapdragon Flight, the TROne I2C adapter must be used. The TROne must be flashed with the I2C firmware by the vendor.

The TROne is connected to the Snapdragon Flight through a custom DF13 4-to-6 pin cable. The wiring is as follows:

| 4 pin | <-> | 6 pin |
| -- | -- | -- |
| 1 |  | 1 |
| 2 |  | 6 |
| 3 |  | 4 |
| 4 |  | 5 |

The TROne must be powered with 10 - 20V.

## Optical flow
The optical flow is computed on the application processor and sent to PX4 through Mavlink.
Clone and compile the [snap_cam](https://github.com/PX4/snap_cam) repo according to the instructions in its readme.

Run the optical flow application as root:
````
optical_flow -n 50 -f 30
```

The optical flow application requres IMU Mavlink messages from PX4. You may have to add an additional Mavlink instance to PX4 by adding the following to your `mainapp.config`:

```
mavlink start -u 14557 -r 1000000 -t 127.0.0.1 -o 14558
mavlink stream -u 14557 -s HIGHRES_IMU -r 250
```
