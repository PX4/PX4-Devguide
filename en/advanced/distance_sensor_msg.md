# Distance sensor message
The not self-explanatory attributes of the [distance sensor message](http://mavlink.org/messages/common#DISTANCE_SENSOR) need to be filled accordingly.

> **Warning**
> The distances in the mavlink message are in centimeters while the distances in the [uorb message](https://github.com/PX4/Firmware/blob/b596874b91e8b1178bbdb9df625460d8188e079b/msg/distance_sensor.msg) are in meters!

## Type
The type needs to be set according to [MAV_DISTANCE_SENSOR](http://mavlink.org/messages/common#MAV_DISTANCE_SENSOR).

## Orientation
The [distance sensor message](https://github.com/PX4/Firmware/blob/b596874b91e8b1178bbdb9df625460d8188e079b/msg/distance_sensor.msg) has an orientation attribute which is of type [MAV_SENSOR_ORIENTATION](http://mavlink.org/messages/common#MAV_SENSOR_ORIENTATION). As a distance sensor is symmetric alongs it's axis, several orientations lead to the same setup.

### Convention

| Orientation     | CMD ID | Field name                    |
| --------------- | ------ | ----------------------------- |
| Downward-facing | 8      | MAV_SENSOR_ROTATION_ROLL_180  |
| Upward-facing   | 0      | MAV_SENSOR_ROTATION_NONE      |
| Forward-facing  | 25     | MAV_SENSOR_ROTATION_PITCH_270 |
| Backward-facing | 24     | MAV_SENSOR_ROTATION_PITCH_90  |
| Right-facing    | 16     | MAV_SENSOR_ROTATION_ROLL_90   |
| Left-facing     | 20     | MAV_SENSOR_ROTATION_ROLL_270  |
