# Thermal Calibration and Compensation

## Overview

Use material from [https://docs.google.com/document/d/1GHzjsPTfSUAxYGPNlXSL7FYkxFcq76jTfsaquc57Rww/edit](https://docs.google.com/document/d/1GHzjsPTfSUAxYGPNlXSL7FYkxFcq76jTfsaquc57Rww/edit)

PX4 conains functionality to calibrate and compensate rate gyro, accelerometer and barometric pressure sensors for the effect of changing sensor temperature on sensor bias. Calibration refers to the process of measuring the change in sensor value across a range of internal temperatures and performing a polynomial fit on the data to calculate a set of coefficients stored as parameters that can be used to correct the sensor data. Compensation refers to the process of using the internal temperature to calculate an offset which is subtracted from the sensor reading to correct for changing offset with temperature

The inertial rate gyro and accelerometer sensor offsets are calculated using a 3rd order polynomial, whereas the barometric pressure sensor offset is calculated using a 5th order polynomial.

## Calibration Parameter Storage

With the existing parameter system implementation we are limited to storing each value in the struct as a separate entry. To work around this limitation the following logical naming convention for the parameters is used:

TC\_&lt;type&gt;&lt;instance&gt;\_&lt;cal\_name&gt;\_&lt;axis&gt; , where

* &lt;type&gt; : is a single character indicating the type of sensor where G = rate gyroscope, A = accelerometer and B = barometer

* &lt;instance&gt; : is an integer 0,1 or 2 allowing for calibration of up to three sensors of the same &lt;type&gt;

* &lt;cal\_name&gt; : is a string identifyng the calibration value with the following possible values:

  * Xn : Polynomial coefficient where n is the order of the coefficient, eg X3 \* \(temperature - reference temperature\)\*\*3

  * SCL : scale factor

  * TREF : reference temperature \(deg C\)

  * TMIN : minimum valid temperature \(deg C\)

  * TMAX : maximum valid temperature \(deg C\)

* &lt;axis&gt; : is an integer 0,1 or 2 indicating that the cal data is for X,Y or Z axis in the board frame of reference. for the barometric pressure sensor, the \_&lt;axis&gt; suffix is omitted.

Examples:

TC\_G0\_X3\_0 is the ^3 coefficient for the first gyro x-axis

TC\_A1\_TREF is the reference temperature for the second accelerometer

## Calibration Parameter Useage

The correction for thermal offsets using the calibration parameters is performed in the sensors module.  The reference temperature is subtracted from the measured temperature to obtain a delta temperature where:

delta = measured\_temperature - refernece\_temperature.

The delta temperature is then used to calculate a offset, where:

offset = X0 + X1\*delta + X2\*delta\*\*2 + ... + Xn\*delta\*\*n

The offset and temperature scale factor are then used to correct the sensor measurement where:

corrected\_measurement = \(raw\_measurement - offset\) \* scale\_factor

If the temperature is above the test range set by the TMIN and TMAX parameters, then the measured temperature will be clipped to remain within the limits.

Correction os the accelerometer, barometers or rate gyroscope data is enabled by setting TC\_A\_ENABLE\_, \_TC\_B\_ENABLE or TC\_G\_ENABLE parameters to 1 respectively.

## Compatibility with legacy CAL\_\* parameters and commander controlled calibration

The legacy temperature agnostic PX4 rate gyro and accelerometer sensor calibration is performed by the commander module and involves adjusting offset, and in the case of accelerometer calibration, scale factor calibraton parameters. the parameters them selves are applied within the individual drivers for each sensor. These parameters are found in the CAL parameter group.

Onboard temperature calibration is controlled by the events module and the corrections are applied within the sensors module before the sensor combined uORB topic is published. This means that if thermal compensation is being used, all of the corresponding legacy offset and scale factor parameters must be set to defaults of zero and unity before a thermal calibration is performed. If an on-board temperature calibration is performed, this will be done automatically, however if an offboard calibration is being performed it is important that the legacy CAL\*OFF and CAL\*SCALE parameters be reset before calibration data is logged

If gyro thermal compensation has been enabled by setting the TC\_\_G\_\_ENABLE parameter to 1, then the commander controlled gyro calibration can still be perfomred, however it will be used to shift the compensation curve up or down by the amount required to zero the angular rate offset at the current temperature. It achieves this by adjusting the X0 coefficients.

If accel thermal compensation has been enabled by setting the TC\_A\_ENABLE parameter to 1, then the commander controlled 6-point accel calibration can still be perfomred, however in stead of adjusting the \*OFF and \*SCALE parameters in the CAL parameter group, these parameters are set to defaults and the thermal compensation X0 and SCL parameters are adjusted instead.

## Limitations

Scale factors are assumed to be temperature invariant due to the difficulty associated with measuring these at different temperatures. This limits the usefulness of the accelerometer calibration to those sensor models with stable scale factors. In theory with a thermal chamber or IMU heater capable of controlling IMU internal temperature to within a degree, it would be possible to perform a series of 6 sided accelerometer calibrations and correct the acclerometers for both offset and scale factor. Due to the complexity of integrating the motion control with the calibration algorithm, this capability  has not been  included.

## Onboard Calibration Procedure

Thermal calibration requires warming the board across the range of temperature it is required to operate in whilst running the onboard calibrator.

## Offboard Calibration Procedure

Add step by step procedure calibration using the SYS\_LOGGER and SDLOG\_MODE parameters and the process\_sensor\_caldata.py script file.

## FAQ



