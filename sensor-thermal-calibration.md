# Thermal Calibration and Compensation

## Overview

Use material from [https://docs.google.com/document/d/1GHzjsPTfSUAxYGPNlXSL7FYkxFcq76jTfsaquc57Rww/edit](https://docs.google.com/document/d/1GHzjsPTfSUAxYGPNlXSL7FYkxFcq76jTfsaquc57Rww/edit)

PX4 conains functionality to calibrate and compensate rate gyro, accelerometer and barometric pressure sensors for the effect of changing sensor temperature on sensor bias. Calibration refers to the process of measuring the change in sensor value across a range of internal temperatures and performing a polynomial fit on the data to calculate a set of coefficients stored as parameters that can be used to correct the sensor data. Compensation refers to the process of using the internal temperature to calculate an offset which is subtracted from the sensor reading to correct for changing offset with temperature

The inertial rate gyro and accelerometer sensor offsets are calculated using a 3rd order polynomial, whereas the barometric pressure sensor offset is calculated using a 5th order polynomial.

Scale factors are assumed to be temperature invariant due to the difficulty associated with measuring these at different temperatures. In theory with a thermal chamber or IMU heater capable of controlling IMU internal temperature to within a degree, it would be possible to perform a series of

## Onboard Calibration Procedure

Add step by step procedure for calibration using the inbuilt calibrator and the SYS\_CAL\_\* parameters.

## Offboard Calibration Procedure

Add step by step procedure calibration using the SYS\_LOGGER and SDLOG\_MODE parameters and the process\_sensor\_caldata.py script file.

## FAQ

### Why aren't scale factors compensated?



