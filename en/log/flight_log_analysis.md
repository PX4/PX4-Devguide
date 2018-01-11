# Flight Log Analysis

There are several software packages that exist to analyze PX4 flight logs. They are described below.

## Reporting Flights

The instructions in the PX4 user guide detail [how to report a flight](https://docs.px4.io/en/flight-reporting.html) or issues with a flight.

## Structured Analysis

Before analyzing a flight log it is key to establish the context of it:

* If the analysis is done after a malfunction, did the log capture the crash or did it stop mid-air?
* Did all controllers track their references? The easiest location to establish this is comparing attitude roll and pitch rates to their set points.
* Does the sensor data look valid? Was there very strong vibration \(a reasonable threshold for strong vibration is anything with a peak-to-peak of more than 2-3 m/s/s\)
* If the root cause is not specific to the vehicle make sure to report it with a link to the log file \(and video if one exists\) on the [PX4 issue tracker](https://github.com/px4/firmware/issues/new).

## Ruling Out Power Failures

If a log file ends mid-air, two main causes are possible: A power failure or a hard fault of the operating system. On autopilots based on the [STM32 ](http://www.st.com/en/microcontrollers/stm32-32-bit-arm-cortex-mcus.html?querycriteria=productId=SC1169)series hard faults of the operating system are logged to the SD card.

These are located on the top level of the SD card and named_ fault\_date.log_, e.g. **fault\_2017\_04\_03\_00\_26\_05.log**. Please always check for the presence of this file if a flight log ends abruptly.

## [Flight Review Online Tool](http://logs.px4.io)

Flight Review is the successor of Log Muncher, used in combination with the new  
ULog logging format.

### Example

![](../../assets/flight_log_analysis/flight-review-example.png)

### Strengths

* web based, great for end-users
* user can upload, load and then share report with others
* interactive plots

## [FlightPlot Desktop Tool](https://github.com/PX4/FlightPlot)

![](https://pixhawk.org/_media/dev/flightplot-0.2.16-screenshot.png)

* [FlightPlot Downloads](https://s3.amazonaws.com/flightplot/releases/latest.html)

### Strengths

* java based, cross-platform
* intuitive GUI, no programming knowledge required

## [PX4Tools](https://github.com/dronecrew/px4tools)

![](../../assets/flight_log_analysis/px4tools.png)

### Install

The recommended procedure is to use anaconda3. See [px4tools github page](https://github.com/dronecrew/px4tools) for details.


### Strengths

* easy to share, users can view notebooks on github \(e.g. [https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb](https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30 Kabir Log.ipynb)\)
* python based, cross platform, works witn anaconda 2 and anaconda3
* ipython/ jupyter notebooks can be used to share analysis easily
* advanced plotting capabilities to allow detailed analysis



