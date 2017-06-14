# 비행 로그 분석

PX4 비행 로그를 분석하는 다양한 소프트웨어 패키지가 있습니다. 아래에서 소개합니다.

## 비행 리포팅

PX4 사용자 가이드의 [how to report a flight](https://docs.px4.io/en/flight-reporting.html) 혹은 비행 관련 이슈에서 상세하게 소개합니다.

## 구조화된 분석

비행 로그를 분석하기 전에 콘텍스트를 만드는 것이 핵심 :

* 오작동 이후에 분석을 한다면, 해당 로그가 crash나 캡쳐했거나 공중에서 멈췄을까?
* 
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

* The recommended procedure is to use anaconda3. See [px4tools github page](https://github.com/dronecrew/px4tools) for details.

```bash
conda install -c https://conda.anaconda.org/dronecrew px4tools
```

### Strengths

* easy to share, users can view notebooks on github \(e.g. [https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb](https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30 Kabir Log.ipynb)\)
* python based, cross platform, works witn anaconda 2 and anaconda3
* ipython/ jupyter notebooks can be used to share analysis easily
* advanced plotting capabilities to allow detailed analysis
