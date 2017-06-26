# 비행 로그 분석

PX4 비행 로그를 분석하는 다양한 소프트웨어 패키지가 있습니다. 아래에서 소개합니다.

## 비행 리포팅

PX4 사용자 가이드의 [how to report a flight](https://docs.px4.io/en/flight-reporting.html) 혹은 비행 관련 이슈에서 상세하게 소개합니다.

## 구조화된 분석

비행 로그를 분석하기 전에 콘텍스트를 만드는 것이 핵심 :

* 오작동 이후에 분석을 한다면, 해당 로그가 crash나 캡쳐했거나 공중에서 멈췄을까?
* 모든 컨트롤러들이 레퍼런스를 추적했는가? 이를 구성하는 가장 쉬운 위치가 attitude roll과 pitch를 set point를 비교해 나가는 것입니다.
* 센서 데이터가 정상적으로 보이는가? 강한 진동이 있었는가? \(강한 진동에 대해서 적절한 한계치는 2-3 m/s/s 이상의 peak-to-peak\)
* 비행체에 구체적인 근본 원인이 없다면, [PX4 issue tracker](https://github.com/px4/firmware/issues/new)에 log 파일에 대한 링크를 리포팅하세요.

## 전원 문제 제외하기

로그 파일이 비행중에 끝나버렸다면, 2개의 주요 원인을 생각해 볼 수 있습니다. : 전원 문제 아니면 운영시스템의 문제입니다. [STM32 ](http://www.st.com/en/microcontrollers/stm32-32-bit-arm-cortex-mcus.html?querycriteria=productId=SC1169)시리즈를 기반으로하는 autopilot의 운영체제 문제는 SD 카드에 기록됩니다.

이는 SD 카드의 상단부에 위치하며 **fault\_2017\_04\_03\_00\_26\_05.log** 와 같이 _ fault\_date.log_ 같은 형태입니다. 비행 로그가 갑자기 종료되는 경우라면 항상 이 파일이 생겨되었는지 확인하도록 합니다.

## [Flight Review 리뷰 온라인 툴](http://logs.px4.io)

Flight Review는 Log Muncher의 다음 버전으로 새로운 ULog 로깅 포맷과 결합하여 사용됩니다.

### 예제

![](../../assets/flight_log_analysis/flight-review-example.png)

### 강점

* 웹 기반, 엔드유저에게 효과적
* 사용자가 업로드 가능하며 리포트를 다른 사람들과 공유 가능
* 인터렉티브 그래프

## [FlightPlot 데스크탑 툴](https://github.com/PX4/FlightPlot)

![](https://pixhawk.org/_media/dev/flightplot-0.2.16-screenshot.png)

* [FlightPlot 다운로드](https://s3.amazonaws.com/flightplot/releases/latest.html)

### 강점

* java기반, 크로스 플랫폼
* 직관적인 GUI, 프로그래밍 지식없이 사용 가능

## [PX4Tools](https://github.com/dronecrew/px4tools)

![](../../assets/flight_log_analysis/px4tools.png)

### 설치

* 추천하는 방식은 anaconda3를 사용하는 것입니다. 상세 내용은 [px4tools github page](https://github.com/dronecrew/px4tools)을 참고하세요.

```bash
conda install -c https://conda.anaconda.org/dronecrew px4tools
```

### 강점

* 공유하기 쉽고, 사용자는 github에서 notebooks에 볼 수 있습니다. \(예제 [https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb](https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30 Kabir Log.ipynb)\)
* python 기반, 크로스 플랫폼, anaconda2와 anaconda3와 동작
* ipython/ jupyter notebooks은 쉽게 분석을 공유할 수 있습니다.
* 상세한 분석을 위해서 고급 그래프 기능
