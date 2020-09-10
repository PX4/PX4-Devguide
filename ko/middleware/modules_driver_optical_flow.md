# 모듈 참고: 광류 센서 (드라이버)
## thoneflow
소스 코드: [drivers/optical_flow/thoneflow](https://github.com/PX4/Firmware/tree/master/src/drivers/optical_flow/thoneflow)


### 설명

ThoneFlow-3901U 광류 센서용 직렬 버스 드라이버입니다.

대부분의 보드는 SENS_TFLOW_CFG 매개변수로 지정 UART 드라이버를 활성/시작하도록 설정했습니다.

설정/활용 정보: https://docs.px4.io/en/sensor/thoneflow.html

### 예제

지정 직렬 통신 장치에서 드라이버를 시작하려면
```
thoneflow start -d /dev/ttyS1
```
드라이버 동작 중단
```
thoneflow stop
```

### 사용법 {#thoneflow_usage}
```
thoneflow <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   stop          Stop driver

   info          Print driver information
```
