# Modules Reference: Communication
## mavlink
소스: [modules/mavlink](https://github.com/PX4/Firmware/tree/master/src/modules/mavlink)


### 설명
이 모듈은 MAVLink 프로토콜을 구현하며 시리얼 링크나 UDP 네트워크 연결에 사용할 수 있습니다.
uORB를 통한 시스템과 통신 : 일부 메시지는 모듈에서 직접 처리(예제 mission protocol)하며 다른 메시지는 uORB를 통해 publish(예제 vehicle_command).

스트림은 vehicle attitude와 같이 특정 rate로 주기적인 메시지를 전송합니다. mavlink instance를 시작시킬때, mode는 지정할 수 있고 각자의 rate와 함께 활성화된 스트림의 집합을 정의합니다.
실행 인스턴스에 대해서 스트림은 `mavlink stream` 명령을 통해 설정할 수 있습니다.

모듈의 다양한 독립적인 인스턴스가 있을 수 있는데, 시리얼 장치나 네트워크 포트에 연결될 수 있습니다.

### 구현
구현에서 수신/발신 thread 이렇게 2개 thread를 사용합니다. 발신자는 고정된 rate로 실행되고 결합된 대역폭이 설정한 rate(`-r`)보다 높거나 물리적 링크가 포화상태가 되면 동적으로 스트림의 rate를 줄입니다. `mavlink status`로 검사할 수 있으며 `rate mult`가 1보다 작은지를 살펴봅니다.

**주의**: 데이터의 일부는 양쪽 thread로 접근이나 수정이 됩니다. 따라서 코드를 바꾸거나 기능을 확장하는 경우 race condition이나 데이터가 잘못되는 일이 발생하지 않는지 고려해야 합니다.

### 예제
mavlink를 ttyS1에서 baudrate은 921600로 그리고 최대 전송 rete는 80kB/s로 시작합니다. :
```
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
```

UDP 포트 14556에서 mavlink를 시작시키고 HIGHRES_IMU 메시지는 50Hz로 활성화 시킴:
```
mavlink start -u 14556 -r 1000000
mavlink stream -u 14556 -s HIGHRES_IMU -r 50
```

### 사용법
```
mavlink <command> [arguments...]
 Commands:
   start         Start a new instance
     [-d <val>]  Select Serial Device
                 values: <file:dev>, default: /dev/ttyS1
     [-b <val>]  Baudrate
                 default: 57600
     [-r <val>]  Maximum sending data rate in B/s (if 0, use baudrate / 20)
                 default: 0
     [-u <val>]  Select UDP Network Port (local)
                 default: 14556
     [-o <val>]  Select UDP Network Port (remote)
                 default: 14550
     [-t <val>]  Partner IP (broadcasting can be enabled via MAV_BROADCAST
                 param)
                 default: 127.0.0.1
     [-m <val>]  Mode: sets default streams and rates
                 values: custom|camera|onboard|osd|magic|config|iridium,
                 default: normal
     [-f]        Enable message forwarding to other Mavlink instances
     [-v]        Verbose output
     [-w]        Wait to send, until first message received
     [-x]        Enable FTP

   stop-all      Stop all instances

   status        Print status for all instances

   stream        Configure the sending rate of a stream for a running instance
     [-u <val>]  Select Mavlink instance via local Network Port
                 default: 0
     [-d <val>]  Select Mavlink instance via Serial Device
                 values: <file:dev>
     -s <val>    Mavlink stream to configure
     -r <val>    Rate in Hz (0 = turn off)

   boot_complete Enable sending of messages. (Must be) called as last step in
                 startup script.
```
## uorb
소스: [modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)


### 설명
uORB는 내부 pub-sub 메시징 시스템으로 모듈간 통신에 사용됩니다.

가장 먼저 실행되는 모듈 중에 하나로 다른 모듈들이 이 모듈에 의존합니다.

### 구현
thread나 work queue가 필요하지 않습니다. 해당 모듈은 공유하는 global state를 초기화시키기 위해서만 시작합니다.
통신은 공유 메모리를 통해 이뤄집니다.
구현은 비동기로 lock-free 방식입니다. 예로 publisher와 subscriber는 서로 기다리지 않습니다.
이는 publisher와 subscriber가 각자 독립된 버퍼를 가지므로 가능합니다.

코드는 메모리 사용과 메시지 교환 지연시간을 최소화하도록 최적화되어 있습니다.

인터페이스는 file descriptors를 기반으로 합니다 : 내부적으로 `read`, `write` 그리고 `ioctl`를 사용합니다. publications을 제외하고 `orb_advert_t` 핸들을 사용합니다. 따라서 인터럽트로 사용할 수도 있습니다.(NuttX에서)

`/msg` 디렉토리에 메시지가 정의되어 있습니다. 빌드할때 C/C++ 코드로 변환됩니다.

만약 ORB_USE_PUBLISHER_RULES로 컴파일되면, uORB publication rule을 가진 파일은 어떤 모듈이 어떤 topic을 publish할 수 있는지 설정하는데 사용할 수 있습니다. system-wide replay에서 사용됩니다.

### 예제
topic publication rate를 감독합니다. `top`과 함께 일반 시스템 인스펙션을 위한 중요한 명령입니다 :
```
uorb top
```

### 사용법
```
uorb <command> [arguments...]
 Commands:
   start

   status        Print topic statistics

   top           Monitor topic publication rates
     [-a]        print all instead of only currently publishing topics
     [<filter1> [<filter2>]] topic(s) to match (implies -a)
```
