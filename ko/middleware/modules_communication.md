!REDIRECT "https://docs.px4.io/master/ko/middleware/modules_communication.html"

# 모듈 참조: 통신

## frsky_telemetry

Source: [drivers/telemetry/frsky_telemetry](https://github.com/PX4/Firmware/tree/master/src/drivers/telemetry/frsky_telemetry)

FrSky 통신을 지원합니다. D, S.PORT 프로토콜을 자동으로 감지합니다.
<a id="frsky_telemetry_usage"></a>

### Usage

    frsky_telemetry <command> [arguments...]
     Commands:
       start
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyS6
         [-t <val>]  Scanning timeout [s] (default: no timeout)
                     default: 0
         [-m <val>]  Select protocol (default: auto-detect)
                     values: sport|sport_single|sport_single_invert|dtype, default:
                     auto
    
       stop
    
       status
    

## mavlink

Source: [modules/mavlink](https://github.com/PX4/Firmware/tree/master/src/modules/mavlink)

### 설명

이 모듈은 시리얼 통신이나 UDP 통신에 사용할 수 있는 MAVLink 프로토콜 구현체입니다. uORB로 시스템과 통신합니다. 일부 메시지는 모듈에서 직접 처리합니다(예: mission protocol), 다른 메세지는 uORB로 처리합니다(예: vehicle_command).

기체 자세와 같은 주기 메세지는 일정 속도로 내보냅니다. mavlink 인스턴스를 시작할 때, 자체 속도로 송수신하는 활성 스트림을 정의한 모드를 지정할 수 있습니다. 실행 인스턴스에 대해 `mavlink stream` 명령으로 스트림을 설정할 수 있습니다.

직렬 통신 장치나 네트워크 포트에 연결한 다중 독립 인스턴스가 있을 수 있습니다.

### 구현

송신 스레드, 수신 스레드 각각 하나씩 2개로 구현했습니다. 송신 스레드는 일정한 속도로 실행하며, 설정 속도(`-r`)보다 대역폭이 높거나, 물리 연결 회선이 포화 상태가 되면 스트림 전송 속도를 동적으로 줄입니다. `mavlink status` 명령러를 통해 확인할 수 있습니다. `rate mult`가 1보다 작은지 보세요.

**Careful**: 일부 데이터는 양쪽 스레드에서 접근하고 수정합니다. 코드를 바꾸거나 기능을 확장할 경우 경쟁 상태(race condition)로의 진입과 데이터 유실을 피하기 위한 검토가 필요합니다.

### 예제

ttyS1에 전송율 초당 921600비트, 최대 전송속도 80kB/s로 mavlink를 시작합니다.

    mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
    

UDP 포트 14556에 HIGHRES_IMU 메시지를 초당 50번 전송하도록 설정하며 mavlink를 시작합니다.

    mavlink start -u 14556 -r 1000000
    mavlink stream -u 14556 -s HIGHRES_IMU -r 50
    

<a id="mavlink_usage"></a>

### Usage

    mavlink <command> [arguments...]
     Commands:
       start         Start a new instance
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyS1
         [-b <val>]  Baudrate (can also be p:<param_name>)
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
                     values:
                     custom|camera|onboard|osd|magic|config|iridium|minimal|extvsisi
                     on, default: normal
         [-n <val>]  wifi/ethernet interface name
                     values: <interface_name>
         [-c <val>]  Multicast address (multicasting can be enabled via
                     MAV_BROADCAST param)
                     values: Multicast address in the range
                     [239.0.0.0,239.255.255.255]
         [-f]        Enable message forwarding to other Mavlink instances
         [-w]        Wait to send, until first message received
         [-x]        Enable FTP
         [-z]        Force hardware flow control always on
         [-Z]        Force hardware flow control always off
    
       stop-all      Stop all instances
    
       status        Print status for all instances
         [streams]   Print all enabled streams
    
       stream        Configure the sending rate of a stream for a running instance
         [-u <val>]  Select Mavlink instance via local Network Port
         [-d <val>]  Select Mavlink instance via Serial Device
                     values: <file:dev>
         -s <val>    Mavlink stream to configure
         -r <val>    Rate in Hz (0 = turn off, -1 = set to default)
    
       boot_complete Enable sending of messages. (Must be) called as last step in
                     startup script.
    

## micrortps_client

Source: [modules/micrortps_bridge/micrortps_client](https://github.com/PX4/Firmware/tree/master/src/modules/micrortps_bridge/micrortps_client)

<a id="micrortps_client_usage"></a>

### Usage

    micrortps_client <command> [arguments...]
     Commands:
       start
         [-t <val>]  Transport protocol
                     values: UART|UDP, default: UART
         [-d <val>]  Select Serial Device
                     values: <file:dev>, default: /dev/ttyACM0
         [-b <val>]  Baudrate (can also be p:<param_name>)
                     default: 460800
         [-p <val>]  Poll timeout for UART in ms
         [-l <val>]  Limit number of iterations until the program exits
                     (-1=infinite)
                     default: 10000
         [-w <val>]  Time in ms for which each iteration sleeps
                     default: 1
         [-r <val>]  Select UDP Network Port for receiving (local)
                     default: 2019
         [-s <val>]  Select UDP Network Port for sending (remote)
                     default: 2020
         [-i <val>]  Select IP address (remote)
                     values: <x.x.x.x>, default: 127.0.0.1
         [-f]        Activate UART link SW flow control
         [-h]        Activate UART link HW flow control
         [-v]        Add more verbosity
    
       stop
    
       status
    

## uorb

Source: [modules/uORB](https://github.com/PX4/Firmware/tree/master/src/modules/uORB)

### 설명

uORB is the internal pub-sub messaging system, used for communication between modules.

It is typically started as one of the very first modules and most other modules depend on it.

### 구현

No thread or work queue is needed, the module start only makes sure to initialize the shared global state. Communication is done via shared memory. The implementation is asynchronous and lock-free, ie. a publisher does not wait for a subscriber and vice versa. This is achieved by having a separate buffer between a publisher and a subscriber.

The code is optimized to minimize the memory footprint and the latency to exchange messages.

The interface is based on file descriptors: internally it uses `read`, `write` and `ioctl`. Except for the publications, which use `orb_advert_t` handles, so that they can be used from interrupts as well (on NuttX).

Messages are defined in the `/msg` directory. They are converted into C/C++ code at build-time.

If compiled with ORB_USE_PUBLISHER_RULES, a file with uORB publication rules can be used to configure which modules are allowed to publish which topics. This is used for system-wide replay.

### 예제

Monitor topic publication rates. Besides `top`, this is an important command for general system inspection:

    uorb top
    

<a id="uorb_usage"></a>

### Usage

    uorb <command> [arguments...]
     Commands:
       start
    
       status        Print topic statistics
    
       top           Monitor topic publication rates
         [-a]        print all instead of only currently publishing topics with
                     subscribers
         [-1]        run only once, then exit
         [<filter1> [<filter2>]] topic(s) to match (implies -a)