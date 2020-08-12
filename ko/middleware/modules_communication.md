# 모듈 참조: 통신

## frsky_telemetry

Source: [drivers/telemetry/frsky_telemetry](https://github.com/PX4/Firmware/tree/master/src/drivers/telemetry/frsky_telemetry)

FrSky 통신을 지원합니다. D, S.PORT 프로토콜을 자동으로 감지합니다.

### Usage {#frsky_telemetry_usage}

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

### Description

이 모듈은 시리얼통신이나 UDP 통신에 사용될 수 있는 MAVLink 프로토콜을 구현한 것입니다. uORB를 통해 시스템과 통신합니다. 몇몇의 메시지는 이 모듈에서 처리합니다. 예를 들면, mission protocol 입니다. 다른 메시지들은 uROB를 통해 퍼블리시됩니다. 예를 들면, vehicle_command 입니다.

스트림은 특정한 속도로 주기적인 메시지(예, 기체 자세)를 보내기 위해 사용됩니다. mavlink 인스턴스를 시작할 때 특정 속도로 스트림을 활성화 할지 정의하는 모드를 설정할 수 있습니다. 인스터스가 실행중 일때는 `mavlink stream` 명령어를 통해 설정할 수 있습니다.

시리얼 디바이스나 네트워크 포트에 연결된 여러개의 독립적인 인스턴스가 있을 수 있습니다. 

### Implementation

송신하는 쓰레드, 수신하는 쓰레드 2개를 사용해 구현했습니다. 송신 쓰레드는 일정한 속도로 수행되고 설정한 속도(`-r`)보다 대역폭이 높아지거나 물리적인 연결이 포화가 되면 스트림의 속도는 동적으로 줄어듭니다. `mavlink status` 명령러를 통해 확인할 수 있습니다. `rate mult`가 1보다 작은지 보세요.

**Careful**: 일부 데이터는 두개의 쓰레드 모두에서 접근되고 수저됩니다. 따라서 코드를 바꾸거나 기능을 확장할 때는 레이스 컨디션과 데이터가 오염되는 것을 고려해야합니다.

### Examples

ttyS1에 baudrate 921600, 최대 전송속도 80kB/s로 mavlink를 시작합니다.

    mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
    

UDP 포트 14556에 HIGHRES_IMU 메시지를 50Hz속도로 활성화해 mavlink를 시작합니다.

    mavlink start -u 14556 -r 1000000
    mavlink stream -u 14556 -s HIGHRES_IMU -r 50
    

### Usage {#mavlink_usage}

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
         [-z]        Force flow control always on
    
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

### Usage {#micrortps_client_usage}

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

uORB는 모듈간의 통신을 위해 사용되는 내부적인 Pub/Sub 메시징 시스템입니다.

일반적으로 첫 번째로 시작되는 모듈이며 다른 모듈들이 이 모듈에 의존합니다.

### 구현

스레드나 작업 큐는 필요하지 않습니다. 모듈 시작시 공유 광역 상태의 초기화 여부만 확인합니다. 공유 메모리로 통신합니다. 비동기 방식으로 잠금 구현을 배제하여 구현했습니다. 예를 들어, 퍼블리셔와 섭스크라이버는 서로를 기다릴 필요하 없습니다. 이것은 퍼블리셔와 섭스크라이버가 독립적인 버퍼를 가짐으로써 이뤄집니다.

메모리 점유 영역과 메시지 교환 지연을 최소화 하도록 코드를 최적화했습니다.

인터페이스는 내부적으로 `read`, `write`,`ioctl`을 사용하며, 파일 서술자를 기반으로 동작합니다. `orb_advert_t` 핸들을 사용하는 퍼블리시를 제외하고는 NuttX에서는 인터럽트에서도 사용할 수 있습니다.

`/msg` 디렉토리에 메시지들이 정의되어있습니다. 빌드할때 C/C++ 코드로 변환됩니다.

ORB_USE_PUBLISHER_RULES 설정 값을 넣어 컴파일하면, uORB 전송 규칙이 들어있는 파일을 어떤 모듈에서 어떤 토픽을 내보낼지 설정할 목적으로 활용할 수 있습니다. 이 설정은 시스템 영역의 동작 재현에 활용합니다. 

### 예제

토픽 송신 속도를 감시합니다. `top` 과 함께 전반적인 시스템을 관찰할 수 있는 중요한 명령어입니다.

    uorb top
    

### Usage {#uorb_usage}

    uorb <command> [arguments...]
     Commands:
       start
    
       status        Print topic statistics
    
       top           Monitor topic publication rates
         [-a]        print all instead of only currently publishing topics with
                     subscribers
         [-1]        run only once, then exit
         [<filter1> [<filter2>]] topic(s) to match (implies -a)